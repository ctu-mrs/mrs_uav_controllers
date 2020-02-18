#define VERSION "0.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_uav_manager/Controller.h>

#include <mrs_controllers/so3_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>
#include <mrs_lib/mutex.h>

#include <geometry_msgs/Vector3Stamped.h>

//}

#define X 0
#define Y 1
#define Z 2

#define OUTPUT_ATTITUDE_RATE 1
#define OUTPUT_ATTITUDE_QUATERNION 2

namespace mrs_controllers
{

namespace so3_controller
{

/* //{ class So3Controller */

class So3Controller : public mrs_uav_manager::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void dynamicReconfigureCallback(mrs_controllers::so3_controllerConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  std::string _version_;

  bool is_initialized = false;
  bool is_active      = false;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

private:
  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                        config_mutex_;
  typedef mrs_controllers::so3_controllerConfig Config;
  typedef dynamic_reconfigure::Server<Config>   ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>          reconfigure_server_;
  void                                          drs_callback(mrs_controllers::so3_controllerConfig &config, uint32_t level);
  mrs_controllers::so3_controllerConfig         drs_params;

private:
  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;

  // actual gains (used and already filtered)
  double kpxy, kiwxy, kibxy, kvxy, kaxy;
  double kpz, kvz, kaz;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;
  double kqxy, kqz;  // attitude gains
  double kwxy, kwz;  // attitude rate gains

  // desired gains (set by DRS)
  std::mutex mutex_gains;
  std::mutex mutex_drs_params;

  double tilt_angle_saturation_;
  double tilt_angle_failsafe_;
  double thrust_saturation_;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;

  bool   mute_lateral_gains               = false;
  bool   mutex_lateral_gains_after_toggle = false;
  double mute_coefficitent_;

private:
  mrs_lib::Profiler profiler;
  bool              profiler_enabled_ = false;

private:
  ros::Timer timer_gain_filter;
  void       timerGainsFilter(const ros::TimerEvent &event);

  int    gains_filter_timer_rate_;
  double gains_filter_change_rate_;
  double gains_filter_min_change_rate_;

  double gains_filter_max_change_;  // calculated from change_rate_/timer_rate_;
  double gains_filter_min_change_;  // calculated from change_rate_/timer_rate_;

private:
  int        output_mode_;  // 1 = ATTITUDE RATES, 2 = ATTITUDE QUATERNION
  std::mutex mutex_output_mode;

private:
  Eigen::Vector2d Ib_b;  // body error integral in the body frame
  Eigen::Vector2d Iw_w;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

private:
  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_thrust_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time;
  ros::Time rampup_last_time_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void So3Controller::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                               const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                               std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;
  this->uav_mass_     = uav_mass;
  this->g_            = g;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "So3Controller");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[So3Controller]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // lateral gains
  param_loader.load_param("default_gains/horizontal/kp", kpxy);
  param_loader.load_param("default_gains/horizontal/kv", kvxy);
  param_loader.load_param("default_gains/horizontal/ka", kaxy);

  param_loader.load_param("default_gains/horizontal/kiw", kiwxy);
  param_loader.load_param("default_gains/horizontal/kib", kibxy);

  param_loader.load_param("lateral_mute_coefficitent", mute_coefficitent_);

  // | ------------------------- rampup ------------------------- |

  param_loader.load_param("rampup/enabled", _rampup_enabled_);
  param_loader.load_param("rampup/speed", _rampup_speed_);

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz);
  param_loader.load_param("default_gains/vertical/kv", kvz);
  param_loader.load_param("default_gains/vertical/ka", kaz);

  // attitude gains
  param_loader.load_param("default_gains/horizontal/attitude/kq", kqxy);
  param_loader.load_param("default_gains/vertical/attitude/kq", kqz);

  // attitude rate gains
  param_loader.load_param("default_gains/horizontal/attitude/kw", kwxy);
  param_loader.load_param("default_gains/vertical/attitude/kw", kwz);

  // mass estimator
  param_loader.load_param("default_gains/weight_estimator/km", km);
  param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim);

  // integrator limits
  param_loader.load_param("default_gains/horizontal/kiw_lim", kiwxy_lim);
  param_loader.load_param("default_gains/horizontal/kib_lim", kibxy_lim);

  // constraints
  param_loader.load_param("tilt_angle_saturation", tilt_angle_saturation_);
  param_loader.load_param("tilt_angle_failsafe", tilt_angle_failsafe_);
  param_loader.load_param("thrust_saturation", thrust_saturation_);

  // gain filtering
  param_loader.load_param("gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", gains_filter_min_change_rate_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  // output mode
  param_loader.load_param("output_mode", output_mode_);

  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[So3Controller]: output mode has to be {1, 2}!");
  }

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[So3Controller]: Could not load all parameters!");
    ros::shutdown();
  }

  // convert to radians
  tilt_angle_saturation_ = (tilt_angle_saturation_ / 180) * M_PI;
  tilt_angle_failsafe_   = (tilt_angle_failsafe_ / 180) * M_PI;

  uav_mass_difference = 0;
  Iw_w                = Eigen::Vector2d::Zero(2);
  Ib_b                = Eigen::Vector2d::Zero(2);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_params.kpxy        = kpxy;
  drs_params.kvxy        = kvxy;
  drs_params.kaxy        = kaxy;
  drs_params.kiwxy       = kiwxy;
  drs_params.kibxy       = kibxy;
  drs_params.kpz         = kpz;
  drs_params.kvz         = kvz;
  drs_params.kaz         = kaz;
  drs_params.kqxy        = kqxy;
  drs_params.kqz         = kqz;
  drs_params.kwxy        = kwxy;
  drs_params.kwz         = kwz;
  drs_params.kiwxy_lim   = kiwxy_lim;
  drs_params.kibxy_lim   = kibxy_lim;
  drs_params.km          = km;
  drs_params.km_lim      = km_lim;
  drs_params.output_mode = output_mode_;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_params);
  ReconfigureServer::CallbackType f = boost::bind(&So3Controller::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = mrs_lib::Profiler(nh_, "So3Controller", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &So3Controller::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[So3Controller]: Could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("[So3Controller]: initialized, version %s", VERSION);

  is_initialized = true;
}

//}

/* //{ activate() */

bool So3Controller::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[So3Controller]: activated without getting the last controller's command.");

    return false;

  } else {

    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;

    activation_control_command_.controller_enforcing_constraints = false;

    Ib_b[0] = cmd->disturbance_bx_b;
    Ib_b[1] = cmd->disturbance_by_b;

    Iw_w[0] = cmd->disturbance_wx_w;
    Iw_w[1] = cmd->disturbance_wy_w;

    ROS_INFO(
        "[So3Controller]: setting the mass difference and disturbances from the last AttitudeCmd: mass difference: %.2f kg, Ib_b: %.2f, %.2f N, Iw_w: %.2f, "
        "%.2f N",
        uav_mass_difference, Ib_b[0], Ib_b[1], Iw_w[0], Iw_w[1]);

    ROS_INFO("[So3Controller]: activated with a last controller's command, mass difference %.2f kg.", uav_mass_difference);
  }

  // rampup check
  if (_rampup_enabled_) {

    double hover_thrust      = sqrt(cmd->total_mass * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
    double thrust_difference = hover_thrust - cmd->thrust;

    if (thrust_difference > 0) {
      rampup_direction_ = 1;
    } else if (thrust_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[So3Controller]: activating rampup with initial thrust: %.4f, target: %.4f", cmd->thrust, hover_thrust);

    rampup_active_    = true;
    rampup_start_time = ros::Time::now();
    rampup_last_time_ = ros::Time::now();
    rampup_thrust_    = cmd->thrust;

    rampup_duration_ = fabs(thrust_difference) / _rampup_speed_;
  }

  first_iteration = true;

  ROS_INFO("[So3Controller]: activated");

  is_active = true;

  return true;
}  // namespace so3_controller

//}

/* //{ deactivate() */

void So3Controller::deactivate(void) {

  is_active           = false;
  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[So3Controller]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr So3Controller::update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("update");

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }

  if (!is_active) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = uav_state->header.stamp;

    first_iteration = false;

    ROS_INFO("[So3Controller]: first iteration");

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (uav_state->header.stamp - last_update).toSec();
    last_update = uav_state->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[So3Controller]: the last odometry message came too close! %f", dt);

    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  // --------------------------------------------------------------
  // |                 calculate the euler angles                 |
  // --------------------------------------------------------------

  double         uav_yaw, uav_pitch, uav_roll;
  tf::Quaternion uav_attitude;
  quaternionMsgToTF(uav_state->pose.orientation, uav_attitude);
  tf::Matrix3x3 m(uav_attitude);
  m.getRPY(uav_roll, uav_pitch, uav_yaw);

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rw = Eigen::Vector3d::Zero(3);

  Eigen::Quaternion<double> Rq;

  Eigen::Matrix3d Rd;

  if (reference->use_position_vertical || reference->use_position_horizontal) {

    if (reference->use_position_horizontal) {
      Rp[0] = reference->position.x;
      Rp[1] = reference->position.y;
    } else {
      Rv[0] = 0;
      Rv[1] = 0;
    }

    if (reference->use_position_vertical) {
      Rp[2] = reference->position.z;
    } else {
      Rv[2] = 0;
    }
  }

  if (reference->use_velocity_horizontal) {
    Rv[0] = reference->velocity.x;
    Rv[1] = reference->velocity.y;
  } else {
    Rv[0] = 0;
    Rv[1] = 0;
  }

  if (reference->use_velocity_vertical) {
    Rv[2] = reference->velocity.z;
  } else {
    Rv[2] = 0;
  }

  if (reference->use_acceleration) {
    Ra << reference->acceleration.x, reference->acceleration.y, reference->acceleration.z;
  } else {
    Ra << 0, 0, 0;
  }

  if (reference->use_yaw) {
    Rq.coeffs() << 0, 0, sin(reference->yaw / 2.0), cos(reference->yaw / 2.0);
  } else {
    Rq.coeffs() << 0, 0, sin(uav_yaw / 2.0), cos(uav_yaw / 2.0);
  }

  if (reference->use_attitude_rate) {
    Rw << reference->attitude_rate.x, reference->attitude_rate.y, reference->attitude_rate.z;
  } else if (reference->use_yaw_dot) {
    Rw << 0, 0, reference->yaw_dot;
  }

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << uav_state->pose.orientation.x, uav_state->pose.orientation.y, uav_state->pose.orientation.z, uav_state->pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  // position control error
  Eigen::Vector3d Ep = Eigen::Vector3d::Zero(3);

  if (reference->use_position_horizontal || reference->use_position_vertical) {
    Ep = Op - Rp;
  }

  // velocity control error
  Eigen::Vector3d Ev = Eigen::Vector3d::Zero(3);

  if (reference->use_velocity_horizontal || reference->use_velocity_vertical) {
    Ev = Ov - Rv;
  }

  // --------------------------------------------------------------
  // |                            gains                           |
  // --------------------------------------------------------------

  Eigen::Vector3d Ka = Eigen::Vector3d::Zero(3);
  Eigen::Array3d  Kp = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kv = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kq = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kw = Eigen::Array3d::Zero(3);

  {
    std::scoped_lock lock(mutex_gains);

    if (reference->use_position_horizontal) {
      Kp[0] = kpxy;
      Kp[1] = kpxy;
    } else {
      Kp[0] = 0;
      Kp[1] = 0;
    }

    if (reference->use_position_vertical) {
      Kp[2] = kpz;
    } else {
      Kp[2] = 0;
    }

    if (reference->use_velocity_horizontal) {
      Kv[0] = kvxy;
      Kv[1] = kvxy;
    } else {
      Kv[0] = 0;
      Kv[1] = 0;
    }

    if (reference->use_velocity_vertical) {
      Kv[2] = kvz;
    } else {
      Kv[2] = 0;
    }

    if (reference->use_acceleration) {
      Ka << kaxy, kaxy, kaz;
    } else {
      Ka << 0, 0, 0;
    }

    Kq << kqxy, kqxy, kqz;

    if (!reference->use_yaw) {
      Kq[2] = 0;
    }

    Kw << kwxy, kwxy, kwz;
  }

  Kp = Kp * (uav_mass_ + uav_mass_difference);
  Kv = Kv * (uav_mass_ + uav_mass_difference);

  // --------------------------------------------------------------
  // |                 desired orientation matrix                 |
  // --------------------------------------------------------------

  Eigen::Vector2d Ib_w = rotate2d(Ib_b, uav_yaw);

  double total_mass = uav_mass_ + uav_mass_difference;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, g_) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array();
  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w[0], Ib_w[1] + Iw_w[1], 0;
  }

  Eigen::Vector3d f = position_feedback + velocity_feedback + integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[So3Controller]: the calculated downwards desired force is negative (%.2f) -> mitigating the flip", f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in the spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[So3Controller]: NaN detected in variable \"theta\", returning null");

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (tilt_angle_failsafe_ > 1e-3 && theta > tilt_angle_failsafe_) {

    ROS_ERROR("[So3Controller]: The produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta,
              (180.0 / M_PI) * tilt_angle_failsafe_);
    ROS_INFO("[So3Controller]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
    ROS_INFO("[So3Controller]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
    ROS_INFO("[So3Controller]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
    ROS_INFO("[So3Controller]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[So3Controller]: position_cmd: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", reference->position.x, reference->position.y, reference->position.z,
             reference->yaw);
    ROS_INFO("[So3Controller]: odometry: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", uav_state->pose.position.x, uav_state->pose.position.y,
             uav_state->pose.position.z, uav_yaw);

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle
  if (tilt_angle_saturation_ > 1e-3 && theta > tilt_angle_saturation_) {
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: tilt is being saturated, desired: %f deg, saturated %f deg", (theta / M_PI) * 180.0,
                      (tilt_angle_saturation_ / M_PI) * 180.0);
    theta = tilt_angle_saturation_;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

  // | ------------- construct the rotational matrix ------------ |

  Rd.col(2) = f_norm;
  Rd.col(1) = Rd.col(2).cross(Rq.toRotationMatrix().col(0));
  Rd.col(1).normalize();
  Rd.col(0) = Rd.col(1).cross(Rd.col(2));

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------

  /* orientation error */
  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;
  Eq << (E(2, 1) - E(1, 2)) / 2.0, (E(0, 2) - E(2, 0)) / 2.0, (E(1, 0) - E(0, 1)) / 2.0;

  // --------------------------------------------------------------
  // |                     angular rate error                     |
  // --------------------------------------------------------------
  //
  Eigen::Vector3d Ew;
  Ew = R.transpose() * (Ow - Rw);

  /* output */
  double thrust_force = f.dot(R.col(2));

  double thrust = 0;

  if (thrust_force >= 0) {
    thrust = sqrt(thrust_force) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
  } else {
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: Just so you know, the desired thrust force is negative (%.2f)", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[So3Controller]: NaN detected in variable \"thrust\", setting it to 0 and returning!!!");

  } else if (thrust > thrust_saturation_) {

    thrust = thrust_saturation_;
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: saturating thrust to %.2f", thrust_saturation_);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: saturating thrust to 0");
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  if (mute_lateral_gains && !reference->disable_position_gains) {
    mutex_lateral_gains_after_toggle = true;
  }
  mute_lateral_gains = reference->disable_position_gains;

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (reference->use_position_horizontal) {
      Iw_w -= kiwxy * Ep.head(2) * dt;
    } else if (reference->use_velocity_horizontal) {
      Iw_w -= kiwxy * Ev.head(2) * dt;
    }

    // saturate the world
    double world_integral_saturated = false;
    if (!std::isfinite(Iw_w[0])) {
      Iw_w[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Iw_w[0]\", setting it to 0!!!");
    } else if (Iw_w[0] > kiwxy_lim) {
      Iw_w[0]                  = kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w[0] < -kiwxy_lim) {
      Iw_w[0]                  = -kiwxy_lim;
      world_integral_saturated = true;
    }

    if (kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's world X integral is being saturated!");
    }

    // saturate the world
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w[1])) {
      Iw_w[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Iw_w[1]\", setting it to 0!!!");
    } else if (Iw_w[1] > kiwxy_lim) {
      Iw_w[1]                  = kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w[1] < -kiwxy_lim) {
      Iw_w[1]                  = -kiwxy_lim;
      world_integral_saturated = true;
    }

    if (kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's world Y integral is being saturated!");
    }
  }

  //}

  /* body error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the body error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    // rotate the control errors to the body
    Eigen::Vector2d Ep_body = rotate2d(Ep.head(2), -uav_yaw);
    Eigen::Vector2d Ev_body = rotate2d(Ev.head(2), -uav_yaw);

    // integrate the body error
    if (reference->use_position_horizontal) {
      Ib_b -= kibxy * Ep_body * dt;
    } else if (reference->use_velocity_horizontal) {
      Ib_b -= kibxy * Ev_body * dt;
    }

    // saturate the body
    double body_integral_saturated = false;
    if (!std::isfinite(Ib_b[0])) {
      Ib_b[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Ib_b[0]\", setting it to 0!!!");
    } else if (Ib_b[0] > kibxy_lim) {
      Ib_b[0]                 = kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b[0] < -kibxy_lim) {
      Ib_b[0]                 = -kibxy_lim;
      body_integral_saturated = true;
    }

    if (kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b[1])) {
      Ib_b[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Ib_b[1]\", setting it to 0!!!");
    } else if (Ib_b[1] > kibxy_lim) {
      Ib_b[1]                 = kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b[1] < -kibxy_lim) {
      Ib_b[1]                 = -kibxy_lim;
      body_integral_saturated = true;
    }

    if (kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's body roll integral is being saturated!");
    }
  }

  //}

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    if (reference->use_position_vertical && !rampup_active_) {
      uav_mass_difference -= km * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference)) {
      uav_mass_difference = 0;
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!");
    } else if (uav_mass_difference > km_lim) {
      uav_mass_difference = km_lim;
      uav_mass_saturated  = true;
    } else if (uav_mass_difference < -km_lim) {
      uav_mass_difference = -km_lim;
      uav_mass_saturated  = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: The uav_mass_difference is being saturated to %1.3f!", uav_mass_difference);
    }
  }

  //}

  // --------------------------------------------------------------
  // |            report on the values of the integrals           |
  // --------------------------------------------------------------

  ROS_INFO_THROTTLE(5.0, "[So3Controller]: world error integral: x %0.2f N, y %0.2f N, lim: %0.2f N", Iw_w[X], Iw_w[Y], kiwxy_lim);
  ROS_INFO_THROTTLE(5.0, "[So3Controller]: body error integral:  x %0.2f N, y %0.2f N, lim: %0.2f N", Ib_b[X], Ib_b[Y], kibxy_lim);

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // --------------------------------------------------------------
  // |              compensated desired acceleration              |
  // --------------------------------------------------------------

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Quaterniond des_quater = Eigen::Quaterniond(Rd);

    // rotate the drone's z axis
    Eigen::Vector3d uav_z_in_world = des_quater * Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d thrust_vector = thrust_force * uav_z_in_world;

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w[1] / total_mass) - (Ib_w[1] / total_mass);
    double world_accel_z = reference->acceleration.z;

    geometry_msgs::Vector3Stamped world_accel;
    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state->header.frame_id;
    world_accel.vector.x        = world_accel_x;
    world_accel.vector.y        = world_accel_y;
    world_accel.vector.z        = world_accel_z;

    auto res = common_handlers_->transformer->transformSingle("fcu", world_accel);

    if (res) {

      desired_x_accel = res.value().vector.x;
      desired_y_accel = res.value().vector.y;
      desired_z_accel = res.value().vector.z;
    }
  }

  // | --------------- fill the resulting command --------------- |

  {
    std::scoped_lock lock(mutex_output_mode);

    if (output_mode_ == OUTPUT_ATTITUDE_RATE) {

      // output the desired attitude rate
      output_command->attitude_rate.x   = t[0];
      output_command->attitude_rate.y   = t[1];
      output_command->attitude_rate.z   = t[2];
      output_command->attitude_rate_set = true;

      Eigen::Quaterniond thrust_vec       = Eigen::Quaterniond(Rd);
      output_command->quater_attitude.w   = thrust_vec.w();
      output_command->quater_attitude.x   = thrust_vec.x();
      output_command->quater_attitude.y   = thrust_vec.y();
      output_command->quater_attitude.z   = thrust_vec.z();
      output_command->quater_attitude_set = true;

      output_command->euler_attitude_set = false;

      output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

    } else if (output_mode_ == OUTPUT_ATTITUDE_QUATERNION) {

      // output the desired attitude
      Eigen::Quaterniond thrust_vec       = Eigen::Quaterniond(Rd);
      output_command->quater_attitude.w   = thrust_vec.w();
      output_command->quater_attitude.x   = thrust_vec.x();
      output_command->quater_attitude.y   = thrust_vec.y();
      output_command->quater_attitude.z   = thrust_vec.z();
      output_command->quater_attitude_set = true;

      output_command->euler_attitude_set = false;
      output_command->attitude_rate_set  = false;

      output_command->mode_mask = output_command->MODE_QUATER_ATTITUDE;

      ROS_WARN_THROTTLE(1.0, "[So3Controller]: outputting attitude quaternion");
    }

    output_command->desired_acceleration.x = desired_x_accel;
    output_command->desired_acceleration.y = desired_y_accel;
    output_command->desired_acceleration.z = desired_z_accel;
  }

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time).toSec()) >= rampup_duration_) {

      rampup_active_         = false;
      output_command->thrust = thrust;

      ROS_INFO("[So3Controller]: rampup finished");

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_thrust_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      output_command->thrust = rampup_thrust_;

      ROS_INFO_THROTTLE(0.1, "[So3Controller]: ramping up thrust, %.4f", output_command->thrust);
    }

  } else {
    output_command->thrust = thrust;
  }

  output_command->ramping_up = rampup_active_;

  output_command->mass_difference = uav_mass_difference;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = Ib_b[0];
  output_command->disturbance_by_b = Ib_b[1];

  output_command->disturbance_bx_w = Ib_w[0];
  output_command->disturbance_by_w = Ib_w[1];

  output_command->disturbance_wx_w = Iw_w[0];
  output_command->disturbance_wy_w = Iw_w[1];

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "So3Controller";

  last_output_command = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus So3Controller::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void So3Controller::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg) {

  ROS_INFO("[So3Controller]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w[0];
  world_integrals.vector.y = Iw_w[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(msg->header.frame_id, world_integrals);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w[0] = res.value().vector.x;
    Iw_w[1] = res.value().vector.y;
  } else {

    ROS_ERROR_THROTTLE(1.0, "[So3Controller]: could not transform world integral to the new frame");

    std::scoped_lock lock(mutex_integrals_);

    Iw_w[0] = 0;
    Iw_w[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void So3Controller::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w = Eigen::Vector2d::Zero(2);
  Ib_b = Eigen::Vector2d::Zero(2);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void So3Controller::dynamicReconfigureCallback(mrs_controllers::so3_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params, mutex_output_mode);

    drs_params = config;

    output_mode_ = config.output_mode;
  }

  ROS_INFO("[So3Controller]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void So3Controller::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.05, event);

  double gain_coeff                = 1;
  bool   bypass_filter             = mute_lateral_gains || mutex_lateral_gains_after_toggle;
  mutex_lateral_gains_after_toggle = false;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains, mutex_drs_params);

    kpxy      = calculateGainChange(kpxy, drs_params.kpxy * gain_coeff, bypass_filter, "kpxy");
    kvxy      = calculateGainChange(kvxy, drs_params.kvxy * gain_coeff, bypass_filter, "kvxy");
    kaxy      = calculateGainChange(kaxy, drs_params.kaxy * gain_coeff, bypass_filter, "kaxy");
    kiwxy     = calculateGainChange(kiwxy, drs_params.kiwxy * gain_coeff, bypass_filter, "kiwxy");
    kibxy     = calculateGainChange(kibxy, drs_params.kibxy * gain_coeff, bypass_filter, "kibxy");
    kpz       = calculateGainChange(kpz, drs_params.kpz, false, "kpz");
    kvz       = calculateGainChange(kvz, drs_params.kvz, false, "kvz");
    kaz       = calculateGainChange(kaz, drs_params.kaz, false, "kaz");
    kqxy      = calculateGainChange(kqxy, drs_params.kqxy, false, "kqxy");
    kqz       = calculateGainChange(kqz, drs_params.kqz, false, "kqz");
    kwxy      = calculateGainChange(kwxy, drs_params.kwxy, false, "kwxy");
    kwz       = calculateGainChange(kwz, drs_params.kwz, false, "kwz");
    km        = calculateGainChange(km, drs_params.km, false, "km");
    kiwxy_lim = calculateGainChange(kiwxy_lim, drs_params.kiwxy_lim, false, "kiwxy_lim");
    kibxy_lim = calculateGainChange(kibxy_lim, drs_params.kibxy_lim, false, "kibxy_lim");
    km_lim    = calculateGainChange(km_lim, drs_params.km_lim, false, "km_lim");
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* calculateGainChange() //{ */

double So3Controller::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

  double change = desired_value - current_value;

  if (!bypass_rate) {

    // if current value is near 0...
    double change_in_perc;
    double saturated_change;

    if (fabs(current_value) < 1e-6) {
      change *= gains_filter_max_change_;
    } else {

      saturated_change = change;

      change_in_perc = (current_value + saturated_change) / current_value - 1.0;

      if (change_in_perc > gains_filter_max_change_) {
        saturated_change = current_value * gains_filter_max_change_;
      } else if (change_in_perc < -gains_filter_max_change_) {
        saturated_change = current_value * -gains_filter_max_change_;
      }

      if (fabs(saturated_change) < fabs(change) * gains_filter_min_change_) {
        change *= gains_filter_min_change_;
      } else {
        change = saturated_change;
      }
    }
  }

  if (fabs(change) > 1e-3) {
    ROS_INFO_THROTTLE(1.0, "[So3Controller]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* rotate2d() //{ */

Eigen::Vector2d So3Controller::rotate2d(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace so3_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::so3_controller::So3Controller, mrs_uav_manager::Controller)

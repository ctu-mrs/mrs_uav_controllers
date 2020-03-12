#define VERSION "0.0.5.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <mrs_msgs/AttitudeCommand.h>

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
  void initialize(const ros::NodeHandle& parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr& cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr& uav_state, const mrs_msgs::PositionCommand::ConstPtr& reference);
  const mrs_msgs::ControllerStatus          getStatus();

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool& updated);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr& msg);

  void resetDisturbanceEstimators(void);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef mrs_controllers::so3_controllerConfig    DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(mrs_controllers::so3_controllerConfig& config, uint32_t level);
  DrsConfig_t                                      drs_gains_;

  // | ---------- thrust generation and mass estimation --------- |

  double                       _uav_mass_;
  double                       uav_mass_difference_;
  double                       _g_;
  mrs_uav_manager::MotorParams _motor_params_;

  // gains that are used and already filtered
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
  double kaxy_;       // acceleration xy gain (feed forward, =1)
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double kpz_;        // position z gain
  double kvz_;        // velocity z gain
  double kaz_;        // acceleration z gain (feed forward, =1)
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kqxy_;       // pitch/roll attitude gain
  double kqz_;        // yaw attitude gain
  double kwxy_;       // pitch/roll attitude rate gain
  double kwz_;        // yaw attitude rate gain

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  // | ----------------------- gain muting ---------------------- |

  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;

  // | --------------------- gain filtering --------------------- |

  void filterGains(const bool mute_gains, const double dt);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ------------ controller limits and saturations ----------- |

  double _tilt_angle_saturation_;
  double _tilt_angle_failsafe_;
  double _thrust_saturation_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  int        output_mode_;  // 1 = ATTITUDE RATES, 2 = ATTITUDE QUATERNION
  std::mutex mutex_output_mode_;

  // | ------------------------ profiler_ ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

  // | ------------------------- rampup ------------------------- |

  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_thrust_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time_;
  ros::Time rampup_last_time_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void So3Controller::initialize(const ros::NodeHandle& parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                               const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                               std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _motor_params_   = motor_params;
  _uav_mass_       = uav_mass;
  _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "So3Controller");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[So3Controller]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  // lateral gains
  param_loader.load_param("default_gains/horizontal/kp", kpxy_);
  param_loader.load_param("default_gains/horizontal/kv", kvxy_);
  param_loader.load_param("default_gains/horizontal/ka", kaxy_);

  param_loader.load_param("default_gains/horizontal/kiw", kiwxy_);
  param_loader.load_param("default_gains/horizontal/kib", kibxy_);

  param_loader.load_param("gain_mute_coefficient", _gain_mute_coefficient_);

  // | ------------------------- rampup ------------------------- |

  param_loader.load_param("rampup/enabled", _rampup_enabled_);
  param_loader.load_param("rampup/speed", _rampup_speed_);

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz_);
  param_loader.load_param("default_gains/vertical/kv", kvz_);
  param_loader.load_param("default_gains/vertical/ka", kaz_);

  // attitude gains
  param_loader.load_param("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.load_param("default_gains/vertical/attitude/kq", kqz_);

  // attitude rate gains
  param_loader.load_param("default_gains/horizontal/attitude/kw", kwxy_);
  param_loader.load_param("default_gains/vertical/attitude/kw", kwz_);

  // mass estimator
  param_loader.load_param("default_gains/weight_estimator/km", km_);
  param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim_);

  // integrator limits
  param_loader.load_param("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.load_param("default_gains/horizontal/kib_lim", kibxy_lim_);

  // constraints
  param_loader.load_param("tilt_angle_saturation", _tilt_angle_saturation_);
  param_loader.load_param("tilt_angle_failsafe", _tilt_angle_failsafe_);
  param_loader.load_param("thrust_saturation", _thrust_saturation_);

  // gain filtering
  param_loader.load_param("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // output mode
  param_loader.load_param("output_mode", output_mode_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[So3Controller]: could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[So3Controller]: output mode has to be {1, 2}!");
    ros::shutdown();
  }

  // convert to radians
  _tilt_angle_saturation_ = (_tilt_angle_saturation_ / 180.0) * M_PI;
  _tilt_angle_failsafe_   = (_tilt_angle_failsafe_ / 180.0) * M_PI;

  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | --------------- dynamic reconfigure server --------------- |

  drs_gains_.kpxy        = kpxy_;
  drs_gains_.kvxy        = kvxy_;
  drs_gains_.kaxy        = kaxy_;
  drs_gains_.kiwxy       = kiwxy_;
  drs_gains_.kibxy       = kibxy_;
  drs_gains_.kpz         = kpz_;
  drs_gains_.kvz         = kvz_;
  drs_gains_.kaz         = kaz_;
  drs_gains_.kqxy        = kqxy_;
  drs_gains_.kqz         = kqz_;
  drs_gains_.kwxy        = kwxy_;
  drs_gains_.kwz         = kwz_;
  drs_gains_.kiwxy_lim   = kiwxy_lim_;
  drs_gains_.kibxy_lim   = kibxy_lim_;
  drs_gains_.km          = km_;
  drs_gains_.km_lim      = km_lim_;
  drs_gains_.output_mode = output_mode_;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_gains_);
  Drs_t::CallbackType f = boost::bind(&So3Controller::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "So3Controller", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[So3Controller]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool So3Controller::activate(const mrs_msgs::AttitudeCommand::ConstPtr& cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[So3Controller]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *cmd;
    uav_mass_difference_     = cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    Ib_b_[0] = cmd->disturbance_bx_b;
    Ib_b_[1] = cmd->disturbance_by_b;

    Iw_w_[0] = cmd->disturbance_wx_w;
    Iw_w_[1] = cmd->disturbance_wy_w;

    ROS_INFO(
        "[So3Controller]: setting the mass difference and disturbances from the last AttitudeCmd: mass difference: %.2f kg, Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, "
        "%.2f N",
        uav_mass_difference_, Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);

    ROS_INFO("[So3Controller]: activated with a last controller's command, mass difference %.2f kg", uav_mass_difference_);
  }

  // rampup check
  if (_rampup_enabled_) {

    double hover_thrust      = sqrt(cmd->total_mass * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;
    double thrust_difference = hover_thrust - cmd->thrust;

    if (thrust_difference > 0) {
      rampup_direction_ = 1;
    } else if (thrust_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[So3Controller]: activating rampup with initial thrust: %.4f, target: %.4f", cmd->thrust, hover_thrust);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_thrust_     = cmd->thrust;

    rampup_duration_ = fabs(thrust_difference) / _rampup_speed_;
  }

  first_iteration_ = true;
  gains_muted_     = true;

  ROS_INFO("[So3Controller]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void So3Controller::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[So3Controller]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr So3Controller::update(const mrs_msgs::UavState::ConstPtr&        uav_state,
                                                                const mrs_msgs::PositionCommand::ConstPtr& reference) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {

    last_update_time_ = uav_state->header.stamp;

    first_iteration_ = false;

    ROS_INFO("[So3Controller]: first iteration");

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));

  } else {

    dt                = (uav_state->header.stamp - last_update_time_).toSec();
    last_update_time_ = uav_state->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[So3Controller]: the last odometry message came too close (%.2f s)!", dt);

    if (last_attitude_cmd_ != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_attitude_cmd_;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));
    }
  }

  // | --------------- calculate the Euler angles --------------- |

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

  // | -------------- calculate the control errors -------------- |

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

  // | --------------------- load the gains --------------------- |

  filterGains(reference->disable_position_gains, dt);

  Eigen::Vector3d Ka = Eigen::Vector3d::Zero(3);
  Eigen::Array3d  Kp = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kv = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kq = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kw = Eigen::Array3d::Zero(3);

  {
    std::scoped_lock lock(mutex_gains_);

    if (reference->use_position_horizontal) {
      Kp[0] = kpxy_;
      Kp[1] = kpxy_;
    } else {
      Kp[0] = 0;
      Kp[1] = 0;
    }

    if (reference->use_position_vertical) {
      Kp[2] = kpz_;
    } else {
      Kp[2] = 0;
    }

    if (reference->use_velocity_horizontal) {
      Kv[0] = kvxy_;
      Kv[1] = kvxy_;
    } else {
      Kv[0] = 0;
      Kv[1] = 0;
    }

    if (reference->use_velocity_vertical) {
      Kv[2] = kvz_;
    } else {
      Kv[2] = 0;
    }

    if (reference->use_acceleration) {
      Ka << kaxy_, kaxy_, kaz_;
    } else {
      Ka << 0, 0, 0;
    }

    Kq << kqxy_, kqxy_, kqz_;

    if (!reference->use_yaw) {
      Kq[2] = 0;
    }

    Kw << kwxy_, kwxy_, kwz_;
  }

  Kp = Kp * (_uav_mass_ + uav_mass_difference_);
  Kv = Kv * (_uav_mass_ + uav_mass_difference_);

  // | --------------- desired orientation matrix --------------- |

  Eigen::Vector2d Ib_w = Eigen::Vector2d(0, 0);

  // get body disturbance in the world frame
  {

    geometry_msgs::Vector3Stamped Ib_b_stamped;

    Ib_b_stamped.header.stamp    = ros::Time::now();
    Ib_b_stamped.header.frame_id = "fcu_untilted";
    Ib_b_stamped.vector.x        = Ib_b_(0);
    Ib_b_stamped.vector.y        = Ib_b_(1);
    Ib_b_stamped.vector.z        = 0;

    auto res = common_handlers_->transformer->transformSingle(uav_state_.header.frame_id, Ib_b_stamped);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: could not transform the Ib_b_ to the world frame");
    }
  }

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, _g_) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array();
  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
  }

  Eigen::Vector3d f = position_feedback + velocity_feedback + integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[So3Controller]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[So3Controller]: NaN detected in variable 'theta', returning null");

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (_tilt_angle_failsafe_ > 1e-3 && theta > _tilt_angle_failsafe_) {

    ROS_ERROR("[So3Controller]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta,
              (180.0 / M_PI) * _tilt_angle_failsafe_);
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
  if (_tilt_angle_saturation_ > 1e-3 && theta > _tilt_angle_saturation_) {
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta / M_PI) * 180.0,
                      (_tilt_angle_saturation_ / M_PI) * 180.0);
    theta = _tilt_angle_saturation_;
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
    thrust = sqrt(thrust_force) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;
  } else {
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: just so you know, the desired thrust force is negative (%.2f)", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[So3Controller]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");

  } else if (thrust > _thrust_saturation_) {

    thrust = _thrust_saturation_;
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: saturating thrust to %.2f", _thrust_saturation_);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: saturating thrust to 0");
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (reference->use_position_horizontal) {
      Iw_w_ -= kiwxy_ * Ep.head(2) * dt;
    } else if (reference->use_velocity_horizontal) {
      Iw_w_ -= kiwxy_ * Ev.head(2) * dt;
    }

    // saturate the world X
    double world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's world Y integral is being saturated!");
    }
  }

  //}

  /* body error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the body error                  |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    Eigen::Vector2d Ep_fcu_untilted = Eigen::Vector2d(0, 0);  // position error in the untilted frame of the UAV
    Eigen::Vector2d Ev_fcu_untilted = Eigen::Vector2d(0, 0);  // velocity error in the untilted frame of the UAV

    // get the position control error in the fcu_untilted frame
    {

      geometry_msgs::Vector3Stamped Ep_stamped;

      Ep_stamped.header.stamp    = ros::Time::now();
      Ep_stamped.header.frame_id = uav_state_.header.frame_id;
      Ep_stamped.vector.x        = Ep(0);
      Ep_stamped.vector.y        = Ep(1);
      Ep_stamped.vector.z        = Ep(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[So3Controller]: could not transform the position error to fcu_untilted");
      }
    }

    // get the velocity control error in the fcu_untilted frame
    {
      geometry_msgs::Vector3Stamped Ev_stamped;

      Ev_stamped.header.stamp    = ros::Time::now();
      Ev_stamped.header.frame_id = uav_state_.header.frame_id;
      Ev_stamped.vector.x        = Ev(0);
      Ev_stamped.vector.y        = Ev(1);
      Ev_stamped.vector.z        = Ev(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ev_stamped);

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[So3Controller]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (reference->use_position_horizontal) {
      Ib_b_ -= kibxy_ * Ep_fcu_untilted * dt;
    } else if (reference->use_velocity_horizontal) {
      Ib_b_ -= kibxy_ * Ev_fcu_untilted * dt;
    }

    // saturate the body
    double body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's body roll integral is being saturated!");
    }
  }

  //}

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    if (reference->use_position_vertical && !rampup_active_) {
      uav_mass_difference_ -= km_ * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // | ----------- report the values of the integrals ----------- |

  ROS_INFO_THROTTLE(5.0, "[So3Controller]: world error integral: x %.2f N, y %.2f N, lim: %.2f N", Iw_w_[X], Iw_w_[Y], kiwxy_lim_);
  ROS_INFO_THROTTLE(5.0, "[So3Controller]: body error integral:  x %.2f N, y %.2f N, lim: %.2f N", Ib_b_[X], Ib_b_[Y], kibxy_lim_);

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Quaterniond des_quater = Eigen::Quaterniond(Rd);

    // rotate the drone's z axis
    Eigen::Vector3d uav_z_in_world = des_quater * Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d thrust_vector = thrust_force * uav_z_in_world;

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
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
    std::scoped_lock lock(mutex_output_mode_);

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

      ROS_WARN_THROTTLE(1.0, "[So3Controller]: outputting attitude quaternion (this is not normal)");
    }

    output_command->desired_acceleration.x = desired_x_accel;
    output_command->desired_acceleration.y = desired_y_accel;
    output_command->desired_acceleration.z = desired_z_accel;
  }

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

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

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = Ib_b_[0];
  output_command->disturbance_by_b = Ib_b_[1];

  output_command->disturbance_bx_w = Ib_w[0];
  output_command->disturbance_by_w = Ib_w[1];

  output_command->disturbance_wx_w = Iw_w_[0];
  output_command->disturbance_wy_w = Iw_w_[1];

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "So3Controller";

  last_attitude_cmd_ = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus So3Controller::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void So3Controller::switchOdometrySource(const mrs_msgs::UavState::ConstPtr& msg) {

  ROS_INFO("[So3Controller]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w_[0];
  world_integrals.vector.y = Iw_w_[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(msg->header.frame_id, world_integrals);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = res.value().vector.x;
    Iw_w_[1] = res.value().vector.y;
  } else {

    ROS_ERROR_THROTTLE(1.0, "[So3Controller]: could not transform world integral to the new frame");

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = 0;
    Iw_w_[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void So3Controller::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w_ = Eigen::Vector2d::Zero(2);
  Ib_b_ = Eigen::Vector2d::Zero(2);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void So3Controller::callbackDrs(mrs_controllers::so3_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_, mutex_output_mode_);

    drs_gains_ = config;

    output_mode_ = config.output_mode;
  }

  ROS_INFO("[So3Controller]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void So3Controller::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kpxy_  = calculateGainChange(dt, kpxy_, drs_gains_.kpxy * gain_coeff, bypass_filter, "kpxy", updated);
    kvxy_  = calculateGainChange(dt, kvxy_, drs_gains_.kvxy * gain_coeff, bypass_filter, "kvxy", updated);
    kaxy_  = calculateGainChange(dt, kaxy_, drs_gains_.kaxy * gain_coeff, bypass_filter, "kaxy", updated);
    kiwxy_ = calculateGainChange(dt, kiwxy_, drs_gains_.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
    kibxy_ = calculateGainChange(dt, kibxy_, drs_gains_.kibxy * gain_coeff, bypass_filter, "kibxy", updated);
    kpz_   = calculateGainChange(dt, kpz_, drs_gains_.kpz * gain_coeff, bypass_filter, "kpz", updated);
    kvz_   = calculateGainChange(dt, kvz_, drs_gains_.kvz * gain_coeff, bypass_filter, "kvz", updated);
    kaz_   = calculateGainChange(dt, kaz_, drs_gains_.kaz * gain_coeff, bypass_filter, "kaz", updated);
    kqxy_  = calculateGainChange(dt, kqxy_, drs_gains_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kqz_   = calculateGainChange(dt, kqz_, drs_gains_.kqz * gain_coeff, bypass_filter, "kqz", updated);
    kwxy_  = calculateGainChange(dt, kwxy_, drs_gains_.kwxy * gain_coeff, bypass_filter, "kwxy", updated);
    kwz_   = calculateGainChange(dt, kwz_, drs_gains_.kwz * gain_coeff, bypass_filter, "kwz", updated);
    km_    = calculateGainChange(dt, km_, drs_gains_.km * gain_coeff, bypass_filter, "km", updated);

    kiwxy_lim_ = calculateGainChange(dt, kiwxy_lim_, drs_gains_.kiwxy_lim, false, "kiwxy_lim", updated);
    kibxy_lim_ = calculateGainChange(dt, kibxy_lim_, drs_gains_.kibxy_lim, false, "kibxy_lim", updated);
    km_lim_    = calculateGainChange(dt, km_lim_, drs_gains_.km_lim, false, "km_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_gains_;

      new_drs_gains_.kpxy        = kpxy_;
      new_drs_gains_.kvxy        = kvxy_;
      new_drs_gains_.kaxy        = kaxy_;
      new_drs_gains_.kiwxy       = kiwxy_;
      new_drs_gains_.kibxy       = kibxy_;
      new_drs_gains_.kpz         = kpz_;
      new_drs_gains_.kvz         = kvz_;
      new_drs_gains_.kaz         = kaz_;
      new_drs_gains_.kqxy        = kqxy_;
      new_drs_gains_.kqz         = kqz_;
      new_drs_gains_.kwxy        = kwxy_;
      new_drs_gains_.kwz         = kwz_;
      new_drs_gains_.kiwxy_lim   = kiwxy_lim_;
      new_drs_gains_.kibxy_lim   = kibxy_lim_;
      new_drs_gains_.km          = km_;
      new_drs_gains_.km_lim      = km_lim_;
      new_drs_gains_.output_mode = output_mode_;

      drs_->updateConfig(new_drs_gains_);
    }
  }
}

//}

/* calculateGainChange() //{ */

double So3Controller::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name,
                                          bool& updated) {

  double change = desired_value - current_value;

  double gains_filter_max_change = _gains_filter_change_rate_ * dt;
  double gains_filter_min_change = _gains_filter_min_change_rate_ * dt;

  if (!bypass_rate) {

    // if current value is near 0...
    double change_in_perc;
    double saturated_change;

    if (fabs(current_value) < 1e-6) {
      change *= gains_filter_max_change;
    } else {

      saturated_change = change;

      change_in_perc = (current_value + saturated_change) / current_value - 1.0;

      if (change_in_perc > gains_filter_max_change) {
        saturated_change = current_value * gains_filter_max_change;
      } else if (change_in_perc < -gains_filter_max_change) {
        saturated_change = current_value * -gains_filter_max_change;
      }

      if (fabs(saturated_change) < fabs(change) * gains_filter_min_change) {
        change *= gains_filter_min_change;
      } else {
        change = saturated_change;
      }
    }
  }

  if (fabs(change) > 1e-3) {
    ROS_INFO_THROTTLE(1.0, "[So3Controller]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace so3_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::so3_controller::So3Controller, mrs_uav_manager::Controller)

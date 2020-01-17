/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_uav_manager/Controller.h>

#include <mrs_controllers/partial_landing_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

//}

#define X 0
#define Y 1
#define Z 2

namespace mrs_controllers
{

namespace partial_landing_controller
{

/* //{ class PartialLandingController */

class PartialLandingController : public mrs_uav_manager::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void dynamicReconfigureCallback(mrs_controllers::partial_landing_controllerConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  bool is_initialized = false;
  bool is_active      = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                                    config_mutex_;
  typedef mrs_controllers::partial_landing_controllerConfig Config;
  typedef dynamic_reconfigure::Server<Config>               ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                      reconfigure_server_;
  void                                                      drs_callback(mrs_controllers::partial_landing_controllerConfig &config, uint32_t level);
  mrs_controllers::partial_landing_controllerConfig         drs_desired_gains;

private:
  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       uav_total_mass;
  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;

  double mass_factor_;
  double thrust_filter_coeff_;
  double output_thrust = 0;

  // actual gains (used and already filtered)
  double kqxy, kqz;  // attitude gains
  double kwxy, kwz;  // attitude rate gains

  // desired gains (set by DRS)
  std::mutex mutex_gains;
  std::mutex mutex_desired_gains;

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
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void PartialLandingController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                                          const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g) {

  ros::NodeHandle nh_(parent_nh, name_space);

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;
  this->uav_mass_     = uav_mass;
  this->g_            = g;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "PartialLandingController");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // attitude gains
  param_loader.load_param("default_gains/horizontal/attitude/kq", kqxy);
  param_loader.load_param("default_gains/vertical/attitude/kq", kqz);

  // attitude rate gains
  param_loader.load_param("default_gains/horizontal/attitude/kw", kwxy);
  param_loader.load_param("default_gains/vertical/attitude/kw", kwz);

  // gain filtering
  param_loader.load_param("gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", gains_filter_min_change_rate_);

  param_loader.load_param("mass_factor", mass_factor_);
  param_loader.load_param("thrust_filter_coeff", thrust_filter_coeff_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[PartialLandingController]: Could not load all parameters!");
    ros::shutdown();
  }

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_desired_gains.kqxy = kqxy;
  drs_desired_gains.kqz  = kqz;
  drs_desired_gains.kwxy = kwxy;
  drs_desired_gains.kwz  = kwz;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_desired_gains);
  ReconfigureServer::CallbackType f = boost::bind(&PartialLandingController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = mrs_lib::Profiler(nh_, "PartialLandingController", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &PartialLandingController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[PartialLandingController]: Could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("[PartialLandingController]: initialized");

  is_initialized = true;
}

//}

/* //{ activate() */

bool PartialLandingController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[PartialLandingController]: activated without getting the last controller's command.");

    return false;

  } else {

    activation_control_command_ = *cmd;

    activation_control_command_.controller_enforcing_constraints = false;

    ROS_INFO("[PartialLandingController]: setting mass difference from the last AttitudeCmd: %.2f kg", uav_mass_difference);

    ROS_INFO("[PartialLandingController]: activated with a last controller's command.");
  }

  output_thrust = cmd->thrust;

  first_iteration = true;

  ROS_INFO("[PartialLandingController]: activated");

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void PartialLandingController::deactivate(void) {

  is_active       = false;
  first_iteration = false;

  ROS_INFO("[PartialLandingController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr PartialLandingController::update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                           const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("update");

  if (!is_active) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // calculate the euler angles
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(uav_state->pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  double        odometry_roll, odometry_pitch, odometry_yaw;
  m.getRPY(odometry_roll, odometry_pitch, odometry_yaw);

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rp - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference
  Eigen::Vector3d           Rp, Rv, Ra, Rw;
  Eigen::Quaternion<double> Rq;

  Eigen::Matrix3d Rd;

  Rp << 0, 0, 0;  // fill the desired position
  Rv << 0, 0, 0;
  Ra << 0, 0, 0;
  Rw << 0, 0, odometry_yaw;

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(0, 0, 0);
  Eigen::Vector3d Ov(0, 0, 0);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << uav_state->pose.orientation.x, uav_state->pose.orientation.y, uav_state->pose.orientation.z, uav_state->pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = uav_state->header.stamp;

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (uav_state->header.stamp - last_update).toSec();
    last_update = uav_state->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_WARN_STREAM_THROTTLE(1.0, "[PartialLandingController]: last " << last_update << ", current " << uav_state->header.stamp);
    ROS_WARN_THROTTLE(1.0, "[PartialLandingController]: the last odometry message came too close! %f", dt);
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  // --------------------------------------------------------------
  // |                            gains                           |
  // --------------------------------------------------------------

  Eigen::Vector3d Ka;
  Eigen::Array3d  Kp, Kv, Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains);

    Kp << 0, 0, 0;
    Kv << 0, 0, 0;
    Ka << 0, 0, 0;
    Kq << kqxy, kqxy, kqz;
    Kw << kwxy, kwxy, kwz;
  }

  // --------------------------------------------------------------
  // |                 desired orientation matrix                 |
  // --------------------------------------------------------------

  Eigen::Vector2d Ib_w;
  Ib_w[0] = activation_control_command_.disturbance_bx_w;
  Ib_w[1] = activation_control_command_.disturbance_by_w;

  Eigen::Vector2d Iw_w;
  Iw_w[0] = activation_control_command_.disturbance_wx_w;
  Iw_w[1] = activation_control_command_.disturbance_wy_w;

  Eigen::Vector3d integral_feedback;
  integral_feedback << Ib_w[0] + Iw_w[0], Ib_w[1] + Iw_w[1], 0;

  Eigen::Vector3d f = integral_feedback + activation_control_command_.total_mass * Eigen::Vector3d(0, 0, g_);

  Eigen::Vector3d f_norm = f.normalized();

  // | ------------- construct the rotational matrix ------------ |

  tf::Quaternion desired_orientation = tf::createQuaternionFromRPY(0, 0, reference->yaw);
  Rq.coeffs() << desired_orientation.getX(), desired_orientation.getY(), desired_orientation.getZ(), desired_orientation.getW();

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

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  if (mute_lateral_gains && !reference->disable_position_gains) {
    mutex_lateral_gains_after_toggle = true;
  }
  mute_lateral_gains = reference->disable_position_gains;

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand output_command = activation_control_command_;
  output_command.header.stamp              = ros::Time::now();

  output_command.attitude_rate.x   = t[0];
  output_command.attitude_rate.y   = t[1];
  output_command.attitude_rate.z   = 0;
  output_command.attitude_rate_set = true;

  Eigen::Quaterniond thrust_vec      = Eigen::Quaterniond(Rd);
  output_command.quater_attitude.w   = thrust_vec.w();
  output_command.quater_attitude.x   = thrust_vec.x();
  output_command.quater_attitude.y   = thrust_vec.y();
  output_command.quater_attitude.z   = thrust_vec.z();
  output_command.quater_attitude_set = true;

  output_command.controller_enforcing_constraints = false;
  output_command.desired_acceleration.x           = 0;
  output_command.desired_acceleration.y           = 0;
  output_command.desired_acceleration.z           = 0;

  output_thrust = thrust_filter_coeff_ * output_thrust +
                  (1 - thrust_filter_coeff_) * (sqrt(mass_factor_ * uav_mass_ * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b);

  output_command.thrust = output_thrust;

  output_command.euler_attitude_set = false;

  output_command.mode_mask = output_command.MODE_ATTITUDE_RATE;

  output_command.controller = "PartialLandingController";

  return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(output_command));
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus PartialLandingController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void PartialLandingController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &msg) {
}

//}

/* resetDisturbanceEstimators() //{ */

void PartialLandingController::resetDisturbanceEstimators(void) {
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void PartialLandingController::dynamicReconfigureCallback(mrs_controllers::partial_landing_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_desired_gains);

    drs_desired_gains = config;
  }

  ROS_INFO("[PartialLandingController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void PartialLandingController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.01, event);

  double gain_coeff                = 1;
  mutex_lateral_gains_after_toggle = false;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains, mutex_desired_gains);

    kqxy = calculateGainChange(kqxy, drs_desired_gains.kqxy, false, "kqxy");
    kqz  = calculateGainChange(kqz, drs_desired_gains.kqz, false, "kqz");
    kwxy = calculateGainChange(kwxy, drs_desired_gains.kwxy, false, "kwxy");
    kwz  = calculateGainChange(kwz, drs_desired_gains.kwz, false, "kwz");
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* calculateGainChange() //{ */

double PartialLandingController::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

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
    ROS_INFO_THROTTLE(1.0, "[PartialLandingController]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* rotate2d() //{ */

Eigen::Vector2d PartialLandingController::rotate2d(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace partial_landing_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::partial_landing_controller::PartialLandingController, mrs_uav_manager::Controller)

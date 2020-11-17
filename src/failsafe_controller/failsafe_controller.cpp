#define VERSION "0.0.5.0"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

//}

namespace mrs_uav_controllers
{

namespace failsafe_controller
{

/* class FailsafeController //{ */

class FailsafeController : public mrs_uav_managers::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const mrs_uav_managers::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | --------------------- thrust control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double                       _g_;
  mrs_uav_managers::MotorParams _motor_params_;
  double                       hover_thrust_;

  double _thrust_decrease_rate_;
  double _initial_thrust_percentage_;

  std::mutex mutex_hover_thrust_;

  // | ----------------------- yaw control ---------------------- |

  double yaw_setpoint_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void FailsafeController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                                    const mrs_uav_managers::MotorParams motor_params, const double uav_mass, const double g,
                                    std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _motor_params_   = motor_params;
  _uav_mass_       = uav_mass;
  _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "FailsafeController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[FailsafeController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("thrust_decrease_rate", _thrust_decrease_rate_);
  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  param_loader.loadParam("initial_thrust_percentage", _initial_thrust_percentage_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[FailsafeController]: Could not load all parameters!");
    ros::shutdown();
  }

  uav_mass_difference_ = 0;

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = sqrt(_uav_mass_ * _g_) * motor_params.A + motor_params.B;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "FailsafeController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[FailsafeController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool FailsafeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  std::scoped_lock lock(mutex_hover_thrust_);

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[FailsafeController]: activated without getting the last controller's command");

    return false;

  } else {

    // | --------------- calculate the euler angles --------------- |

    yaw_setpoint_ = mrs_lib::AttitudeConverter(last_attitude_cmd->attitude).getYaw();

    ROS_INFO("[FailsafeController]: activated with yaw: %.2f rad", yaw_setpoint_);

    activation_attitude_cmd_ = *last_attitude_cmd;
    uav_mass_difference_     = last_attitude_cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    hover_thrust_ = _initial_thrust_percentage_ * sqrt((_uav_mass_ + uav_mass_difference_) * _g_) * _motor_params_.A + _motor_params_.B;

    ROS_INFO("[FailsafeController]: activated with uav_mass_difference %.2f kg.", uav_mass_difference_);
  }

  first_iteration_ = true;

  is_active_ = true;

  return true;
}

//}

/* deactivate() //{ */

void FailsafeController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[FailsafeController]: deactivated");
}

//}

/* update() //{ */

const mrs_msgs::AttitudeCommand::ConstPtr FailsafeController::update([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                     [[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &control_reference) {

  // WARNING: this mutex keeps the disarming routine from being called during the same moment, when the update routine is being called
  // If we try to disarm during the update() execution, it will freeze, since the update() is being called by the control manager
  // and the disarm is automatically swithing motors off, which is automatically switching to NullTracker, which means this controller
  // is getting deactivated
  std::scoped_lock lock(mutex_hover_thrust_);

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {

    last_update_time_ = ros::Time::now();

    first_iteration_ = false;

    ROS_INFO("[FailsafeController]: first iteration");

  } else {

    dt = (ros::Time::now() - last_update_time_).toSec();

    if (dt <= 0.001) {

      ROS_WARN("[FailsafeController]: the update was called with too small dt!");
      if (last_attitude_cmd_ != mrs_msgs::AttitudeCommand::Ptr()) {

        return last_attitude_cmd_;

      } else {

        return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));
      }
    }

    // decrease the hover thrust
    hover_thrust_ -= _thrust_decrease_rate_ * dt;
  }

  last_update_time_ = ros::Time::now();

  // | --------------- prepare the control output --------------- |

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  if (!std::isfinite(hover_thrust_)) {
    hover_thrust_ = 0;
    ROS_ERROR("[FailsafeController]: NaN detected in variable 'hover_thrust', setting it to 0 and returning!!!");
  } else if (hover_thrust_ > 1.0) {
    hover_thrust_ = 1.0;
  } else if (hover_thrust_ < 0.0) {
    hover_thrust_ = 0.0;
  }

  output_command->attitude = mrs_lib::AttitudeConverter(0, 0, yaw_setpoint_);

  output_command->thrust    = hover_thrust_;
  output_command->mode_mask = output_command->MODE_ATTITUDE;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = _uav_mass_ + uav_mass_difference_;

  output_command->desired_acceleration.x = 0;
  output_command->desired_acceleration.y = 0;
  output_command->desired_acceleration.z = 0;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "FailsafeController";

  last_attitude_cmd_ = output_command;

  return output_command;
}

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus FailsafeController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void FailsafeController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void FailsafeController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr FailsafeController::setConstraints([[maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
}

//}

}  // namespace failsafe_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::failsafe_controller::FailsafeController, mrs_uav_managers::Controller)

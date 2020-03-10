#define VERSION "0.0.5.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_uav_manager/Controller.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

#include <std_srvs/SetBool.h>

//}

namespace mrs_controllers
{

namespace failsafe_controller
{

/* class FailsafeController //{ */

class FailsafeController : public mrs_uav_manager::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // | --------------------- thrust control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double                       _g_;
  mrs_uav_manager::MotorParams _motor_params_;
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

void FailsafeController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                                    const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                                    std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _motor_params_   = motor_params;
  _uav_mass_       = uav_mass;
  _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "FailsafeController");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[FailsafeController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("thrust_decrease_rate", _thrust_decrease_rate_);
  param_loader.load_param("enable_profiler", _profiler_enabled_);
  param_loader.load_param("initial_thrust_percentage", _initial_thrust_percentage_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[FailsafeController]: Could not load all parameters!");
    ros::shutdown();
  }

  uav_mass_difference_ = 0;

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = sqrt(_uav_mass_ * _g_) * motor_params.hover_thrust_a + motor_params.hover_thrust_b;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "FailsafeController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[FailsafeController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool FailsafeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  std::scoped_lock lock(mutex_hover_thrust_);

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[FailsafeController]: activated without getting the last controller's command");

    return false;

  } else {

    // | --------------- calculate the euler angles --------------- |

    yaw_setpoint_ = cmd->euler_attitude.z;

    ROS_INFO("[FailsafeController]: activated with yaw: %.2f rad", yaw_setpoint_);

    activation_attitude_cmd_ = *cmd;
    uav_mass_difference_     = cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    hover_thrust_ =
        _initial_thrust_percentage_ * sqrt((_uav_mass_ + uav_mass_difference_) * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;

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
                                                                     [[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &reference) {

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

  output_command->euler_attitude.x   = 0.0;
  output_command->euler_attitude.y   = 0.0;
  output_command->euler_attitude.z   = yaw_setpoint_;
  output_command->euler_attitude_set = true;

  output_command->quater_attitude.x = 0;
  output_command->quater_attitude.y = 0;
  output_command->quater_attitude.z = 0;
  output_command->quater_attitude.w = 1;

  output_command->quater_attitude_set = false;
  output_command->attitude_rate_set   = false;

  output_command->thrust    = hover_thrust_;
  output_command->mode_mask = output_command->MODE_EULER_ATTITUDE;

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

void FailsafeController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &msg) {
}

//}

/* resetDisturbanceEstimators() //{ */

void FailsafeController::resetDisturbanceEstimators(void) {
}

//}

}  // namespace failsafe_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::failsafe_controller::FailsafeController, mrs_uav_manager::Controller)

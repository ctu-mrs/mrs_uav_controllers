#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>

#include <common.h>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>

//}

namespace mrs_uav_controllers
{

namespace failsafe_controller
{

/* class FailsafeController //{ */

class FailsafeController : public mrs_uav_managers::Controller {

public:
  ~FailsafeController(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handler);

  bool activate(const ControlOutput &last_control_output);
  void deactivate(void);

  ControlOutput update(const mrs_msgs::UavState &uav_state, const std::optional<mrs_msgs::TrackerCommand> &tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | ------------------- remember uav state ------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------------- throttle control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double hover_throttle_;

  double _throttle_decrease_rate_;
  double _initial_throttle_percentage_;

  std::mutex mutex_hover_throttle_;

  // | ----------------------- yaw control ---------------------- |

  double yaw_setpoint_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

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
                                    std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = common_handlers->getMass();

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "FailsafeController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[FailsafeController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("throttle_decrease_rate", _throttle_decrease_rate_);
  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  param_loader.loadParam("initial_throttle_percentage", _initial_throttle_percentage_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[FailsafeController]: Could not load all parameters!");
    ros::shutdown();
  }

  uav_mass_difference_ = 0;

  // | ----------- calculate the default hover throttle ----------- |

  hover_throttle_ = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, _uav_mass_ * common_handlers_->g);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "FailsafeController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[FailsafeController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool FailsafeController::activate(const ControlOutput &last_control_output) {

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  if (!last_control_output.control_output) {

    ROS_WARN("[FailsafeController]: activated without getting the last controller's command");

    return false;

  } else {

    // | --------------- calculate the euler angles --------------- |

    if (last_control_output_.desired_orientation) {
      yaw_setpoint_ = mrs_lib::AttitudeConverter(last_control_output.desired_orientation.value()).getYaw();
    } else {
      yaw_setpoint_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
    }

    ROS_INFO("[FailsafeController]: activated with yaw: %.2f rad", yaw_setpoint_);

    activation_control_output_ = last_control_output;

    if (last_control_output.diagnostics.mass_estimator) {
      uav_mass_difference_ = last_control_output.diagnostics.mass_difference;
    } else {
      uav_mass_difference_ = 0;
    }

    activation_control_output_.diagnostics.controller_enforcing_constraints = false;

    {
      std::scoped_lock lock(mutex_hover_throttle_);

      hover_throttle_ = _initial_throttle_percentage_ * mrs_lib::quadratic_throttle_model::forceToThrottle(
                                                            common_handlers_->throttle_model, (_uav_mass_ + uav_mass_difference_) * common_handlers_->g);
    }

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

FailsafeController::ControlOutput FailsafeController::update(const mrs_msgs::UavState &                     uav_state,
                                                             const std::optional<mrs_msgs::TrackerCommand> &tracker_command) {

  // WARNING: this mutex keeps the disarming routine from being called during the same moment, when the update routine is being called
  // If we try to disarm during the update() execution, it will freeze, since the update() is being called by the control manager
  // and the disarm is automatically swithing motors off, which is automatically switching to NullTracker, which means this controller
  // is getting deactivated
  std::scoped_lock lock(mutex_hover_throttle_);

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("FailsafeController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;
  }

  if (!is_active_) {
    return ControlOutput();
  }

  if (!tracker_command) {
    return ControlOutput();
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

      ROS_WARN_THROTTLE(0.1, "[FailsafeController]: the update was called with too small dt (%.3f s)!", dt);

      if (last_control_output_.control_output) {

        return last_control_output_;

      } else {

        return activation_control_output_;
      }
    }

    // decrease the hover throttle
    {
      std::scoped_lock lock(mutex_hover_throttle_);

      hover_throttle_ -= _throttle_decrease_rate_ * dt;

      if (!std::isfinite(hover_throttle_)) {
        hover_throttle_ = 0;
        ROS_ERROR("[FailsafeController]: NaN detected in variable 'hover_throttle', setting it to 0 and returning!!!");
      } else if (hover_throttle_ > 1.0) {
        hover_throttle_ = 1.0;
      } else if (hover_throttle_ < 0.0) {
        hover_throttle_ = 0.0;
      }
    }
  }

  last_update_time_ = ros::Time::now();

  // | --------------- prepare the control output --------------- |

  auto hover_throttle = mrs_lib::get_mutexed(mutex_hover_throttle_, hover_throttle_);

  FailsafeController::ControlOutput control_output;

  control_output.diagnostics.controller = "FailsafeController";

  if (common_handlers_->control_output_modalities.attitude) {

    mrs_msgs::HwApiAttitudeCmd attitude_cmd;

    attitude_cmd.stamp       = ros::Time::now();
    attitude_cmd.orientation = mrs_lib::AttitudeConverter(0, 0, yaw_setpoint_);
    attitude_cmd.throttle    = hover_throttle;

    control_output.control_output = attitude_cmd;
  }

  // TODO finish the other output modalities

  return control_output;
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

void FailsafeController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void FailsafeController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr FailsafeController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
}

//}

}  // namespace failsafe_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::failsafe_controller::FailsafeController, mrs_uav_managers::Controller)

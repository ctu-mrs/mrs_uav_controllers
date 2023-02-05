#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

//}

namespace mrs_uav_controllers
{

namespace midair_activation_controller
{

/* class MidairActivationController //{ */

class MidairActivationController : public mrs_uav_managers::Controller {

public:
  ~MidairActivationController(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::TrackerCommand::ConstPtr &control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------------- thrust control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double hover_thrust_;

  // | ----------------------- yaw control ---------------------- |

  double heading_setpoint_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MidairActivationController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                                            const double uav_mass, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = uav_mass;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MidairActivationController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[MidairActivationController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  uav_mass_difference_ = 0;

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, _uav_mass_ * common_handlers_->g);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "MidairActivationController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MidairActivationController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool MidairActivationController::activate([[maybe_unused]] const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  ROS_INFO("[MidairActivationController]: activating");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  try {
    heading_setpoint_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_WARN_THROTTLE(1.0, "[MidairActivationController]: could not calculate heading");
    heading_setpoint_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
  }

  ROS_INFO("[MidairActivationController]: activated with heading: %.2f rad", heading_setpoint_);

  hover_thrust_ = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, _uav_mass_ * common_handlers_->g);

  is_active_ = true;

  ROS_INFO("[MidairActivationController]: activated");

  return true;
}

//}

/* deactivate() //{ */

void MidairActivationController::deactivate(void) {

  is_active_           = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[MidairActivationController]: deactivated");
}

//}

/* update() //{ */

const mrs_msgs::AttitudeCommand::ConstPtr MidairActivationController::update([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                             [[maybe_unused]] const mrs_msgs::TrackerCommand::ConstPtr &control_reference) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer =
      mrs_lib::ScopeTimer("MidairActivationController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  mrs_lib::set_mutexed(mutex_uav_state_, *uav_state, uav_state_);

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (control_reference == mrs_msgs::TrackerCommand::Ptr()) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // | --------------- prepare the control output --------------- |

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);

  output_command->header.stamp = ros::Time::now();

  output_command->attitude = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(heading_setpoint_);

  output_command->thrust    = hover_thrust_;
  output_command->mode_mask = output_command->MODE_ATTITUDE;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = _uav_mass_ + uav_mass_difference_;

  output_command->desired_acceleration.x = 0;
  output_command->desired_acceleration.y = 0;
  output_command->desired_acceleration.z = 0;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "MidairActivationController";

  return output_command;
}

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus MidairActivationController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MidairActivationController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void MidairActivationController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr MidairActivationController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
}

//}

}  // namespace midair_activation_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::midair_activation_controller::MidairActivationController, mrs_uav_managers::Controller)

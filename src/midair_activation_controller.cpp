/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <common.h>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace mrs_uav_controllers
{

namespace midair_activation_controller
{

/* class MidairActivationController //{ */

class MidairActivationController : public mrs_uav_managers::Controller {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  void destroy();

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::msg::UavState &uav_state, const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command);

  ControlOutput updateActive(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command);

  const mrs_msgs::msg::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::msg::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
  setConstraints(const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::msg::UavState uav_state_;
  std::mutex              mutex_uav_state_;

  // | --------------------- thrust control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double hover_throttle_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ routines ------------------------ |

  double getHeadingSafely(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool MidairActivationController::initialize(const rclcpp::Node::SharedPtr                                        &node,
                                            std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers,
                                            std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node->get_clock();

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  // | ---------- loading params using the parent's nh ---------- |

  private_handlers->parent_param_loader->loadParamReusable("enable_profiler", _profiler_enabled_);

  if (!private_handlers->parent_param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[MidairActivationController]: Could not load all parameters!");
    return false;
  }

  // | -------------------- loading my params ------------------- |

  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("mrs_uav_controllers") +
                                              "/config/private/midair_activation_controller.yaml");
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("mrs_uav_controllers") +
                                              "/config/public/midair_activation_controller.yaml");

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[MidairActivationController]: Could not load all parameters!");
    return false;
  }

  uav_mass_difference_ = 0;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(common_handlers->parent_node, "MidairActivationController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "[MidairActivationController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* destroy() //{ */

void MidairActivationController::destroy() {
}

//}

/* activate() //{ */

bool MidairActivationController::activate([[maybe_unused]] const ControlOutput &last_control_output) {

  RCLCPP_INFO(node_->get_logger(), "[MidairActivationController]: activating");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  hover_throttle_ = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, _uav_mass_ * common_handlers_->g);

  is_active_ = true;

  RCLCPP_INFO(node_->get_logger(), "[MidairActivationController]: activated, hover throttle %.2f", hover_throttle_);

  return true;
}

//}

/* deactivate() //{ */

void MidairActivationController::deactivate(void) {

  is_active_           = false;
  uav_mass_difference_ = 0;

  RCLCPP_INFO(node_->get_logger(), "[MidairActivationController]: deactivated");
}

//}

/* updateInactive() //{ */

void MidairActivationController::updateInactive(const mrs_msgs::msg::UavState                                       &uav_state,
                                                [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);
}

//}

/* //{ updateWhenAcctive() */

MidairActivationController::ControlOutput MidairActivationController::updateActive(const mrs_msgs::msg::UavState       &uav_state,
                                                                                   const mrs_msgs::msg::TrackerCommand &tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer =
      mrs_lib::ScopeTimer(node_, "MidairActivationController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  if (!is_active_) {
    return Controller::ControlOutput();
  }

  // | --------------- prepare the control output --------------- |

  ControlOutput control_output;

  control_output.diagnostics.controller = "MidairActivationController";

  auto highest_modality = common::getHighestOuput(common_handlers_->control_output_modalities);

  if (!highest_modality) {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[MidairActivationController]: output modalities are empty! This error should never appear.");

    return control_output;
  }

  switch (highest_modality.value()) {

  case common::POSITION: {

    mrs_msgs::msg::HwApiPositionCmd cmd;

    cmd.header.stamp    = clock_->now();
    cmd.header.frame_id = uav_state.header.frame_id;

    cmd.position.x = uav_state.pose.position.x;
    cmd.position.y = uav_state.pose.position.y;
    cmd.position.z = uav_state.pose.position.z;

    cmd.heading = getHeadingSafely(uav_state, tracker_command);

    control_output.control_output = cmd;

    break;
  }

  case common::VELOCITY_HDG: {

    mrs_msgs::msg::HwApiVelocityHdgCmd cmd;

    cmd.header.stamp    = clock_->now();
    cmd.header.frame_id = uav_state.header.frame_id;

    cmd.velocity.x = uav_state.velocity.linear.x;
    cmd.velocity.y = uav_state.velocity.linear.y;
    cmd.velocity.z = uav_state.velocity.linear.z;

    cmd.heading = getHeadingSafely(uav_state, tracker_command);

    control_output.control_output = cmd;

    break;
  }

  case common::VELOCITY_HDG_RATE: {

    mrs_msgs::msg::HwApiVelocityHdgRateCmd cmd;

    cmd.header.stamp    = clock_->now();
    cmd.header.frame_id = uav_state.header.frame_id;

    cmd.velocity.x = uav_state.velocity.linear.x;
    cmd.velocity.y = uav_state.velocity.linear.y;
    cmd.velocity.z = uav_state.velocity.linear.z;

    cmd.heading_rate = 0;

    control_output.control_output = cmd;

    break;
  }

  case common::ACCELERATION_HDG: {

    mrs_msgs::msg::HwApiAccelerationHdgCmd cmd;

    cmd.header.stamp    = clock_->now();
    cmd.header.frame_id = uav_state.header.frame_id;

    cmd.acceleration.x = uav_state.acceleration.linear.x;
    cmd.acceleration.y = uav_state.acceleration.linear.y;
    cmd.acceleration.z = uav_state.acceleration.linear.z;

    cmd.heading = getHeadingSafely(uav_state, tracker_command);

    control_output.control_output = cmd;

    break;
  }

  case common::ACCELERATION_HDG_RATE: {

    mrs_msgs::msg::HwApiAccelerationHdgRateCmd cmd;

    cmd.header.stamp    = clock_->now();
    cmd.header.frame_id = uav_state.header.frame_id;

    cmd.acceleration.x = uav_state.acceleration.linear.x;
    cmd.acceleration.y = uav_state.acceleration.linear.y;
    cmd.acceleration.z = uav_state.acceleration.linear.z;

    cmd.heading_rate = 0;

    control_output.control_output = cmd;

    break;
  }

  case common::ATTITUDE: {

    mrs_msgs::msg::HwApiAttitudeCmd cmd;

    cmd.stamp = clock_->now();

    cmd.orientation = uav_state.pose.orientation;
    cmd.throttle    = hover_throttle_;

    control_output.control_output = cmd;

    break;
  }

  case common::ATTITUDE_RATE: {

    mrs_msgs::msg::HwApiAttitudeRateCmd cmd;

    cmd.stamp = clock_->now();

    cmd.body_rate.x = 0;
    cmd.body_rate.y = 0;
    cmd.body_rate.z = 0;

    cmd.throttle = hover_throttle_;

    control_output.control_output = cmd;

    break;
  }

  case common::CONTROL_GROUP: {

    mrs_msgs::msg::HwApiControlGroupCmd cmd;

    cmd.stamp = clock_->now();

    cmd.roll     = 0;
    cmd.pitch    = 0;
    cmd.yaw      = 0;
    cmd.throttle = hover_throttle_;

    control_output.control_output = cmd;

    break;
  }

  case common::ACTUATORS_CMD: {

    mrs_msgs::msg::HwApiActuatorCmd cmd;

    cmd.stamp = clock_->now();

    for (int i = 0; i < common_handlers_->throttle_model.n_motors; i++) {
      cmd.motors.push_back(hover_throttle_);
    }

    control_output.control_output = cmd;

    break;
  }
  }

  return control_output;
} // namespace midair_activation_controller

//}

/* getStatus() //{ */

const mrs_msgs::msg::ControllerStatus MidairActivationController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MidairActivationController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void MidairActivationController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
MidairActivationController::setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) {

  return nullptr;
}

//}

/* getHeadingSafely() //{ */

double MidairActivationController::getHeadingSafely(const mrs_msgs::msg::UavState &uav_state, const mrs_msgs::msg::TrackerCommand &tracker_command) {

  try {
    return mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
  }

  try {
    return mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
  }
  catch (...) {
  }

  if (tracker_command.use_heading) {
    return tracker_command.heading;
  }

  return 0;
}

//}

} // namespace midair_activation_controller

} // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::midair_activation_controller::MidairActivationController, mrs_uav_managers::Controller)

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <common.h>

#include <mrs_uav_managers/controller.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscriber_handler.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_controllers
{

namespace failsafe_controller
{

/* class FailsafeController //{ */

class FailsafeController : public mrs_uav_managers::Controller {

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

  double getHeadingSafely(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr quaternion);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ----------------------- parameters ----------------------- |

  double _descend_speed_;
  double _descend_acceleration_;

  double _kq_;
  double _kw_;

  // | ------------------- remember uav state ------------------- |

  mrs_msgs::msg::UavState uav_state_;
  std::mutex              mutex_uav_state_;

  // | --------------------- throttle control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double hover_throttle_;

  double _throttle_decrease_rate_;
  double _initial_throttle_percentage_;

  // | ----------------------- yaw control ---------------------- |

  double heading_setpoint_;

  mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped> sh_hw_api_orientation_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  rclcpp::Time      last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::msg::DynamicsConstraints constraints_;
  std::mutex                         mutex_constraints_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool FailsafeController::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                    std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  node_  = node;
  clock_ = node->get_clock();

  cbkgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(node_->get_logger(), "initializing");

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();
  // | ---------- loading params using the parent's nh ---------- |

  private_handlers->parent_param_loader->loadParamReusable("enable_profiler", _profiler_enabled_);

  if (!private_handlers->parent_param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[FailsafeController]: Could not load all parameters!");
    return false;
  }

  // | -------------------- loading my params ------------------- |

  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("mrs_uav_controllers") + "/config/private/failsafe_controller.yaml");
  private_handlers->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("mrs_uav_controllers") + "/config/public/failsafe_controller.yaml");

  private_handlers->param_loader->loadParam("throttle_output/throttle_decrease_rate", _throttle_decrease_rate_);
  private_handlers->param_loader->loadParam("throttle_output/initial_throttle_percentage", _initial_throttle_percentage_);

  private_handlers->param_loader->loadParam("attitude_controller/gains/kp", _kq_);

  private_handlers->param_loader->loadParam("rate_controller/gains/kp", _kw_);

  private_handlers->param_loader->loadParam("velocity_output/descend_speed", _descend_speed_);
  private_handlers->param_loader->loadParam("acceleration_output/descend_acceleration", _descend_acceleration_);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[FailsafeController]: Could not load all parameters!");
    return false;
  }

  _descend_speed_        = std::abs(_descend_speed_);
  _descend_acceleration_ = std::abs(_descend_acceleration_);

  uav_mass_difference_ = 0;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_hw_api_orientation_ =
      mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, "/" + common_handlers->uav_name + "/" + "hw_api/orientation");

  // | ----------- calculate the default hover throttle ----------- |

  hover_throttle_ = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, _uav_mass_ * common_handlers_->g);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(common_handlers->parent_node, "FailsafeController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "[FailsafeController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* destroy() //{ */

void FailsafeController::destroy() {
}

//}

/* activate() //{ */

bool FailsafeController::activate(const ControlOutput &last_control_output) {

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  if (!last_control_output.control_output) {

    RCLCPP_WARN(node_->get_logger(), "[FailsafeController]: activated without getting the last controller's command");

    return false;

  } else {

    // | -------------- calculate the initial heading ------------- |

    if (sh_hw_api_orientation_.getMsg()) {

      auto hw_api_orientation = sh_hw_api_orientation_.getMsg();

      heading_setpoint_ = getHeadingSafely(hw_api_orientation);

      RCLCPP_INFO(node_->get_logger(), "[FailsafeController]: activated with heading = %.2f rad", heading_setpoint_);

    } else {

      RCLCPP_ERROR(node_->get_logger(), "[FailsafeController]: missing orientation from HW API, activated with heading = 0 rad");

      heading_setpoint_ = 0;
    }

    activation_control_output_ = last_control_output;

    if (last_control_output.diagnostics.mass_estimator) {
      uav_mass_difference_ = last_control_output.diagnostics.mass_difference;
    } else {
      uav_mass_difference_ = 0;
    }

    activation_control_output_.diagnostics.controller_enforcing_constraints = false;

    hover_throttle_ = _initial_throttle_percentage_ * mrs_lib::quadratic_throttle_model::forceToThrottle(
                                                          common_handlers_->throttle_model, (_uav_mass_ + uav_mass_difference_) * common_handlers_->g);

    RCLCPP_INFO(node_->get_logger(), "[FailsafeController]: activated with uav_mass_difference %.2f kg, hover_throttle %.3f", uav_mass_difference_,
                hover_throttle_);
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

  RCLCPP_INFO(node_->get_logger(), "[FailsafeController]: deactivated");
}

//}

/* updateInactive() //{ */

void FailsafeController::updateInactive(const mrs_msgs::msg::UavState                                       &uav_state,
                                        [[maybe_unused]] const std::optional<mrs_msgs::msg::TrackerCommand> &tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = clock_->now();

  first_iteration_ = false;
}

//}

/* //{ updateWhenAcctive() */

FailsafeController::ControlOutput FailsafeController::updateActive(const mrs_msgs::msg::UavState                        &uav_state,
                                                                   [[maybe_unused]] const mrs_msgs::msg::TrackerCommand &tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer =
      mrs_lib::ScopeTimer(node_, "FailsafeController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;
  }

  if (!is_active_) {
    return ControlOutput();
  }

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (clock_->now() - last_update_time_).seconds();
  }

  last_update_time_ = clock_->now();

  hover_throttle_ -= _throttle_decrease_rate_ * dt;

  if (!std::isfinite(hover_throttle_)) {
    hover_throttle_ = 0;
    RCLCPP_ERROR(node_->get_logger(), "[FailsafeController]: NaN detected in variable 'hover_throttle', setting it to 0 and returning!!!");
  } else if (hover_throttle_ > 1.0) {
    hover_throttle_ = 1.0;
  } else if (hover_throttle_ < 0.0) {
    hover_throttle_ = 0.0;
  }

  // | --------------- prepare the control output --------------- |

  FailsafeController::ControlOutput control_output;

  control_output.diagnostics.controller = "FailsafeController";

  auto highest_modality = common::getHighestOuput(common_handlers_->control_output_modalities);

  if (!highest_modality) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: output modalities are empty! This error should never appear.");
    return control_output;
  }

  control_output.diagnostics.controller = "FailsafeController";

  // --------------------------------------------------------------
  // |                       position output                      |
  // --------------------------------------------------------------

  if (highest_modality.value() == common::POSITION) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning empty command, because we are at the position modality");
    return control_output;
  }

  // --------------------------------------------------------------
  // |                       velocity output                      |
  // --------------------------------------------------------------

  if (highest_modality.value() == common::VELOCITY_HDG) {

    mrs_msgs::msg::HwApiVelocityHdgCmd vel_cmd;

    vel_cmd.header.stamp = clock_->now();

    vel_cmd.velocity.x = 0;
    vel_cmd.velocity.y = 0;
    vel_cmd.velocity.z = -_descend_speed_;
    vel_cmd.heading    = heading_setpoint_;

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning velocity+hdg output");
    control_output.control_output = vel_cmd;
    return control_output;
  }

  if (highest_modality.value() == common::VELOCITY_HDG_RATE) {

    mrs_msgs::msg::HwApiVelocityHdgRateCmd vel_cmd;

    vel_cmd.header.stamp = clock_->now();

    vel_cmd.velocity.x   = 0;
    vel_cmd.velocity.y   = 0;
    vel_cmd.velocity.z   = -_descend_speed_;
    vel_cmd.heading_rate = 0.0;

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning velocity+hdg rate output");
    control_output.control_output = vel_cmd;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                     acceleration output                    |
  // --------------------------------------------------------------

  if (highest_modality.value() == common::ACCELERATION_HDG) {

    mrs_msgs::msg::HwApiAccelerationHdgCmd acc_cmd;

    acc_cmd.header.stamp = clock_->now();

    acc_cmd.acceleration.x = 0;
    acc_cmd.acceleration.y = 0;
    acc_cmd.acceleration.z = -_descend_acceleration_;
    acc_cmd.heading        = heading_setpoint_;

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning acceleration+hdg output");
    control_output.control_output = acc_cmd;
    return control_output;
  }

  if (highest_modality.value() == common::ACCELERATION_HDG_RATE) {

    mrs_msgs::msg::HwApiAccelerationHdgRateCmd acc_cmd;

    acc_cmd.header.stamp = clock_->now();

    acc_cmd.acceleration.x = 0;
    acc_cmd.acceleration.y = 0;
    acc_cmd.acceleration.z = -_descend_acceleration_;
    acc_cmd.heading_rate   = 0.0;

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning acceleration+hdg rate output");
    control_output.control_output = acc_cmd;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                       attitude output                      |
  // --------------------------------------------------------------

  mrs_msgs::msg::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.stamp       = clock_->now();
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(0, 0, heading_setpoint_);
  attitude_cmd.throttle    = hover_throttle_;

  if (highest_modality.value() == common::ATTITUDE) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning attitude output");
    control_output.control_output = attitude_cmd;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                      attitude control                      |
  // --------------------------------------------------------------

  Eigen::Vector3d attitude_rate_saturation(constraints.roll_rate, constraints.pitch_rate, constraints.yaw_rate);
  Eigen::Vector3d rate_ff(0, 0, 0);
  Eigen::Vector3d Kq(_kq_, _kq_, _kq_);

  auto attitude_rate_command = common::attitudeController(node_, uav_state, attitude_cmd, rate_ff, attitude_rate_saturation, Kq, false);

  if (highest_modality.value() == common::ATTITUDE_RATE) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning attitude rate output");
    control_output.control_output = attitude_rate_command;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                    attitude rate control                   |
  // --------------------------------------------------------------

  Eigen::Vector3d Kw = common_handlers_->detailed_model_params->inertia.diagonal() * _kw_;

  auto control_group_command = common::attitudeRateController(node_, uav_state, attitude_rate_command.value(), Kw);

  if (highest_modality.value() == common::CONTROL_GROUP) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning control group output");
    control_output.control_output = control_group_command;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                            mixer                           |
  // --------------------------------------------------------------

  mrs_msgs::msg::HwApiActuatorCmd actuator_cmd =
      common::actuatorMixer(node_, control_group_command.value(), common_handlers_->detailed_model_params->control_group_mixer);

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[FailsafeController]: returning actuators output");
  control_output.control_output = actuator_cmd;

  return control_output;
}

//}

/* getStatus() //{ */

const mrs_msgs::msg::ControllerStatus FailsafeController::getStatus() {

  mrs_msgs::msg::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void FailsafeController::switchOdometrySource([[maybe_unused]] const mrs_msgs::msg::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void FailsafeController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response>
FailsafeController::setConstraints([[maybe_unused]] const std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Request> &constraints) {

  std::shared_ptr<mrs_msgs::srv::DynamicsConstraintsSrv::Response> response = std::make_shared<mrs_msgs::srv::DynamicsConstraintsSrv::Response>();

  if (!is_initialized_) {
    response->success = false;
    response->message = "not initialized";
    return response;
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  RCLCPP_INFO(node_->get_logger(), "[FailsafeController]: updating constraints");

  response->success = true;
  response->message = "constraints updated";

  return response;
}

//}

// | ------------------------- methods ------------------------ |

/* getHeadingSafely() //{ */

double FailsafeController::getHeadingSafely(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr quaternion) {

  try {
    return mrs_lib::AttitudeConverter(quaternion->quaternion).getHeading();
  }
  catch (...) {
  }

  try {
    return mrs_lib::AttitudeConverter(quaternion->quaternion).getYaw();
  }
  catch (...) {
  }

  return 0;
}

//}

} // namespace failsafe_controller

} // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::failsafe_controller::FailsafeController, mrs_uav_managers::Controller)

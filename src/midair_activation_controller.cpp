#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>

#include <common.h>

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

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  void update(const mrs_msgs::UavState &uav_state);

  MidairActivationController::ControlOutput update(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

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

  double hover_throttle_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ routines ------------------------ |

  double getHeadingSafely(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MidairActivationController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                                            std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = common_handlers->getMass();

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MidairActivationController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[MidairActivationController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  uav_mass_difference_ = 0;

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "MidairActivationController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MidairActivationController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool MidairActivationController::activate([[maybe_unused]] const ControlOutput &last_control_output) {

  ROS_INFO("[MidairActivationController]: activating");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  hover_throttle_ = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, _uav_mass_ * common_handlers_->g);

  is_active_ = true;

  ROS_INFO("[MidairActivationController]: activated, hover throttle %.2f", hover_throttle_);

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

void MidairActivationController::update(const mrs_msgs::UavState &uav_state) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);
}

MidairActivationController::ControlOutput MidairActivationController::update([[maybe_unused]] const mrs_msgs::UavState &      uav_state,
                                                                             [[maybe_unused]] const mrs_msgs::TrackerCommand &tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer =
      mrs_lib::ScopeTimer("MidairActivationController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  if (!is_active_) {
    return Controller::ControlOutput();
  }

  // | --------------- prepare the control output --------------- |

  ControlOutput control_output;

  control_output.diagnostics.controller = "MidairActivationController";

  auto highest_modality = common::getHighestOuput(common_handlers_->control_output_modalities);

  if (!highest_modality) {

    ROS_ERROR_THROTTLE(1.0, "[MidairActivationController]: output modalities are empty! This error should never appear.");

    return control_output;
  }

  switch (highest_modality.value()) {

    case common::POSITION: {

      mrs_msgs::HwApiPositionCmd cmd;

      cmd.header.stamp    = ros::Time::now();
      cmd.header.frame_id = uav_state.header.frame_id;

      cmd.position.x = uav_state.pose.position.x;
      cmd.position.y = uav_state.pose.position.y;
      cmd.position.z = uav_state.pose.position.z;

      cmd.heading = getHeadingSafely(uav_state, tracker_command);

      control_output.control_output = cmd;

      break;
    }

    case common::VELOCITY_HDG: {

      mrs_msgs::HwApiVelocityHdgCmd cmd;

      cmd.header.stamp    = ros::Time::now();
      cmd.header.frame_id = uav_state.header.frame_id;

      cmd.velocity.x = uav_state.velocity.linear.x;
      cmd.velocity.y = uav_state.velocity.linear.y;
      cmd.velocity.z = uav_state.velocity.linear.z;

      cmd.heading = getHeadingSafely(uav_state, tracker_command);

      control_output.control_output = cmd;

      break;
    }

    case common::VELOCITY_HDG_RATE: {

      mrs_msgs::HwApiVelocityHdgRateCmd cmd;

      cmd.header.stamp    = ros::Time::now();
      cmd.header.frame_id = uav_state.header.frame_id;

      cmd.velocity.x = uav_state.velocity.linear.x;
      cmd.velocity.y = uav_state.velocity.linear.y;
      cmd.velocity.z = uav_state.velocity.linear.z;

      cmd.heading_rate = 0;

      control_output.control_output = cmd;

      break;
    }

    case common::ACCELERATION_HDG: {

      mrs_msgs::HwApiAccelerationHdgCmd cmd;

      cmd.header.stamp    = ros::Time::now();
      cmd.header.frame_id = uav_state.header.frame_id;

      cmd.acceleration.x = 0;
      cmd.acceleration.y = 0;
      cmd.acceleration.z = 0;

      cmd.heading = getHeadingSafely(uav_state, tracker_command);

      control_output.control_output = cmd;

      break;
    }

    case common::ACCELERATION_HDG_RATE: {

      mrs_msgs::HwApiAccelerationHdgRateCmd cmd;

      cmd.header.stamp    = ros::Time::now();
      cmd.header.frame_id = uav_state.header.frame_id;

      cmd.acceleration.x = 0;
      cmd.acceleration.y = 0;
      cmd.acceleration.z = 0;

      cmd.heading_rate = 0;

      control_output.control_output = cmd;

      break;
    }

    case common::ATTITUDE: {

      mrs_msgs::HwApiAttitudeCmd cmd;

      cmd.stamp = ros::Time::now();

      cmd.orientation = uav_state.pose.orientation;
      cmd.throttle    = hover_throttle_;

      control_output.control_output = cmd;

      break;
    }

    case common::ATTITUDE_RATE: {

      mrs_msgs::HwApiAttitudeRateCmd cmd;

      cmd.stamp = ros::Time::now();

      cmd.body_rate.x = 0;
      cmd.body_rate.y = 0;
      cmd.body_rate.z = 0;

      cmd.throttle = hover_throttle_;

      control_output.control_output = cmd;

      break;
    }

    case common::CONTROL_GROUP: {

      mrs_msgs::HwApiControlGroupCmd cmd;

      cmd.stamp = ros::Time::now();

      cmd.roll     = 0;
      cmd.pitch    = 0;
      cmd.yaw      = 0;
      cmd.throttle = hover_throttle_;

      control_output.control_output = cmd;

      break;
    }

    case common::ACTUATORS_CMD: {

      mrs_msgs::HwApiActuatorCmd cmd;

      cmd.stamp = ros::Time::now();

      for (int i = 0; i < common_handlers_->throttle_model.n_motors; i++) {
        cmd.motors.push_back(hover_throttle_);
      }

      control_output.control_output = cmd;

      break;
    }
  }

  return control_output;
}  // namespace midair_activation_controller

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus MidairActivationController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MidairActivationController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState &new_uav_state) {
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

/* getHeadingSafely() //{ */

double MidairActivationController::getHeadingSafely(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command) {

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

}  // namespace midair_activation_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::midair_activation_controller::MidairActivationController, mrs_uav_managers::Controller)

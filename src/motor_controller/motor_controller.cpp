#define VERSION "1.0.3.0"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/controller.h>
#include <mrs_msgs/ActuatorControl.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>

#include <geometry_msgs/Vector3Stamped.h>

//}

#define OUTPUT_ATTITUDE_RATE 0
#define OUTPUT_ATTITUDE_QUATERNION 1
#define OUTPUT_ACTUATOR_CONTROL 2

namespace mrs_uav_controllers
{

namespace motor_controller
{

/* //{ class MotorController */

class MotorController : public mrs_uav_managers::Controller {

public:
  ~MotorController(){};

  void initialize(const ros::NodeHandle& parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr& last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr& uav_state, const mrs_msgs::PositionCommand::ConstPtr& control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints([
      [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  bool                          got_constraints_ = false;

  // | ---------- thrust generation and mass estimation --------- |

  double _uav_mass_;
  double uav_mass_difference_;

  // | ------------ controller limits and saturations ----------- |

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::ActuatorControl> sc_actuator_control_;

  std::future<mrs_msgs::ActuatorControl> future_service_result_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void MotorController::initialize(const ros::NodeHandle& parent_nh, [[maybe_unused]] const std::string name, const std::string name_space, const double uav_mass,
                                 std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = uav_mass;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MotorController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[MotorController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  // constraints
  param_loader.loadParam("constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam("constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);
  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[MotorController]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MotorController]: could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sc_actuator_control_ = mrs_lib::ServiceClientHandler<mrs_msgs::ActuatorControl>(nh_, "actuator_control_out");

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MotorController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool MotorController::activate(const mrs_msgs::AttitudeCommand::ConstPtr& last_attitude_cmd) {

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[MotorController]: activated without getting the last controller's command");

    return false;
  }

  ROS_INFO("[MotorController]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void MotorController::deactivate(void) {

  is_active_           = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[MotorController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr MotorController::update(const mrs_msgs::UavState::ConstPtr&        uav_state,
                                                                  const mrs_msgs::PositionCommand::ConstPtr& control_reference) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MotorController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (control_reference == mrs_msgs::PositionCommand::Ptr()) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // | ------------------ prepeare service out ------------------ |

  mrs_msgs::ActuatorControl srv_out;

  srv_out.request.header    = uav_state->header;
  srv_out.request.uav_state = *uav_state;
  srv_out.request.reference = *control_reference;

  future_service_result_ = sc_actuator_control_.callAsync(srv_out);

  int i = 0;

  while (ros::ok() && future_service_result_.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {

    if (i++ > 10) {
      ROS_WARN("[MotorController]: service request timeouted, switching back");
      return mrs_msgs::AttitudeCommand::ConstPtr();
    }
  }

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  Eigen::Vector3d actuator_action;

  Eigen::Matrix4d mixer_matrix;

  output_command->actuator_control.x = -actuator_action[0];
  output_command->actuator_control.y = actuator_action[1];
  output_command->actuator_control.z = actuator_action[2];

  output_command->mass_difference = 0;
  output_command->total_mass      = _uav_mass_;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "MotorController";

  last_attitude_cmd_ = output_command;

  return output_command;
}  // namespace motor_controller

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus MotorController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MotorController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void MotorController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr MotorController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  got_constraints_ = true;

  ROS_INFO("[MotorController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

}  // namespace motor_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::motor_controller::MotorController, mrs_uav_managers::Controller)

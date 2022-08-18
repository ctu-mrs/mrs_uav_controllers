#define VERSION "1.0.3.0"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/controller.h>
#include <mrs_msgs/ActuatorControl.h>
#include <mrs_msgs/ActuatorControlIn.h>
#include <mrs_msgs/ActuatorControlOut.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

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
  ros::NodeHandle nh_;

  std::string _version_;
  int         _mode_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  Eigen::MatrixXd _allocation_matrix_;

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

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::ActuatorControl> sc_actuator_control_;

  std::future<mrs_msgs::ActuatorControl> future_service_result_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::ActuatorControlOut> ph_out_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::ActuatorControlIn> sh_in_;

  void callbackActuatorControl(mrs_lib::SubscribeHandler<mrs_msgs::ActuatorControlIn>& wrp);

  std::atomic<bool> got_response_ = false;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void MotorController::initialize(const ros::NodeHandle& parent_nh, [[maybe_unused]] const std::string name, const std::string name_space, const double uav_mass,
                                 std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  nh_ = ros::NodeHandle(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = uav_mass;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MotorController");

  param_loader.loadParam("version", _version_);
  param_loader.loadParam("mode", _mode_);

  _allocation_matrix_ = param_loader.loadMatrixDynamic2("allocation_matrix", 4, -1);

  if (_version_ != VERSION) {

    ROS_ERROR("[MotorController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MotorController]: could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sc_actuator_control_ = mrs_lib::ServiceClientHandler<mrs_msgs::ActuatorControl>(nh_, "actuator_control_srv_out");

  // | ----------------------- publishers ----------------------- |

  ph_out_ = mrs_lib::PublisherHandler<mrs_msgs::ActuatorControlOut>(nh_, "actuator_control_out", 10);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MotorController";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_in_ = mrs_lib::SubscribeHandler<mrs_msgs::ActuatorControlIn>(shopts, "actuator_control_in", &MotorController::callbackActuatorControl, this);

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

  // | ------------------------- request ------------------------ |

  mrs_msgs::ActuatorControlOut msg_out;

  msg_out.header    = uav_state->header;
  msg_out.uav_state = *uav_state;
  msg_out.reference = *control_reference;

  // | ------------------------- result ------------------------- |

  mrs_msgs::ActuatorControlIn msg_in;

  // | ------------------ prepeare service out ------------------ |

  if (_mode_ == 0) {

    mrs_msgs::ActuatorControl srv_out;
    srv_out.request.request = msg_out;

    future_service_result_ = sc_actuator_control_.callAsync(srv_out);

    int i = 0;

    while (ros::ok() && future_service_result_.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {

      if (i > 10) {
        ROS_WARN("[MotorController]: control service call takes more than 10 ms!");
      } else if (i > 20) {
        ROS_ERROR("[MotorController]: control service call takes more than 20 ms!");
      }

      if (i++ > 100) {
        ROS_WARN("[MotorController]: service request timeouted, switching back");
        return mrs_msgs::AttitudeCommand::ConstPtr();
      }
    }

    msg_in = future_service_result_.get().response.response;

  } else if (_mode_ == 1) {

    got_response_ = false;

    ph_out_.publish(msg_out);

    int i = 0;

    while (ros::ok() && !got_response_) {

      ros::Duration(0.001).sleep();

      if (i > 10) {
        ROS_WARN("[MotorController]: control topic takes more than 10 ms!");
      } else if (i > 20) {
        ROS_ERROR("[MotorController]: control topic takes more than 20 ms!");
      }

      if (i++ > 100) {
        ROS_WARN("[MotorController]: topic request timeouted, switching back");
        return mrs_msgs::AttitudeCommand::ConstPtr();
      }
    }

    msg_in = *sh_in_.getMsg();

  } else if (_mode_ == 2) {

    mrs_msgs::ActuatorControlIn msg_in;
  }

  if (!msg_in.success) {
    ROS_WARN("[MotorController]: received false status from the external controller, switching back");
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  ROS_INFO("[MotorController]: response is %.3 s old", (ros::Time::now() - msg_in.header.stamp).toSec());

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | -------------- unbiased desired acceleration ------------- |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state->header.frame_id;
    world_accel.vector.x        = msg_in.desired_acceleration.x;
    world_accel.vector.y        = msg_in.desired_acceleration.y;
    world_accel.vector.z        = msg_in.desired_acceleration.z;

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {

      desired_x_accel = res.value().vector.x;
      desired_y_accel = res.value().vector.y;
      desired_z_accel = res.value().vector.z;
    }
  }

  // | --------------------- motor commands --------------------- |

  Eigen::Vector4d motors;

  if (!std::isfinite(msg_in.motors[0])) {
    ROS_ERROR("NaN detected in variable \"msg_in.motors[0]\"!!!");
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (!std::isfinite(msg_in.motors[1])) {
    ROS_ERROR("NaN detected in variable \"msg_in.motors[1]\"!!!");
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (!std::isfinite(msg_in.motors[2])) {
    ROS_ERROR("NaN detected in variable \"msg_in.motors[2]\"!!!");
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (!std::isfinite(msg_in.motors[3])) {
    ROS_ERROR("NaN detected in variable \"msg_in.motors[3]\"!!!");
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  motors << msg_in.motors[0], msg_in.motors[1], msg_in.motors[2], msg_in.motors[3];

  Eigen::Vector4d control_group = _allocation_matrix_ * motors;

  output_command->actuator_control.x = control_group[0];
  output_command->actuator_control.y = control_group[1];
  output_command->actuator_control.z = control_group[2];
  output_command->thrust             = control_group[3];

  output_command->desired_acceleration.x = desired_x_accel;
  output_command->desired_acceleration.y = desired_y_accel;
  output_command->desired_acceleration.z = desired_z_accel;

  output_command->attitude = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(control_reference->heading);

  output_command->mass_difference = 0;
  output_command->total_mass      = _uav_mass_;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "MotorController";

  output_command->mode_mask = output_command->MODE_ACTUATORS;

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

// | ------------------------ callbacks ----------------------- |

/* callbackActuatroControl() //{ */

void MotorController::callbackActuatorControl([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::ActuatorControlIn>& wrp) {

  if (!is_initialized_)
    return;

  got_response_ = true;
}

//}

}  // namespace motor_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::motor_controller::MotorController, mrs_uav_managers::Controller)

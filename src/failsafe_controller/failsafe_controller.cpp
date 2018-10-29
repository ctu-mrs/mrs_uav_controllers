#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_msgs/ControllerStatus.h>
#include <mrs_mav_manager/Controller.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

namespace mrs_controllers
{

//{ class FailsafeController

class FailsafeController : public mrs_mav_manager::Controller {

public:
  FailsafeController(void);

  void initialize(const ros::NodeHandle &parent_nh, mrs_mav_manager::MotorParams motor_params);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus::Ptr     getStatus();

private:
  bool is_initialized = false;
  bool is_active      = false;

  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_mav_manager::MotorParams motor_params_;
  double                       hover_thrust;
  double                       thrust_decrease_rate_;

  double roll, pitch, yaw;
  double setpoint_yaw;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;

private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;
};

FailsafeController::FailsafeController(void) {
}

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

//{ initialize()

void FailsafeController::initialize(const ros::NodeHandle &parent_nh, mrs_mav_manager::MotorParams motor_params) {

  ros::NodeHandle nh_(parent_nh, "failsafe_controller");

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "FailsafeController");

  param_loader.load_param("thrust_decrease_rate", thrust_decrease_rate_);
  param_loader.load_param("uav_mass", uav_mass_);
  param_loader.load_param("g", g_);
  param_loader.load_param("enable_profiler", profiler_enabled_);

  uav_mass_difference = 0;

  // --------------------------------------------------------------
  // |                 calculate the hover thrust                 |
  // --------------------------------------------------------------

  hover_thrust = sqrt(uav_mass_ * g_) * motor_params.hover_thrust_a + motor_params.hover_thrust_b;

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "FailsafeController", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[FailsafeController]: Could not load all parameters!");
    ros::shutdown();
  }


  ROS_INFO("[FailsafeController]: initialized");

  is_initialized = true;
}

//}

//{ activate()

bool FailsafeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {
    activation_control_command_ = mrs_msgs::AttitudeCommand();
    uav_mass_difference         = 0;
    ROS_WARN("[FailsafeController]: activated without getting the last tracker's command.");
  } else {
    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;
    hover_thrust                = sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
    ROS_INFO("[FailsafeController]: activated with uav_mass_difference %1.2f kg.", uav_mass_difference);
  }

  first_iteration = true;

  ROS_INFO("[FailsafeController]: activated");

  is_active = true;

  return true;
}

//}

//{ deactivate()

void FailsafeController::deactivate(void) {

  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[FailsafeController]: deactivated");
}

//}

//{ update()

const mrs_msgs::AttitudeCommand::ConstPtr FailsafeController::update(const nav_msgs::Odometry::ConstPtr &                        odometry,
                                                                     [[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

  // --------------------------------------------------------------
  // |                 calculate the euler angles                 |
  // --------------------------------------------------------------

  double         yaw, pitch, roll;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry->pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = ros::Time::now();

    setpoint_yaw    = yaw;
    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt = (ros::Time::now() - last_update).toSec();
  }

  if (dt <= 0.001) {

    ROS_WARN("[FailsafeController]: the update was called with too small dt!");
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  last_update = ros::Time::now();

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // decrease the hover thrust
  hover_thrust -= thrust_decrease_rate_ * dt;

  if (!std::isfinite(hover_thrust)) {
    hover_thrust = 0;
    ROS_ERROR("NaN detected in variable \"hover_thrust\", setting it to 0 and returning!!!");
  } else if (hover_thrust > 1.0) {
    hover_thrust = 1.0;
  } else if (hover_thrust < -0.0) {
    hover_thrust = -0.0;
  }

  output_command->euler_attitude.pitch  = 0.0;
  output_command->euler_attitude.roll   = 0.0;
  output_command->euler_attitude.yaw    = setpoint_yaw;
  output_command->thrust = hover_thrust;
  output_command->mode_mask = output_command->MODE_EULER_ATTITUDE;

  last_output_command = output_command;

  return output_command;
}

//}

//{ gettatus()

const mrs_msgs::ControllerStatus::Ptr FailsafeController::getStatus() {

  if (is_initialized) {

    mrs_msgs::ControllerStatus::Ptr controller_status(new mrs_msgs::ControllerStatus);

    if (is_active) {
      controller_status->active = mrs_msgs::ControllerStatus::ACTIVE;
    } else {
      controller_status->active = mrs_msgs::ControllerStatus::NONACTIVE;
    }

    return controller_status;
  } else {

    return mrs_msgs::ControllerStatus::Ptr();
  }
}

//}

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::FailsafeController, mrs_mav_manager::Controller)

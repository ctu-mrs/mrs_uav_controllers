#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_msgs/ControllerStatus.h>
#include <mrs_mav_manager/Controller.h>

#include <mrs_lib/Profiler.h>

namespace mrs_controllers
{

//{ class FailsafeController

class FailsafeController : public mrs_mav_manager::Controller {

public:
  FailsafeController(void);

  void initialize(const ros::NodeHandle &parent_nh);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus::Ptr status();

private:
  double uav_mass_;
  double uav_mass_difference;
  double g_;
  double hover_thrust_a_, hover_thrust_b_;
  double hover_thrust;
  double thrust_decrease_rate_;

  double roll, pitch, yaw;
  double setpoint_yaw;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;

private:
  mrs_lib::Profiler *profiler;
  mrs_lib::Routine * routine_update;
};

FailsafeController::FailsafeController(void) {
}

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

//{ initialize()

void FailsafeController::initialize(const ros::NodeHandle &parent_nh) {

  ros::NodeHandle nh_(parent_nh, "failsafe_controller");

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  nh_.param("hover_thrust/a", hover_thrust_a_, -1000.0);
  nh_.param("hover_thrust/b", hover_thrust_b_, -1000.0);
  nh_.param("thrust_decrease_rate", thrust_decrease_rate_, -1.0);
  nh_.param("uav_mass", uav_mass_, -1.0);
  nh_.param("g", g_, -1.0);

  if (hover_thrust_a_ < -999) {
    ROS_ERROR("[FailsafeController]: hover_thrust/a is not specified!");
    ros::shutdown();
  }

  if (hover_thrust_b_ < -999) {
    ROS_ERROR("[FailsafeController]: hover_thrust/b is not specified!");
    ros::shutdown();
  }

  if (thrust_decrease_rate_ < 0) {
    ROS_ERROR("[FailsafeController]: thrust_decrease_rate is not specified!");
    ros::shutdown();
  }

  if (uav_mass_ < 0) {
    ROS_ERROR("[FailsafeController]: uav_mass is not specified!");
    ros::shutdown();
  }

  if (g_ < 0) {
    ROS_ERROR("[FailsafeController]: g is not specified!");
    ros::shutdown();
  }

  uav_mass_difference = 0;

  // --------------------------------------------------------------
  // |                 calculate the hover thrust                 |
  // --------------------------------------------------------------

  hover_thrust = sqrt(uav_mass_ * g_) * hover_thrust_a_ + hover_thrust_b_;

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler       = new mrs_lib::Profiler(nh_, "FailsafeController");
  routine_update = profiler->registerRoutine("update");

  ROS_INFO("[FailsafeController]: initialized");
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
    hover_thrust                = sqrt((uav_mass_ + uav_mass_difference) * g_) * hover_thrust_a_ + hover_thrust_b_;
    ROS_INFO("[FailsafeController]: activated with uav_mass_difference %1.2f kg.", uav_mass_difference);
  }

  first_iteration = true;

  ROS_INFO("[FailsafeController]: activated");

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

const mrs_msgs::AttitudeCommand::ConstPtr FailsafeController::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                     const mrs_msgs::PositionCommand::ConstPtr &reference) {

  routine_update->start();

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

    routine_update->end();
    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt = (ros::Time::now() - last_update).toSec();
  }

  if (dt <= 0.001) {

    ROS_WARN("[FailsafeController]: the update was called with too small dt!");
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      routine_update->end();
      return last_output_command;

    } else {

      routine_update->end();
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

  output_command->pitch  = 0.0;
  output_command->roll   = 0.0;
  output_command->yaw    = setpoint_yaw;
  output_command->thrust = hover_thrust;

  last_output_command = output_command;

  routine_update->end();
  return output_command;
}

//}

//{ status()

const mrs_msgs::ControllerStatus::Ptr FailsafeController::status() {

  return mrs_msgs::ControllerStatus::Ptr();
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::FailsafeController, mrs_mav_manager::Controller)

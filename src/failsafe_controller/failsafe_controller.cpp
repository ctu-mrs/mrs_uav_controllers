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
                  const double uav_mass, const double g);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  bool is_initialized = false;
  bool is_active      = false;

  double uav_mass_;
  double uav_mass_difference;

  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;
  double                       hover_thrust;

  double thrust_decrease_rate_;
  double initial_thrust_percentage_;

  std::mutex mutex_hover_thrust;

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

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void FailsafeController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                                    const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g) {

  ros::NodeHandle nh_(parent_nh, name_space);

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;
  this->uav_mass_     = uav_mass;
  this->g_            = g;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "FailsafeController");

  param_loader.load_param("thrust_decrease_rate", thrust_decrease_rate_);
  param_loader.load_param("enable_profiler", profiler_enabled_);
  param_loader.load_param("initial_thrust_percentage", initial_thrust_percentage_);

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

/* activate() //{ */

bool FailsafeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  std::scoped_lock lock(mutex_hover_thrust);

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[FailsafeController]: activated without getting the last controller's command.");

    return false;

  } else {

    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;

    activation_control_command_.controller_enforcing_constraints = false;

    hover_thrust = initial_thrust_percentage_ * sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

    ROS_INFO("[FailsafeController]: activated with uav_mass_difference %1.2f kg.", uav_mass_difference);
  }

  first_iteration = true;

  is_active = true;

  return true;
}

//}

/* deactivate() //{ */

void FailsafeController::deactivate(void) {

  is_active           = false;
  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[FailsafeController]: deactivated");
}

//}

/* update() //{ */

const mrs_msgs::AttitudeCommand::ConstPtr FailsafeController::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
                                                                     [[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &reference) {

  // WARNING: this mutex keeps the disarming routine from being called during the same moment, when the update routine is being called
  // If we try to disarm during the update() execution, it will freeze, since the update() is being called by the control manager
  // and the disarm is automatically swithing motors off, which is automatically switching to NullTracker, which means this controller
  // is getting deactivated
  std::scoped_lock lock(mutex_hover_thrust);

  mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

  if (!is_active) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // --------------------------------------------------------------
  // |                 calculate the euler angles                 |
  // --------------------------------------------------------------

  double         yaw, pitch, roll;
  tf::Quaternion uav_atttitude;
  quaternionMsgToTF(uav_state->pose.orientation, uav_atttitude);
  tf::Matrix3x3 m(uav_atttitude);
  m.getRPY(roll, pitch, yaw);

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = ros::Time::now();

    setpoint_yaw    = yaw;
    first_iteration = false;

    ROS_INFO("[FailsafeController]: first iteration");

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
    ROS_ERROR("[FailsafeController]: NaN detected in variable \"hover_thrust\", setting it to 0 and returning!!!");
  } else if (hover_thrust > 1.0) {
    hover_thrust = 1.0;
  } else if (hover_thrust < 0.0) {
    hover_thrust = 0.0;
  }

  output_command->euler_attitude.x   = 0.0;
  output_command->euler_attitude.y   = 0.0;
  output_command->euler_attitude.z   = setpoint_yaw;
  output_command->euler_attitude_set = true;

  output_command->quater_attitude_set = false;
  output_command->attitude_rate_set   = false;

  output_command->thrust    = hover_thrust;
  output_command->mode_mask = output_command->MODE_EULER_ATTITUDE;

  output_command->desired_acceleration.x = 0;
  output_command->desired_acceleration.y = 0;
  output_command->desired_acceleration.z = 0;

  last_output_command = output_command;

  output_command->controller_enforcing_constraints = false;

  return output_command;
}

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus FailsafeController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active;

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

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_msgs/ControllerStatus.h>
#include <mrs_mav_manager/Controller.h>

#include <mrs_controllers/pid_gainsConfig.h>

namespace mrs_controllers
{

//{ class Pid

class Pid {

public:
  Pid(std::string name, double kp, double kd, double ki, double integral_saturation, double saturation, double exp_filter_const);

  double update(double error, double dt);

  void reset(double last_error);

  void setParams(double kp, double kd, double ki, double integral_saturation, double exp_filter_const);

private:
  double integral;
  double last_error;

  // gains
  double kp;
  double kd;
  double ki;

  double exp_filter_const;
  double integral_saturation;
  double saturation;
  double difference;

  std::string name;
};

void Pid::setParams(double kp, double kd, double ki, double integral_saturation, double exp_filter_const) {

  this->kp                  = kp;
  this->kd                  = kd;
  this->ki                  = ki;
  this->integral_saturation = integral_saturation;
  this->exp_filter_const    = exp_filter_const;
}

Pid::Pid(std::string name, double kp, double kd, double ki, double integral_saturation, double saturation, double exp_filter_const) {

  this->name = name;

  this->kp                  = kp;
  this->kd                  = kd;
  this->ki                  = ki;
  this->integral_saturation = integral_saturation;
  this->saturation          = saturation;
  this->exp_filter_const    = exp_filter_const;

  this->integral   = 0;
  this->last_error = 0;
  this->difference = 0;
}

double Pid::update(double error, double dt) {

  // calculate the filtered difference
  difference = (1 - exp_filter_const) * difference + exp_filter_const * ((error - last_error) / dt);
  last_error = error;

  double p_component = kp * error;
  double d_component = kd * difference;
  double i_component = ki * integral;

  // calculate the pid action
  double control_output = p_component + d_component + i_component;

  // saturate the control output
  bool saturated = false;
  if (!std::isfinite(control_output)) {
    control_output = 0;
    ROS_WARN_THROTTLE(1.0, "[PidController]: NaN detected in variable \"control_output\", setting it to 0 and returning!!!");
  } else if (control_output > saturation) {
    control_output = saturation;
    saturated      = true;
  } else if (control_output < -saturation) {
    control_output = -saturation;
    saturated      = true;
  }

  if (saturated) {

    ROS_WARN_THROTTLE(1.0, "[PidController]: The \"%s\" PID is being saturated!", name.c_str());

    // integrate only in the direction oposite to the saturation (antiwindup)
    if (control_output > 0 && error < 0) {
      integral += error * dt;
    } else if (control_output < 0 && error > 0) {
      integral += error * dt;
    }
  } else {
    // if the output is not saturated, we do not care in which direction do we integrate
    integral += error * dt;
  }

  // saturate the integral
  saturated = false;
  if (!std::isfinite(integral)) {
    integral = 0;
    ROS_WARN_THROTTLE(1.0, "[PidController]: NaN detected in variable \"integral\", setting it to 0 and returning!!!");
  } else if (integral > integral_saturation) {
    integral  = integral_saturation;
    saturated = true;
  } else if (integral < -integral_saturation) {
    integral  = -integral_saturation;
    saturated = true;
  }

  if (saturated) {
    ROS_WARN_THROTTLE(1.0, "[PidController]: The \"%s\" PID's integral is being saturated!", name.c_str());
  }

  return control_output;
}

void Pid::reset(double last_error) {

  this->integral   = 0;
  this->difference = 0;
  this->last_error = last_error;
}

//}

//{ class PidController

class PidController : public mrs_mav_manager::Controller {

public:
  PidController(void);

  void initialize(const ros::NodeHandle &parent_nh);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus::Ptr status();

  void dynamicReconfigureCallback(mrs_controllers::pid_gainsConfig &config, uint32_t level);

private:
  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                      config_mutex_;
  typedef mrs_controllers::pid_gainsConfig    Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>        reconfigure_server_;
  void drs_callback(mrs_controllers::pid_gainsConfig &config, uint32_t level);
  mrs_controllers::pid_gainsConfig last_drs_config;

private:
  Pid *pid_pitch;
  Pid *pid_roll;
  Pid *pid_z;

  double uav_mass_;
  double g_;
  double hover_thrust_a_, hover_thrust_b_;
  double hover_thrust;

  double roll, pitch, yaw;

  // gains
  double kpxy_, kixy_, kdxy_;
  double kpz_, kiz_, kdz_;
  double kixy_lim_, kiz_lim_;

  double max_tilt_angle_;
  double exp_;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;
};

PidController::PidController(void) {
}

//}

//{ dynamicReconfigureCallback()

void PidController::dynamicReconfigureCallback(mrs_controllers::pid_gainsConfig &config, uint32_t level) {

  kpxy_     = config.kpxy;
  kdxy_     = config.kdxy;
  kixy_     = config.kixy;
  kpz_      = config.kpz;
  kdz_      = config.kdz;
  kiz_      = config.kiz;
  kixy_lim_ = config.kixy_lim;
  kiz_lim_  = config.kiz_lim;
  exp_      = config.exp;

  pid_pitch->setParams(kpxy_, kdxy_, kixy_, kixy_lim_, exp_);
  pid_roll->setParams(kpxy_, kdxy_, kixy_, kixy_lim_, exp_);
  pid_z->setParams(kpz_, kdz_, kiz_, kiz_lim_, exp_);
}

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

//{ activate()

bool PidController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  activation_control_command_ = *cmd;

  first_iteration = true;

  ROS_INFO("[PidController]: activated");

  return true;
}

//}

//{ deactivate()

void PidController::deactivate(void) {
}

//}

//{ initialize()

void PidController::initialize(const ros::NodeHandle &parent_nh) {

  ros::NodeHandle nh_(parent_nh, "pid_controller");

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  nh_.param("kpxy", kpxy_, -1.0);
  nh_.param("kdxy", kdxy_, -1.0);
  nh_.param("kixy", kixy_, -1.0);
  nh_.param("kpz", kpz_, -1.0);
  nh_.param("kdz", kdz_, -1.0);
  nh_.param("kiz", kiz_, -1.0);
  nh_.param("kixy_lim", kixy_lim_, -1.0);
  nh_.param("kiz_lim", kiz_lim_, -1.0);
  nh_.param("hover_thrust/a", hover_thrust_a_, -1000.0);
  nh_.param("hover_thrust/b", hover_thrust_b_, -1000.0);
  nh_.param("uav_mass", uav_mass_, -1.0);
  nh_.param("g", g_, -1.0);
  nh_.param("exp", exp_, -1.0);

  if (kpxy_ < 0) {
    ROS_ERROR("[PidController]: kpxy is not specified!");
    ros::shutdown();
  }

  if (kdxy_ < 0) {
    ROS_ERROR("[PidController]: kdxy is not specified!");
    ros::shutdown();
  }

  if (kixy_ < 0) {
    ROS_ERROR("[PidController]: kixy is not specified!");
    ros::shutdown();
  }

  if (kpz_ < 0) {
    ROS_ERROR("[PidController]: kpz is not specified!");
    ros::shutdown();
  }

  if (kdz_ < 0) {
    ROS_ERROR("[PidController]: kdz is not specified!");
    ros::shutdown();
  }

  if (kiz_ < 0) {
    ROS_ERROR("[PidController]: kiz is not specified!");
    ros::shutdown();
  }

  if (kixy_lim_ < 0) {
    ROS_ERROR("[PidController]: kixy_lim is not specified!");
    ros::shutdown();
  }

  if (kiz_lim_ < 0) {
    ROS_ERROR("[PidController]: kiz_lim is not specified!");
    ros::shutdown();
  }

  if (hover_thrust_a_ < -999) {
    ROS_ERROR("[LsfController]: hover_thrust/a is not specified!");
    ros::shutdown();
  }

  if (hover_thrust_b_ < -999) {
    ROS_ERROR("[LsfController]: hover_thrust/b is not specified!");
    ros::shutdown();
  }

  if (uav_mass_ < 0) {
    ROS_ERROR("[LsfController]: uav_mass is not specified!");
    ros::shutdown();
  }

  if (g_ < 0) {
    ROS_ERROR("[LsfController]: g is not specified!");
    ros::shutdown();
  }

  if (exp_ < 0) {
    ROS_ERROR("[PidController]: exp is not specified!");
    ros::shutdown();
  }

  nh_.param("max_tilt_angle", max_tilt_angle_, -1.0);
  if (max_tilt_angle_ < 0) {
    ROS_ERROR("[PidController]: max_tilt_angle is not specified!");
    ros::shutdown();
  }

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180) * 3.141592;

  ROS_INFO("[PidController]: PidController was launched with gains:");
  ROS_INFO("[PidController]: horizontal: kpxy: %3.5f, kdxy: %3.5f, kixy: %3.5f, kixy_lim: %3.5f", kpxy_, kdxy_, kixy_, kixy_lim_);
  ROS_INFO("[PidController]: vertical:   kpz: %3.5f, kdz: %3.5f, kiz: %3.5f, kiz_lim: %3.5f", kpz_, kdz_, kiz_, kiz_lim_);
  ROS_INFO("[PidController]: other:      exp: %3.5f", exp_);

  // --------------------------------------------------------------
  // |                 calculate the hover thrust                 |
  // --------------------------------------------------------------

  hover_thrust = sqrt(uav_mass_ * g_) * hover_thrust_a_ + hover_thrust_b_;

  // --------------------------------------------------------------
  // |                       initialize pids                      |
  // --------------------------------------------------------------

  pid_pitch = new Pid("x", kpxy_, kdxy_, kixy_, kixy_lim_, max_tilt_angle_, exp_);
  pid_roll  = new Pid("y", kpxy_, kdxy_, kixy_, kixy_lim_, max_tilt_angle_, exp_);
  pid_z     = new Pid("z", kpz_, kdz_, kiz_, kiz_lim_, 1.0, exp_);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  last_drs_config.kpxy     = kpxy_;
  last_drs_config.kdxy     = kdxy_;
  last_drs_config.kixy     = kixy_;
  last_drs_config.kpz      = kpz_;
  last_drs_config.kdz      = kdz_;
  last_drs_config.kiz      = kiz_;
  last_drs_config.kixy_lim = kixy_lim_;
  last_drs_config.kiz_lim  = kiz_lim_;
  last_drs_config.exp      = exp_;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(last_drs_config);
  ReconfigureServer::CallbackType f = boost::bind(&PidController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

//}

//{ update()

const mrs_msgs::AttitudeCommand::ConstPtr PidController::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                const mrs_msgs::PositionCommand::ConstPtr &reference) {

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  double error_x = reference->position.x - odometry->pose.pose.position.x;
  double error_y = reference->position.y - odometry->pose.pose.position.y;
  double error_z = reference->position.z - odometry->pose.pose.position.z;

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    pid_pitch->reset(error_x);
    pid_roll->reset(error_y);
    pid_z->reset(error_z);
    last_update = ros::Time::now();

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt = (ros::Time::now() - last_update).toSec();
  }

  if (dt <= 0.001) {

    ROS_WARN("[PidController]: the update was called with too small dt!");
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {
      return last_output_command;
    } else {
      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  last_update = ros::Time::now();

  // --------------------------------------------------------------
  // |                 calculate the euler angles                 |
  // --------------------------------------------------------------

  double         yaw, pitch, roll;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry->pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

  // --------------------------------------------------------------
  // |                     calculate the PIDs                     |
  // --------------------------------------------------------------

  double action_pitch = pid_pitch->update(error_x, dt);
  double action_roll  = pid_roll->update(-error_y, dt);
  double action_z     = (pid_z->update(error_z, dt) + hover_thrust) * (1 / (cos(roll) * cos(pitch)));

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->pitch  = action_pitch * cos(yaw) - action_roll * sin(yaw);
  output_command->roll   = action_roll * cos(yaw) + action_pitch * sin(yaw);
  output_command->yaw    = reference->yaw;
  output_command->thrust = action_z;

  last_output_command = output_command;

  return output_command;
}

//}

//{ status()

const mrs_msgs::ControllerStatus::Ptr PidController::status() {

  return mrs_msgs::ControllerStatus::Ptr();
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::PidController, mrs_mav_manager::Controller)  //<reformat_checkpoint>

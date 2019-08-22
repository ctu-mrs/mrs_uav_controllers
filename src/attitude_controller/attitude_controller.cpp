/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_uav_manager/Controller.h>

#include <mrs_controllers/attitude_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

//}

#define X 0
#define Y 1
#define Z 2

namespace mrs_controllers
{

namespace attitude_controller
{

/* //{ class AttitudeController */

class AttitudeController : public mrs_uav_manager::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void dynamicReconfigureCallback(mrs_controllers::attitude_controllerConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  virtual void switchOdometrySource(const nav_msgs::Odometry::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  bool is_initialized = false;
  bool is_active      = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                             config_mutex_;
  typedef mrs_controllers::attitude_controllerConfig Config;
  typedef dynamic_reconfigure::Server<Config>        ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>               reconfigure_server_;
  void                                               drs_callback(mrs_controllers::attitude_controllerConfig &config, uint32_t level);
  mrs_controllers::attitude_controllerConfig         drs_desired_gains;

private:
  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;
  double                       hover_thrust;

  // actual gains (used and already filtered)
  double kpz, kvz, kaz;
  double km, km_lim;
  double kqxy, kqz;  // attitude gains
  double kwxy, kwz;  // attitude rate gains

  // desired gains (set by DRS)
  std::mutex mutex_gains;
  std::mutex mutex_desired_gains;

  double max_tilt_angle_;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;

  bool   mute_lateral_gains               = false;
  bool   mutex_lateral_gains_after_toggle = false;
  double mute_coefficitent_;

private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;

private:
  ros::Timer timer_gain_filter;
  void       timerGainsFilter(const ros::TimerEvent &event);

  int    gains_filter_timer_rate_;
  double gains_filter_change_rate_;
  double gains_filter_min_change_rate_;

  double gains_filter_max_change_;  // calculated from change_rate_/timer_rate_;
  double gains_filter_min_change_;  // calculated from change_rate_/timer_rate_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void AttitudeController::initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params) {

  ros::NodeHandle nh_(parent_nh, "attitude_controller");

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "AttitudeController");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz);
  param_loader.load_param("default_gains/vertical/kv", kvz);
  param_loader.load_param("default_gains/vertical/ka", kaz);

  // attitude gains
  param_loader.load_param("default_gains/horizontal/attitude/kq", kqxy);
  param_loader.load_param("default_gains/vertical/attitude/kq", kqz);

  // attitude rate gains
  param_loader.load_param("default_gains/horizontal/attitude/kw", kwxy);
  param_loader.load_param("default_gains/vertical/attitude/kw", kwz);

  // mass estimator
  param_loader.load_param("default_gains/weight_estimator/km", km);
  param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim);

  // physical
  param_loader.load_param("uav_mass", uav_mass_);
  param_loader.load_param("g", g_);

  // gain filtering
  param_loader.load_param("gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", gains_filter_min_change_rate_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AttitudeController]: Could not load all parameters!");
    ros::shutdown();
  }

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180) * M_PI;

  uav_mass_difference = 0;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_desired_gains.kpz    = kpz;
  drs_desired_gains.kvz    = kvz;
  drs_desired_gains.kaz    = kaz;
  drs_desired_gains.kqxy   = kqxy;
  drs_desired_gains.kqz    = kqz;
  drs_desired_gains.kwxy   = kwxy;
  drs_desired_gains.kwz    = kwz;
  drs_desired_gains.km     = km;
  drs_desired_gains.km_lim = km_lim;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_desired_gains);
  ReconfigureServer::CallbackType f = boost::bind(&AttitudeController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "AttitudeController", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &AttitudeController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AttitudeController]: Could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("[AttitudeController]: initialized");

  is_initialized = true;
}

//}

/* //{ activate() */

bool AttitudeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    activation_control_command_ = mrs_msgs::AttitudeCommand();
    uav_mass_difference         = 0;

    ROS_WARN("[AttitudeController]: activated without getting the last tracker's command.");

  } else {

    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;

    ROS_INFO("[AttitudeController]: setting mass difference from the last AttitudeCmd: %.2f kg", uav_mass_difference);

    ROS_INFO("[AttitudeController]: activated with a last trackers command.");
  }

  first_iteration = true;

  ROS_INFO("[AttitudeController]: activated");

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void AttitudeController::deactivate(void) {

  is_active           = false;
  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[AttitudeController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr AttitudeController::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                     const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

  if (!is_active) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rp - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference
  Eigen::Vector3d           Rp, Rv, Ra, Rw;
  Eigen::Quaternion<double> Rq;

  Eigen::Matrix3d Rd;

  Rp << 0, 0, reference->position.z;  // fill the desired position
  Rv << 0, 0, reference->velocity.z;
  Ra << 0, 0, reference->acceleration.z;
  Rw << 0, 0, reference->yaw_dot;

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(0, 0, odometry->pose.pose.position.z);
  Eigen::Vector3d Ov(0, 0, odometry->twist.twist.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(odometry->twist.twist.angular.x, odometry->twist.twist.angular.y, odometry->twist.twist.angular.z);

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  Eigen::Vector3d Ep = Op - Rp;
  Eigen::Vector3d Ev = Ov - Rv;

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = odometry->header.stamp;

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (odometry->header.stamp - last_update).toSec();
    last_update = odometry->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_WARN_STREAM_THROTTLE(1.0, "[AttitudeController]: last " << last_update << ", current " << odometry->header.stamp);
    ROS_WARN_THROTTLE(1.0, "[AttitudeController]: the last odometry message came too close! %f", dt);
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  // --------------------------------------------------------------
  // |                            gains                           |
  // --------------------------------------------------------------

  Eigen::Vector3d Ka;
  Eigen::Array3d  Kp, Kv, Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains);

    Kp << 0, 0, kpz;
    Kv << 0, 0, kvz;
    Ka << 0, 0, kaz;
    Kq << kqxy, kqxy, kqz;
    Kw << kwxy, kwxy, kwz;
  }

  // --------------------------------------------------------------
  // |                 desired orientation matrix                 |
  // --------------------------------------------------------------
  //
  double total_mass = uav_mass_ + uav_mass_difference;

  Eigen::Vector3d f = -Kp * Ep.array() - Kv * Ev.array() + total_mass * (Eigen::Vector3d(0, 0, g_) + Ra).array();

  Rq.coeffs() << reference->attitude.x, reference->attitude.y, reference->attitude.z, reference->attitude.w;
  Rd = Rq.matrix();

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------

  /* orientation error */
  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;
  Eq << (E(2, 1) - E(1, 2)) / 2.0, (E(0, 2) - E(2, 0)) / 2.0, (E(1, 0) - E(0, 1)) / 2.0;

  // --------------------------------------------------------------
  // |                recalculate the hover thrust                |
  // --------------------------------------------------------------

  hover_thrust = sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

  // --------------------------------------------------------------
  // |                     angular rate error                     |
  // --------------------------------------------------------------
  //
  Eigen::Vector3d Ew;
  Ew = R.transpose() * (Ow - Rw);

  double thrust_force = f.dot(R.col(2));
  double thrust       = 0;

  if (thrust_force >= 0) {
    thrust = sqrt((thrust_force / 10.0) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
  } else {
    ROS_WARN_THROTTLE(1.0, "[AttitudeController]: Just so you know, the desired thrust force is negative (%f", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    ROS_ERROR("[AttitudeController]: NaN detected in variable \"thrust\", setting it to 0 and returning!!!");
  } else if (thrust > 0.8) {
    thrust = 0.8;
  } else if (thrust < 0.0) {
    thrust = 0.0;
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  if (mute_lateral_gains && !reference->disable_position_gains) {
    mutex_lateral_gains_after_toggle = true;
  }
  mute_lateral_gains = reference->disable_position_gains;

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    uav_mass_difference -= km * Ep[2] * dt;

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference)) {
      uav_mass_difference = 0;
      ROS_WARN_THROTTLE(1.0, "[AttitudeController]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!");
    } else if (uav_mass_difference > km_lim) {
      uav_mass_difference = km_lim;
      uav_mass_saturated  = true;
    } else if (uav_mass_difference < -km_lim) {
      uav_mass_difference = -km_lim;
      uav_mass_saturated  = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[AttitudeController]: The uav_mass_difference is being saturated to %1.3f!", uav_mass_difference);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->attitude_rate.x   = t[0];
  output_command->attitude_rate.y   = t[1];
  output_command->attitude_rate.z   = t[2];
  output_command->attitude_rate_set = true;

  Eigen::Quaterniond thrust_vec       = Eigen::Quaterniond(Rd);
  output_command->quter_attitude.w    = thrust_vec.w();
  output_command->quter_attitude.x    = thrust_vec.x();
  output_command->quter_attitude.y    = thrust_vec.y();
  output_command->quter_attitude.z    = thrust_vec.z();
  output_command->quater_attitude_set = true;

  output_command->desired_acceleration.x = f[0] / total_mass;
  output_command->desired_acceleration.y = f[1] / total_mass;
  output_command->desired_acceleration.z = f[2] / total_mass;

  output_command->euler_attitude_set = false;

  output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  output_command->thrust = thrust;

  output_command->mass_difference = uav_mass_difference;

  last_output_command = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus AttitudeController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void AttitudeController::switchOdometrySource([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg) {
}

//}

/* resetDisturbanceEstimators() //{ */

void AttitudeController::resetDisturbanceEstimators(void) {
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void AttitudeController::dynamicReconfigureCallback(mrs_controllers::attitude_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_desired_gains);

    drs_desired_gains = config;
  }

  ROS_INFO("[AttitudeController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void AttitudeController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.01, event);

  double gain_coeff                = 1;
  mutex_lateral_gains_after_toggle = false;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains, mutex_desired_gains);

    kpz    = calculateGainChange(kpz, drs_desired_gains.kpz, false, "kpz");
    kvz    = calculateGainChange(kvz, drs_desired_gains.kvz, false, "kvz");
    kaz    = calculateGainChange(kaz, drs_desired_gains.kaz, false, "kaz");
    kqxy   = calculateGainChange(kqxy, drs_desired_gains.kqxy, false, "kqxy");
    kqz    = calculateGainChange(kqz, drs_desired_gains.kqz, false, "kqz");
    kwxy   = calculateGainChange(kwxy, drs_desired_gains.kwxy, false, "kwxy");
    kwz    = calculateGainChange(kwz, drs_desired_gains.kwz, false, "kwz");
    km     = calculateGainChange(km, drs_desired_gains.km, false, "km");
    km_lim = calculateGainChange(km_lim, drs_desired_gains.km_lim, false, "km_lim");
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* calculateGainChange() //{ */

double AttitudeController::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

  double change = desired_value - current_value;

  if (!bypass_rate) {

    // if current value is near 0...
    double change_in_perc;
    double saturated_change;

    if (fabs(current_value) < 1e-6) {
      change *= gains_filter_max_change_;
    } else {

      saturated_change = change;

      change_in_perc = (current_value + saturated_change) / current_value - 1.0;

      if (change_in_perc > gains_filter_max_change_) {
        saturated_change = current_value * gains_filter_max_change_;
      } else if (change_in_perc < -gains_filter_max_change_) {
        saturated_change = current_value * -gains_filter_max_change_;
      }

      if (fabs(saturated_change) < fabs(change) * gains_filter_min_change_) {
        change *= gains_filter_min_change_;
      } else {
        change = saturated_change;
      }
    }
  }

  if (fabs(change) > 1e-3) {
    ROS_INFO_THROTTLE(1.0, "[AttitudeController]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* rotate2d() //{ */

Eigen::Vector2d AttitudeController::rotate2d(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace attitude_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::attitude_controller::AttitudeController, mrs_uav_manager::Controller)

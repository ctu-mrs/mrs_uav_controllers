#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_msgs/ControllerStatus.h>
#include <mrs_mav_manager/Controller.h>

#include <mrs_controllers/nsf_gainsConfig.h>

#include <mrs_lib/Profiler.h>

#include <mrs_lib/ParamLoader.h>

namespace mrs_controllers
{

/* //{ class NSF */

class Nsf {

public:
  Nsf(std::string name, double kp, double kv, double ka, double kiw, double kib, double integral_saturation, double saturation, double g);
  double update(double position_error, double speed_error, double desired_acceleration, double pitch, double roll, double dt, double hover_thrust,
                double body_integral);
  void   reset(void);
  void   setParams(double kp, double kv, double ka, double kiw, double kib, double integral_saturation);
  bool   isSaturated(void);
  double getWorldIntegral(void);

private:
  double world_integral = 0;

  // gains
  double kp;
  double kv;
  double ka;
  double kiw;
  double kib;

  double integral_saturation;
  double saturation;

  double g;

  bool saturated;

  std::string name;
};

void Nsf::setParams(double kp, double kv, double ka, double kiw, double kib, double integral_saturation) {

  this->kp                  = kp;
  this->kv                  = kv;
  this->ka                  = ka;
  this->kiw                 = kiw;
  this->kib                 = kib;
  this->integral_saturation = integral_saturation;
}

Nsf::Nsf(std::string name, double kp, double kv, double ka, double kiw, double kib, double integral_saturation, double saturation, double g) {

  this->name = name;

  this->kp                  = kp;
  this->kv                  = kv;
  this->ka                  = ka;
  this->kiw                 = kiw;
  this->kib                 = kib;
  this->integral_saturation = integral_saturation;
  this->saturation          = saturation;

  this->g = g;

  this->saturated = false;

  this->world_integral = 0;
}

double Nsf::update(double position_error, double speed_error, double desired_acceleration, double pitch, double roll, double dt, double hover_thrust,
                   double body_integral) {

  double p_component = kp * position_error;
  double v_component = kv * speed_error;
  double i_component = world_integral + body_integral;
  double a_component;

  if (name.compare(std::string("x")) == 0 || name.compare(std::string("y")) == 0) {
    a_component = ka * asin((desired_acceleration * cos(pitch) * cos(roll)) / g);
  } else {
    a_component = ka * desired_acceleration * (hover_thrust / g);
  }

  // calculate the nsf action
  double control_output = p_component + v_component + a_component + i_component;

  saturated = false;
  // saturate the control output
  if (!std::isfinite(control_output)) {
    control_output = 0;
    ROS_INFO("[NsfController]: p_component=%f", p_component);
    ROS_INFO("[NsfController]: v_component=%f", v_component);
    ROS_INFO("[NsfController]: i_component=%f", i_component);
    ROS_INFO("[NsfController]: a_component=%f", a_component);
    ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable \"control_output\", setting it to 0!!!");
  } else if (control_output > saturation) {
    control_output = saturation;
    saturated      = true;
  } else if (control_output < -saturation) {
    control_output = -saturation;
    saturated      = true;
  }

  if (saturated) {

    ROS_WARN_THROTTLE(1.0, "[NsfController]: The \"%s\" NSF is being saturated! value: %f, saturation: %f", name.c_str(), control_output, saturation);

    // integrate only in the direction oposite to the saturation (antiwindup)
    if (control_output > 0 && position_error < 0) {
      world_integral += kiw * position_error * dt;
    } else if (control_output < 0 && position_error > 0) {
      world_integral += kiw * position_error * dt;
    }
  } else {
    // if the output is not saturated, we do not care in which direction do we integrate
    world_integral += kiw * position_error * dt;
  }

  // saturate the world_integral
  double integral_saturated = false;
  if (!std::isfinite(world_integral)) {
    world_integral = 0;
    ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable \"world_integral\", setting it to 0!!!");
  } else if (world_integral > integral_saturation) {
    world_integral     = integral_saturation;
    integral_saturated = true;
  } else if (world_integral < -integral_saturation) {
    world_integral     = -integral_saturation;
    integral_saturated = true;
  }

  if (integral_saturation > 0 && integral_saturated) {
    ROS_WARN_THROTTLE(1.0, "[NsfController]: The \"%s\" NSF's world_integral is being saturated!", name.c_str());
  }

  return control_output;
}

void Nsf::reset(void) {

  this->world_integral = 0;
}

bool Nsf::isSaturated(void) {

  return saturated;
}

double Nsf::getWorldIntegral(void) {

  return world_integral;
}

//}

/* //{ class NsfController */

class NsfController : public mrs_mav_manager::Controller {

public:
  NsfController(void);

  void initialize(const ros::NodeHandle &parent_nh, mrs_mav_manager::MotorParams motor_params);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus::Ptr     getStatus();

  void dynamicReconfigureCallback(mrs_controllers::nsf_gainsConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

private:
  bool is_initialized = false;
  bool is_active      = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                      config_mutex_;
  typedef mrs_controllers::nsf_gainsConfig    Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>        reconfigure_server_;
  void                                        drs_callback(mrs_controllers::nsf_gainsConfig &config, uint32_t level);
  mrs_controllers::nsf_gainsConfig            drs_desired_gains;

private:
  Nsf *nsf_pitch;
  Nsf *nsf_roll;
  Nsf *nsf_z;

  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_mav_manager::MotorParams motor_params_;
  double                       hover_thrust;

  double roll, pitch, yaw;

  double yaw_offset;

  // actual gains (used and already filtered)
  double kpxy, kiwxy, kibxy, kvxy, kaxy;
  double kpz, kvz, kaz;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;

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

private:
  double body_integral_pitch = 0;
  double body_integral_roll  = 0;
};

NsfController::NsfController(void) {
}

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void NsfController::initialize(const ros::NodeHandle &parent_nh, mrs_mav_manager::MotorParams motor_params) {

  ros::NodeHandle nh_(parent_nh, "nsf_controller");

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "NsfController");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // lateral gains
  param_loader.load_param("default_gains/horizontal/kp", kpxy);
  param_loader.load_param("default_gains/horizontal/kv", kvxy);
  param_loader.load_param("default_gains/horizontal/ka", kaxy);

  param_loader.load_param("default_gains/horizontal/kiw", kiwxy);
  param_loader.load_param("default_gains/horizontal/kib", kibxy);

  param_loader.load_param("lateral_mute_coefficitent", mute_coefficitent_);

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz);
  param_loader.load_param("default_gains/vertical/kv", kvz);
  param_loader.load_param("default_gains/vertical/ka", kaz);

  // mass estimator
  param_loader.load_param("default_gains/weight_estimator/km", km);
  param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim);

  // integrator limits
  param_loader.load_param("default_gains/horizontal/kiw_lim", kiwxy_lim);
  param_loader.load_param("default_gains/horizontal/kib_lim", kibxy_lim);

  // physical
  param_loader.load_param("uav_mass", uav_mass_);
  param_loader.load_param("g", g_);

  // constraints
  param_loader.load_param("max_tilt_angle", max_tilt_angle_);

  // yaw offset for compensating for heading error estimation
  param_loader.load_param("yaw_offset", yaw_offset);

  // gain filtering
  param_loader.load_param("gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", gains_filter_min_change_rate_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180) * 3.141592;
  yaw_offset      = (yaw_offset / 180.0) * 3.141592;

  uav_mass_difference = 0;

  // --------------------------------------------------------------
  // |                 calculate the hover thrust                 |
  // --------------------------------------------------------------

  hover_thrust = sqrt(uav_mass_ * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

  // --------------------------------------------------------------
  // |                       initialize nsfs                      |
  // --------------------------------------------------------------

  nsf_pitch = new Nsf("x", kpxy, kvxy, kaxy, kiwxy, kibxy, kiwxy_lim, max_tilt_angle_, g_);
  nsf_roll  = new Nsf("y", kpxy, kvxy, kaxy, kiwxy, kibxy, kiwxy_lim, max_tilt_angle_, g_);
  nsf_z     = new Nsf("z", kpz, kvz, kaz, 0, 0, 0, 1.0, g_);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_desired_gains.kpxy      = kpxy;
  drs_desired_gains.kvxy      = kvxy;
  drs_desired_gains.kaxy      = kaxy;
  drs_desired_gains.kiwxy     = kiwxy;
  drs_desired_gains.kibxy     = kibxy;
  drs_desired_gains.kpz       = kpz;
  drs_desired_gains.kvz       = kvz;
  drs_desired_gains.kaz       = kaz;
  drs_desired_gains.kiwxy_lim = kiwxy_lim;
  drs_desired_gains.kibxy_lim = kibxy_lim;
  drs_desired_gains.km        = km;
  drs_desired_gains.km_lim    = km_lim;
  /* drs_desired_gains.yaw_offset = (yaw_offset / 3.1415) * 180; */

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_desired_gains);
  ReconfigureServer::CallbackType f = boost::bind(&NsfController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "NsfController", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &NsfController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ros::shutdown();
  }

  ROS_INFO("[NsfController]: initialized");

  is_initialized = true;
}

//}

/* //{ activate() */

bool NsfController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {
    activation_control_command_ = mrs_msgs::AttitudeCommand();
    uav_mass_difference         = 0;
    ROS_WARN("[NsfController]: activated without getting the last tracker's command.");
  } else {
    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;
    ROS_INFO("[NsfController]: activated with a last trackers command.");
  }

  first_iteration = true;

  ROS_INFO("[NsfController]: activated");

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void NsfController::deactivate(void) {

  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[NsfController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr NsfController::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  double position_error_x = reference->position.x - odometry->pose.pose.position.x;
  double position_error_y = -(reference->position.y - odometry->pose.pose.position.y);
  double position_error_z = reference->position.z - odometry->pose.pose.position.z;

  double speed_error_x = reference->velocity.x - odometry->twist.twist.linear.x;
  double speed_error_y = -(reference->velocity.y - odometry->twist.twist.linear.y);
  double speed_error_z = reference->velocity.z - odometry->twist.twist.linear.z;

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    nsf_pitch->reset();
    nsf_roll->reset();
    nsf_z->reset();
    last_update = odometry->header.stamp;

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (odometry->header.stamp - last_update).toSec();
    last_update = odometry->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_WARN_STREAM("[NsfController]: last " << last_update << ", current " << odometry->header.stamp);
    ROS_WARN("[NsfController]: the last odometry message came too close! %f", dt);
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  // --------------------------------------------------------------
  // |                 calculate the euler angles                 |
  // --------------------------------------------------------------

  double         yaw, pitch, roll;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry->pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

  // --------------------------------------------------------------
  // |                recalculate the hover thrust                |
  // --------------------------------------------------------------

  hover_thrust = sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  if (!nsf_z->isSaturated()) {

    uav_mass_difference += km * position_error_z * dt;
  }

  // saturate the world_integral
  bool uav_mass_saturated = false;
  if (!std::isfinite(uav_mass_difference)) {
    uav_mass_difference = 0;
    ROS_WARN_THROTTLE(1.0, "[NsfController]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!");
  } else if (uav_mass_difference > km_lim) {
    uav_mass_difference = km_lim;
    uav_mass_saturated  = true;
  } else if (uav_mass_difference < -km_lim) {
    uav_mass_difference = -km_lim;
    uav_mass_saturated  = true;
  }

  if (uav_mass_saturated) {
    ROS_WARN_THROTTLE(1.0, "[NsfController]: The uav_mass_difference is being saturated to %1.3f!", uav_mass_difference);
  }

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  if (mute_lateral_gains && !reference->disable_position_gains) {
    mutex_lateral_gains_after_toggle = true;
  }
  mute_lateral_gains = reference->disable_position_gains;

  // --------------------------------------------------------------
  // |                     calculate the NSFs                     |
  // --------------------------------------------------------------

  double body_integral_x, body_integral_y;
  body_integral_x = body_integral_pitch * cos(-yaw) - body_integral_roll * sin(-yaw);
  body_integral_y = body_integral_pitch * sin(-yaw) + body_integral_roll * cos(-yaw);

  double action_pitch = nsf_pitch->update(position_error_x, speed_error_x, reference->acceleration.x, pitch, roll, dt, hover_thrust, body_integral_x);
  double action_roll  = nsf_roll->update(position_error_y, speed_error_y, -reference->acceleration.y, pitch, roll, dt, hover_thrust, body_integral_y);
  double action_z     = (nsf_z->update(position_error_z, speed_error_z, reference->acceleration.z, pitch, roll, dt, hover_thrust, 0) + hover_thrust) *
                    (1 / (cos(roll) * cos(pitch)));

  // --------------------------------------------------------------
  // |                       body integrals                       |
  // --------------------------------------------------------------

  mutex_gains.lock();
  {
    // rotate the control errors to the body
    double body_error_pitch, body_error_roll;
    body_error_pitch = position_error_x * cos(yaw) - position_error_y * sin(yaw);
    body_error_roll  = position_error_x * sin(yaw) + position_error_y * cos(yaw);

    body_integral_pitch += kibxy * body_error_pitch * dt;
    body_integral_roll += kibxy * body_error_roll * dt;

    // saturate the world_integral
    double integral_saturated = false;
    if (!std::isfinite(body_integral_pitch)) {
      body_integral_pitch = 0;
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable \"body_integral_pitch\", setting it to 0!!!");
    } else if (body_integral_pitch > kibxy_lim) {
      body_integral_pitch = kibxy_lim;
      integral_saturated  = true;
    } else if (body_integral_pitch < -kibxy_lim) {
      body_integral_pitch = -kibxy_lim;
      integral_saturated  = true;
    }

    if (kibxy_lim > 0 && integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NSF's body pitch integral is being saturated!");
    }

    // saturate the world_integral
    integral_saturated = false;
    if (!std::isfinite(body_integral_roll)) {
      body_integral_roll = 0;
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable \"body_integral_roll\", setting it to 0!!!");
    } else if (body_integral_roll > kibxy_lim) {
      body_integral_roll = kibxy_lim;
      integral_saturated = true;
    } else if (body_integral_roll < -kibxy_lim) {
      body_integral_roll = -kibxy_lim;
      integral_saturated = true;
    }

    if (kibxy_lim > 0 && integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NSF's body roll integral is being saturated!");
    }
  }
  mutex_gains.unlock();

  ROS_INFO_THROTTLE(5.0, "[NsfController]: world error integral: x %1.2f, y %1.2f, lim: %1.2f", nsf_pitch->getWorldIntegral(), nsf_roll->getWorldIntegral(),
                    kiwxy_lim);
  ROS_INFO_THROTTLE(5.0, "[NsfController]: body error integral:  x %1.2f, y %1.2f, lim: %1.2f", body_integral_pitch, body_integral_roll, kibxy_lim);

  // | ------------------- produce the output ------------------- |

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->pitch  = action_pitch * cos(yaw + yaw_offset) - action_roll * sin(yaw + yaw_offset);
  output_command->roll   = action_roll * cos(yaw + yaw_offset) + action_pitch * sin(yaw + yaw_offset);
  output_command->yaw    = reference->yaw;
  output_command->thrust = action_z;

  output_command->yaw_rate = reference->yaw_dot;

  output_command->mass_difference = uav_mass_difference;

  last_output_command = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus::Ptr NsfController::getStatus() {

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

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void NsfController::dynamicReconfigureCallback(mrs_controllers::nsf_gainsConfig &config, [[maybe_unused]] uint32_t level) {

  mutex_desired_gains.lock();
  { drs_desired_gains = config; }
  mutex_desired_gains.unlock();

  ROS_INFO("[NsfController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void NsfController::timerGainsFilter([[maybe_unused]] const ros::TimerEvent &event) {

  double gain_coeff                = 1;
  bool   bypass_filter             = mute_lateral_gains || mutex_lateral_gains_after_toggle;
  mutex_lateral_gains_after_toggle = false;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  mutex_desired_gains.lock();
  mutex_gains.lock();
  {
    kpxy      = calculateGainChange(kpxy, drs_desired_gains.kpxy * gain_coeff, bypass_filter, "kpxy");
    kvxy      = calculateGainChange(kvxy, drs_desired_gains.kvxy * gain_coeff, bypass_filter, "kvxy");
    kaxy      = calculateGainChange(kaxy, drs_desired_gains.kaxy * gain_coeff, bypass_filter, "kaxy");
    kiwxy     = calculateGainChange(kiwxy, drs_desired_gains.kiwxy * gain_coeff, bypass_filter, "kiwxy");
    kibxy     = calculateGainChange(kibxy, drs_desired_gains.kibxy * gain_coeff, bypass_filter, "kibxy");
    kpz       = calculateGainChange(kpz, drs_desired_gains.kpz, false, "kpz");
    kvz       = calculateGainChange(kvz, drs_desired_gains.kvz, false, "kvz");
    kaz       = calculateGainChange(kaz, drs_desired_gains.kaz, false, "kaz");
    km        = calculateGainChange(km, drs_desired_gains.km, false, "km");
    kiwxy_lim = calculateGainChange(kiwxy_lim, drs_desired_gains.kiwxy_lim, false, "kiwxy_lim");
    kibxy_lim = calculateGainChange(kibxy_lim, drs_desired_gains.kibxy_lim, false, "kibxy_lim");
    km_lim    = calculateGainChange(km_lim, drs_desired_gains.km_lim, false, "km_lim");
  }
  mutex_gains.unlock();
  mutex_desired_gains.unlock();

  /* yaw_offset = (drs_desired_gains.yaw_offset / 180) * 3.141592; */

  nsf_pitch->setParams(kpxy, kvxy, kaxy, kiwxy, kibxy, kiwxy_lim);
  nsf_roll->setParams(kpxy, kvxy, kaxy, kiwxy, kibxy, kiwxy_lim);
  nsf_z->setParams(kpz, kvz, kaz, 0, 0, 0);
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* calculateGainChange() //{ */

double NsfController::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

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
    ROS_INFO_THROTTLE(1.0, "[NsfController]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::NsfController, mrs_mav_manager::Controller)

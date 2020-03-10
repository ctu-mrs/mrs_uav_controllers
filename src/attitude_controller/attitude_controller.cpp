#define VERSION "0.0.5.0"

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
  void initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                             mutex_drs_;
  typedef mrs_controllers::attitude_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>   Drs_t;
  boost::shared_ptr<Drs_t>                           drs_;
  void                                               callbackDrs(mrs_controllers::attitude_controllerConfig &config, uint32_t level);
  DrsConfig_t                                        drs_gains_;

  // | ---------- thrust generation and mass estimation --------- |

  double                       _uav_mass_;
  double                       uav_mass_difference_;
  double                       _g_;
  mrs_uav_manager::MotorParams _motor_params_;

  // | ------------------- configurable gains ------------------- |

  // gains that are used and already filtered
  double kpz_;     // position z gain
  double kvz_;     // velocity z gain
  double kaz_;     // acceleration z gain (feedforward gain, should be =1)
  double km_;      // mass integrator gain
  double km_lim_;  // mass integrator limit
  double kqxy_;    // attiude pitch/roll gain
  double kqz_;     // attitude yaw gain
  double kwxy_;    // attitude tilt rate pitch/roll gain
  double kwz_;     // attitude yaw rate gain

  std::mutex mutex_gains_;      // locks the gains the are used and filtered
  std::mutex mutex_drs_gains_;  // locks the gains that came from the drs

  // | --------------------- gain filtering --------------------- |

  ros::Timer timer_gain_filter_;
  void       timerGainsFilter(const ros::TimerEvent &event);

  double _gains_filter_timer_rate_;
  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  double _gains_filter_max_change_;  // calculated as change_rate/timer_rate;
  double _gains_filter_min_change_;  // calculated as change_rate/timer_rate;

  // | ------------ controller limits and saturations ----------- |

  double _max_tilt_angle_;
  double _thrust_saturation_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void AttitudeController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                                    const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                                    std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  this->_motor_params_ = motor_params;
  this->_uav_mass_     = uav_mass;
  this->_g_            = g;

  // | --------------------- load parameters -------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "AttitudeController");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[AttitudeController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz_);
  param_loader.load_param("default_gains/vertical/kv", kvz_);
  param_loader.load_param("default_gains/vertical/ka", kaz_);

  // attitude gains
  param_loader.load_param("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.load_param("default_gains/vertical/attitude/kq", kqz_);

  // attitude rate gains
  param_loader.load_param("default_gains/horizontal/attitude/kw", kwxy_);
  param_loader.load_param("default_gains/vertical/attitude/kw", kwz_);

  // mass estimator
  param_loader.load_param("default_gains/weight_estimator/km", km_);
  param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim_);

  // gain filtering
  param_loader.load_param("gains_filter/filter_rate", _gains_filter_timer_rate_);
  param_loader.load_param("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  param_loader.load_param("thrust_saturation", _thrust_saturation_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AttitudeController]: could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  _gains_filter_max_change_ = _gains_filter_change_rate_ / _gains_filter_timer_rate_;
  _gains_filter_min_change_ = _gains_filter_min_change_rate_ / _gains_filter_timer_rate_;

  // convert to radians
  _max_tilt_angle_ = (_max_tilt_angle_ / 180) * M_PI;

  uav_mass_difference_ = 0;

  // | --------------- dynamic reconfigure server --------------- |

  drs_gains_.kpz    = kpz_;
  drs_gains_.kvz    = kvz_;
  drs_gains_.kaz    = kaz_;
  drs_gains_.kqxy   = kqxy_;
  drs_gains_.kqz    = kqz_;
  drs_gains_.kwxy   = kwxy_;
  drs_gains_.kwz    = kwz_;
  drs_gains_.km     = km_;
  drs_gains_.km_lim = km_lim_;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_gains_);
  Drs_t::CallbackType f = boost::bind(&AttitudeController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "AttitudeController", _profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  timer_gain_filter_ = nh_.createTimer(ros::Rate(_gains_filter_timer_rate_), &AttitudeController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[AttitudeController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool AttitudeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[AttitudeController]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *cmd;
    uav_mass_difference_     = cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    ROS_INFO("[AttitudeController]: setting mass difference from the last AttitudeCmd: %.2f kg", uav_mass_difference_);

    ROS_INFO("[AttitudeController]: activated with a last controller's command");
  }

  first_iteration_ = true;

  ROS_INFO("[AttitudeController]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void AttitudeController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[AttitudeController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr AttitudeController::update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                     const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {

    last_update_time_ = uav_state->header.stamp;

    first_iteration_ = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));

  } else {

    dt                = (uav_state->header.stamp - last_update_time_).toSec();
    last_update_time_ = uav_state->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[AttitudeController]: the last odometry message came too close (%.2f s)!", dt);

    if (last_attitude_cmd_ != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_attitude_cmd_;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));
    }
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

  Rp << 0, 0, reference->position.z;
  Rv << 0, 0, reference->velocity.z;
  Ra << 0, 0, reference->acceleration.z;
  Rw << 0, 0, reference->yaw_dot;

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(0, 0, uav_state->pose.position.z);
  Eigen::Vector3d Ov(0, 0, uav_state->velocity.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << uav_state->pose.orientation.x, uav_state->pose.orientation.y, uav_state->pose.orientation.z, uav_state->pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // | -------------- calculate the control errors -------------- |

  Eigen::Vector3d Ep = Op - Rp;
  Eigen::Vector3d Ev = Ov - Rv;

  // | --------------------- load the gains --------------------- |

  Eigen::Vector3d Ka;
  Eigen::Array3d  Kp, Kv, Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains_);

    Kp << 0, 0, kpz_;
    Kv << 0, 0, kvz_;
    Ka << 0, 0, kaz_;
    Kq << kqxy_, kqxy_, kqz_;
    Kw << kwxy_, kwxy_, kwz_;
  }

  // | --------------- desired orientation matrix --------------- |

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d f = -Kp * Ep.array() - Kv * Ev.array() + total_mass * (Eigen::Vector3d(0, 0, _g_) + Ra).array();

  Rq.coeffs() << reference->attitude.x, reference->attitude.y, reference->attitude.z, reference->attitude.w;
  Rd = Rq.matrix();

  // | -------------------- orientation error ------------------- |

  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;
  Eq << (E(2, 1) - E(1, 2)) / 2.0, (E(0, 2) - E(2, 0)) / 2.0, (E(1, 0) - E(0, 1)) / 2.0;

  // | ------------------- angular rate error ------------------- |

  Eigen::Vector3d Ew;
  Ew = R.transpose() * (Ow - Rw);

  double thrust_force = f.dot(R.col(2));
  double thrust       = 0;

  if (thrust_force >= 0) {
    thrust = sqrt(thrust_force) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;
  } else {
    ROS_WARN_THROTTLE(1.0, "[AttitudeController]: just so you know, the desired thrust force is negative (%.2f)", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    ROS_ERROR("[AttitudeController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");
  } else if (thrust > _thrust_saturation_) {
    thrust = _thrust_saturation_;
    ROS_WARN("[AttitudeController]: saturating thrust to %.2f", _thrust_saturation_);
  } else if (thrust < 0.0) {
    thrust = 0.0;
    ROS_WARN("[AttitudeController]: saturating thrust to %.2f", 0.0);
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                       mass estimation                      |
  // --------------------------------------------------------------

  /* mass estimatior //{ */

  {
    std::scoped_lock lock(mutex_gains_);

    uav_mass_difference_ -= km_ * Ep[2] * dt;

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[AttitudeController]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[AttitudeController]: the UAV mass difference is being saturated to %0.2f!", uav_mass_difference_);
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
  output_command->quater_attitude.w   = thrust_vec.w();
  output_command->quater_attitude.x   = thrust_vec.x();
  output_command->quater_attitude.y   = thrust_vec.y();
  output_command->quater_attitude.z   = thrust_vec.z();
  output_command->quater_attitude_set = true;

  output_command->desired_acceleration.x = f[0] / total_mass;
  output_command->desired_acceleration.y = f[1] / total_mass;
  output_command->desired_acceleration.z = f[2] / total_mass;

  output_command->euler_attitude_set = false;

  output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  output_command->thrust = thrust;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  last_attitude_cmd_ = output_command;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "AttitudeController";

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus AttitudeController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void AttitudeController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &msg) {
}

//}

/* resetDisturbanceEstimators() //{ */

void AttitudeController::resetDisturbanceEstimators(void) {
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void AttitudeController::callbackDrs(mrs_controllers::attitude_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_gains_);

    drs_gains_ = config;
  }

  ROS_INFO("[AttitudeController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainsFilter() //{ */

void AttitudeController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerGainsFilter", _gains_filter_timer_rate_, 0.05, event);

  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_gains_);

    kpz_    = calculateGainChange(kpz_, drs_gains_.kpz, false, "kpz");
    kvz_    = calculateGainChange(kvz_, drs_gains_.kvz, false, "kvz");
    kaz_    = calculateGainChange(kaz_, drs_gains_.kaz, false, "kaz");
    kqxy_   = calculateGainChange(kqxy_, drs_gains_.kqxy, false, "kqxy");
    kqz_    = calculateGainChange(kqz_, drs_gains_.kqz, false, "kqz");
    kwxy_   = calculateGainChange(kwxy_, drs_gains_.kwxy, false, "kwxy");
    kwz_    = calculateGainChange(kwz_, drs_gains_.kwz, false, "kwz");
    km_     = calculateGainChange(km_, drs_gains_.km, false, "km");
    km_lim_ = calculateGainChange(km_lim_, drs_gains_.km_lim, false, "km_lim");
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
      change *= _gains_filter_max_change_;
    } else {

      saturated_change = change;

      change_in_perc = (current_value + saturated_change) / current_value - 1.0;

      if (change_in_perc > _gains_filter_max_change_) {
        saturated_change = current_value * _gains_filter_max_change_;
      } else if (change_in_perc < -_gains_filter_max_change_) {
        saturated_change = current_value * -_gains_filter_max_change_;
      }

      if (fabs(saturated_change) < fabs(change) * _gains_filter_min_change_) {
        change *= _gains_filter_min_change_;
      } else {
        change = saturated_change;
      }
    }
  }

  if (fabs(change) > 1e-3) {
    ROS_INFO_THROTTLE(1.0, "[AttitudeController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

}  // namespace attitude_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::attitude_controller::AttitudeController, mrs_uav_manager::Controller)

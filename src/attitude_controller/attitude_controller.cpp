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
  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

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

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  // | --------------------- gain filtering --------------------- |

  void filterGains(const bool mute_gains, const double dt);

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool &updated);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ----------------------- gain muting ---------------------- |

  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;

  // | ------------ controller limits and saturations ----------- |

  double _attitude_rate_saturation_;
  double _tilt_angle_saturation_;
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

void AttitudeController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                                    const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                                    std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _motor_params_   = motor_params;
  _uav_mass_       = uav_mass;
  _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

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
  param_loader.load_param("default_gains/mass_estimator/km", km_);
  param_loader.load_param("default_gains/mass_estimator/km_lim", km_lim_);

  // gain filtering
  param_loader.load_param("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // gain muting
  param_loader.load_param("gain_mute_coefficient", _gain_mute_coefficient_);

  param_loader.load_param("constraints/attitude_rate_saturation", _attitude_rate_saturation_);
  param_loader.load_param("constraints/thrust_saturation", _thrust_saturation_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AttitudeController]: could not load all parameters!");
    ros::shutdown();
  }

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

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[AttitudeController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool AttitudeController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[AttitudeController]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *last_attitude_cmd;
    uav_mass_difference_     = last_attitude_cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    ROS_INFO("[AttitudeController]: setting mass difference from the last AttitudeCmd: %.2f kg", uav_mass_difference_);

    ROS_INFO("[AttitudeController]: activated with a last controller's command");
  }

  first_iteration_ = true;
  gains_muted_     = true;

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
                                                                     const mrs_msgs::PositionCommand::ConstPtr &control_reference) {

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
  Eigen::Matrix3d Rd_integrals;

  Rp << 0, 0, control_reference->position.z;
  Rv << 0, 0, control_reference->velocity.z;
  Ra << 0, 0, control_reference->acceleration.z;
  Rw << 0, 0, control_reference->yaw_dot;

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

  filterGains(control_reference->disable_position_gains, dt);

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

  // construct the desired force vector

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, _g_) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array();

  Eigen::Vector3d f = position_feedback + velocity_feedback + feed_forward;

  // | ------- extract the attitude reference from tracker ------ |

  if (control_reference->use_quat_attitude) {
    Rq.coeffs() << control_reference->attitude.x, control_reference->attitude.y, control_reference->attitude.z, control_reference->attitude.w;
  } else {
    Rq.coeffs() << 0, 0, sin(control_reference->yaw / 2.0), cos(control_reference->yaw / 2.0);
    ROS_ERROR_THROTTLE(1.0, "[AttitudeController]: missing attitude reference, maintaining a leveled attitude");
  }

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
    thrust = sqrt(thrust_force) * _motor_params_.A + _motor_params_.B;
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

  // | --------------- saturate the attitude rate --------------- |

  if (t[0] > _attitude_rate_saturation_) {
    t[0] = _attitude_rate_saturation_;
  } else if (t[0] < -_attitude_rate_saturation_) {
    t[0] = -_attitude_rate_saturation_;
  }

  if (t[1] > _attitude_rate_saturation_) {
    t[1] = _attitude_rate_saturation_;
  } else if (t[1] < -_attitude_rate_saturation_) {
    t[1] = -_attitude_rate_saturation_;
  }

  if (t[2] > _attitude_rate_saturation_) {
    t[2] = _attitude_rate_saturation_;
  } else if (t[2] < -_attitude_rate_saturation_) {
    t[2] = -_attitude_rate_saturation_;
  }

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Quaterniond des_quater = Eigen::Quaterniond(Rd);

    // rotate the drone's z axis
    Eigen::Vector3d uav_z_in_world = des_quater * Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d thrust_vector = thrust_force * uav_z_in_world;

    double world_accel_x = (thrust_vector[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass);
    double world_accel_z = control_reference->acceleration.z;

    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state->header.frame_id;
    world_accel.vector.x        = world_accel_x;
    world_accel.vector.y        = world_accel_y;
    world_accel.vector.z        = world_accel_z;

    auto res = common_handlers_->transformer->transformSingle("fcu", world_accel);

    if (res) {

      desired_x_accel = res.value().vector.x;
      desired_y_accel = res.value().vector.y;
      desired_z_accel = res.value().vector.z;
    }
  }

  // | --------------- fill the resulting command --------------- |

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

  output_command->desired_acceleration.x = desired_x_accel;
  output_command->desired_acceleration.y = desired_y_accel;
  output_command->desired_acceleration.z = desired_z_accel;

  output_command->euler_attitude_set = false;

  output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  output_command->thrust = thrust;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  // since this controller does not estimate disturbances, fill in the
  // original disturbance that were pass in by the previous controller
  output_command->disturbance_bx_b = activation_attitude_cmd_.disturbance_bx_b;
  output_command->disturbance_by_b = activation_attitude_cmd_.disturbance_by_b;

  output_command->disturbance_bx_w = activation_attitude_cmd_.disturbance_bx_w;
  output_command->disturbance_by_w = activation_attitude_cmd_.disturbance_by_w;

  output_command->disturbance_wx_w = activation_attitude_cmd_.disturbance_wx_w;
  output_command->disturbance_wy_w = activation_attitude_cmd_.disturbance_wy_w;

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

void AttitudeController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void AttitudeController::resetDisturbanceEstimators(void) {

  // Since this controller does not estimate distrubances, but it does pass the old onces,
  // set them to zeros here.
  activation_attitude_cmd_.disturbance_bx_b = 0;
  activation_attitude_cmd_.disturbance_by_b = 0;

  activation_attitude_cmd_.disturbance_bx_w = 0;
  activation_attitude_cmd_.disturbance_by_w = 0;

  activation_attitude_cmd_.disturbance_wx_w = 0;
  activation_attitude_cmd_.disturbance_wy_w = 0;
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void AttitudeController::callbackDrs(mrs_controllers::attitude_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_gains_ = config;
  }

  ROS_INFO("[AttitudeController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void AttitudeController::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kpz_  = calculateGainChange(dt, kpz_, drs_gains_.kpz * gain_coeff, bypass_filter, "kpz", updated);
    kvz_  = calculateGainChange(dt, kvz_, drs_gains_.kvz * gain_coeff, bypass_filter, "kvz", updated);
    kaz_  = calculateGainChange(dt, kaz_, drs_gains_.kaz * gain_coeff, bypass_filter, "kaz", updated);
    kqxy_ = calculateGainChange(dt, kqxy_, drs_gains_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kqz_  = calculateGainChange(dt, kqz_, drs_gains_.kqz * gain_coeff, bypass_filter, "kqz", updated);
    kwxy_ = calculateGainChange(dt, kwxy_, drs_gains_.kwxy * gain_coeff, bypass_filter, "kwxy", updated);
    kwz_  = calculateGainChange(dt, kwz_, drs_gains_.kwz * gain_coeff, bypass_filter, "kwz", updated);
    km_   = calculateGainChange(dt, km_, drs_gains_.km * gain_coeff, bypass_filter, "km", updated);

    km_lim_ = calculateGainChange(dt, km_lim_, drs_gains_.km_lim, false, "km_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_gains;

      new_drs_gains.kpz    = kpz_;
      new_drs_gains.kvz    = kvz_;
      new_drs_gains.kaz    = kaz_;
      new_drs_gains.kqxy   = kqxy_;
      new_drs_gains.kqz    = kqz_;
      new_drs_gains.kwxy   = kwxy_;
      new_drs_gains.kwz    = kwz_;
      new_drs_gains.km     = km_;
      new_drs_gains.km_lim = km_lim_;

      drs_->updateConfig(new_drs_gains);
    }
  }
}

//}

/* calculateGainChange() //{ */

double AttitudeController::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate,
                                               std::string name, bool &updated) {

  double change = desired_value - current_value;

  double gains_filter_max_change = _gains_filter_change_rate_ * dt;
  double gains_filter_min_change = _gains_filter_min_change_rate_ * dt;

  if (!bypass_rate) {

    // if current value is near 0...
    double change_in_perc;
    double saturated_change;

    if (fabs(current_value) < 1e-6) {
      change *= gains_filter_max_change;
    } else {

      saturated_change = change;

      change_in_perc = (current_value + saturated_change) / current_value - 1.0;

      if (change_in_perc > gains_filter_max_change) {
        saturated_change = current_value * gains_filter_max_change;
      } else if (change_in_perc < -gains_filter_max_change) {
        saturated_change = current_value * -gains_filter_max_change;
      }

      if (fabs(saturated_change) < fabs(change) * gains_filter_min_change) {
        change *= gains_filter_min_change;
      } else {
        change = saturated_change;
      }
    }
  }

  if (fabs(change) > 1e-3) {
    ROS_INFO_THROTTLE(1.0, "[AttitudeController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace attitude_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::attitude_controller::AttitudeController, mrs_uav_manager::Controller)

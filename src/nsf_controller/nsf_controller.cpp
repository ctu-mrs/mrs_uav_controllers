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

#include <mrs_controllers/nsf_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>
#include <mrs_lib/mutex.h>

//}

#define X 0
#define Y 1
#define Z 2

namespace mrs_controllers
{

namespace nsf_controller
{

/* //{ class NsfController */

class NsfController : public mrs_uav_manager::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef mrs_controllers::nsf_controllerConfig    DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(mrs_controllers::nsf_controllerConfig &config, uint32_t level);
  DrsConfig_t                                      drs_gains_;

  // | ---------- thrust generation and mass estimation --------- |

  double                       _uav_mass_;
  double                       uav_mass_difference_;
  double                       _g_;
  mrs_uav_manager::MotorParams _motor_params_;
  double                       hover_thrust_;

  // | ------------------- configurable gains ------------------- |

  // gains that are used and already filtered
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
  double kaxy_;       // acceleration xy gain (feed forward, =1)
  double kiwxy_;      // world integral xy
  double kibxy_;      // body integral xy
  double kiwxy_lim_;  // world integral xy limit
  double kibxy_lim_;  // body integral xy limit
  double kpz_;        // position z gain
  double kvz_;        // velocity z gain
  double kaz_;        // acceleration z gain (geed forward, =1)
  double km_;         // mass integral
  double km_lim_;     // mass integral limit

  std::mutex mutex_gains;          // locks the gains the are used and filtered
  std::mutex mutex_desired_gains;  // locks the gains that came from the drs

  // | --------------------- gain filtering --------------------- |

  ros::Timer timer_gain_filter_;
  void       timerGainsFilter(const ros::TimerEvent &event);

  double _gains_filter_timer_rate_;
  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  double _gains_filter_max_change_;  // calculated from change_rate/timer_rate;
  double _gains_filter_min_change_;  // calculated from change_rate/timer_rate;

  // | ------------ controller limits and saturations ----------- |

  double max_tilt_angle_;
  double thrust_saturation_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  // | ----------------------- gain muting ---------------------- |

  bool   mute_lateral_gains_               = false;
  bool   mutex_lateral_gains_after_toggle_ = false;
  double _mute_coefficitent_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void NsfController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                               const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                               std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  this->_motor_params_ = motor_params;
  this->_uav_mass_     = uav_mass;
  this->_g_            = g;

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "NsfController");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[NsfController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  // lateral gains
  param_loader.load_param("default_gains/horizontal/kp", kpxy_);
  param_loader.load_param("default_gains/horizontal/kv", kvxy_);
  param_loader.load_param("default_gains/horizontal/ka", kaxy_);

  param_loader.load_param("default_gains/horizontal/kiw", kiwxy_);
  param_loader.load_param("default_gains/horizontal/kib", kibxy_);

  param_loader.load_param("lateral_mute_coefficitent", _mute_coefficitent_);

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz_);
  param_loader.load_param("default_gains/vertical/kv", kvz_);
  param_loader.load_param("default_gains/vertical/ka", kaz_);

  // mass estimator
  param_loader.load_param("default_gains/weight_estimator/km", km_);
  param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim_);

  // integrator limits
  param_loader.load_param("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.load_param("default_gains/horizontal/kib_lim", kibxy_lim_);

  // constraints
  param_loader.load_param("max_tilt_angle", max_tilt_angle_);
  param_loader.load_param("thrust_saturation", thrust_saturation_);

  // gain filtering
  param_loader.load_param("gains_filter/filter_rate", _gains_filter_timer_rate_);
  param_loader.load_param("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[NsfController]: could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  _gains_filter_max_change_ = _gains_filter_change_rate_ / _gains_filter_timer_rate_;
  _gains_filter_min_change_ = _gains_filter_min_change_rate_ / _gains_filter_timer_rate_;

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180.0) * M_PI;

  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = sqrt(_uav_mass_ * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;

  // | --------------- dynamic reconfigure server --------------- |

  drs_gains_.kpxy      = kpxy_;
  drs_gains_.kvxy      = kvxy_;
  drs_gains_.kaxy      = kaxy_;
  drs_gains_.kiwxy     = kiwxy_;
  drs_gains_.kibxy     = kibxy_;
  drs_gains_.kpz       = kpz_;
  drs_gains_.kvz       = kvz_;
  drs_gains_.kaz       = kaz_;
  drs_gains_.kiwxy_lim = kiwxy_lim_;
  drs_gains_.kibxy_lim = kibxy_lim_;
  drs_gains_.km        = km_;
  drs_gains_.km_lim    = km_lim_;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_gains_);
  Drs_t::CallbackType f = boost::bind(&NsfController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "NsfController", _profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  timer_gain_filter_ = nh_.createTimer(ros::Rate(_gains_filter_timer_rate_), &NsfController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[NsfController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool NsfController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[NsfController]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *cmd;
    uav_mass_difference_     = cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    Ib_b_[0] = asin(cmd->disturbance_bx_b / (_g_ * cmd->total_mass));
    Ib_b_[1] = asin(cmd->disturbance_by_b / (_g_ * cmd->total_mass));

    Iw_w_[0] = asin(cmd->disturbance_wx_w / (_g_ * cmd->total_mass));
    Iw_w_[1] = asin(cmd->disturbance_wy_w / (_g_ * cmd->total_mass));

    ROS_INFO(
        "[NsfController]: setting the mass difference and disturbances from the last AttitudeCmd: mass difference: %.2f kg, Ib_b_: %.2f, %.2f N, Iw_w_: %.2f, "
        "%.2f N",
        uav_mass_difference_, cmd->disturbance_bx_b, cmd->disturbance_by_b, cmd->disturbance_wx_w, cmd->disturbance_wx_w);

    ROS_INFO("[NsfController]: activated with a last controller's command.");
  }

  first_iteration_ = true;

  ROS_INFO("[NsfController]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void NsfController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[NsfController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr NsfController::update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }

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

    ROS_DEBUG("[NsfController]: the last odometry message came too close (%.2f s)!", dt);

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
  Eigen::Vector3d Rp(reference->position.x, -reference->position.y, reference->position.z);
  Eigen::Vector3d Rv(reference->velocity.x, -reference->velocity.y, reference->velocity.z);

  // Op - position in global frame
  // Op - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, -uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, -uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // | -------------- calculate the control errors -------------- |

  Eigen::Vector3d Ep = Rp - Op;
  Eigen::Vector3d Ev = Rv - Ov;

  // | ----------- calculate the current Euler angles ----------- |

  double         yaw, pitch, roll;
  tf::Quaternion uav_attitude;
  quaternionMsgToTF(uav_state->pose.orientation, uav_attitude);
  tf::Matrix3x3 m(uav_attitude);
  m.getRPY(roll, pitch, yaw);

  // | -------------- recaltulate the hover thrust -------------- |

  double total_mass = _uav_mass_ + uav_mass_difference_;

  hover_thrust_ = sqrt(total_mass * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;

  // | ----------------------- gain muting ---------------------- |

  if (mute_lateral_gains_ && !reference->disable_position_gains) {
    mutex_lateral_gains_after_toggle_ = true;
  }
  mute_lateral_gains_ = reference->disable_position_gains;

  // --------------------------------------------------------------
  // |                   calculate the feedback                   |
  // --------------------------------------------------------------

  Eigen::Vector2d Ib_w = rotate2d(Ib_b_, -yaw);

  // create vectors of gains
  Eigen::Vector3d kp, kv, ka;

  {
    std::scoped_lock lock(mutex_gains);

    kp << kpxy_, kpxy_, kpz_;
    kv << kvxy_, kvxy_, kvz_;
    ka << kaxy_, kaxy_, kaz_;
  }

  // calculate the feed forwared acceleration
  Eigen::Vector3d feed_forward(asin((reference->acceleration.x * cos(pitch) * cos(roll)) / _g_),
                               asin((-reference->acceleration.y * cos(pitch) * cos(roll)) / _g_), reference->acceleration.z * (hover_thrust_ / _g_));

  // | -------- calculate the componentes of our feedback ------- |
  Eigen::Vector3d p_component, v_component, a_component, i_component;

  p_component = kp.cwiseProduct(Ep);
  v_component = kv.cwiseProduct(Ev);
  a_component = ka.cwiseProduct(feed_forward);
  {
    std::scoped_lock lock(mutex_integrals_);

    i_component << Ib_w + Iw_w_, Eigen::VectorXd::Zero(1, 1);
  }

  Eigen::Vector3d feedback_w = (p_component + v_component + a_component + i_component + Eigen::Vector3d(0, 0, hover_thrust_))
                                   .cwiseProduct(Eigen::Vector3d(1, 1, 1 / (cos(roll) * cos(pitch))));

  // --------------------------------------------------------------
  // |                  validation and saturation                 |
  // --------------------------------------------------------------

  // | ------------ validate and saturate the X and Y components ------------- |

  // check the world X controller
  double x_saturated = false;
  if (!std::isfinite(feedback_w[X])) {
    feedback_w[X] = 0;
    ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'feedback_w[X]', setting it to 0!!!");
  } else if (feedback_w[X] > max_tilt_angle_) {
    feedback_w[X] = max_tilt_angle_;
    x_saturated   = true;
  } else if (feedback_w[X] < -max_tilt_angle_) {
    feedback_w[X] = -max_tilt_angle_;
    x_saturated   = true;
  }

  // check the world Y controller
  double y_saturated = false;
  if (!std::isfinite(feedback_w[Y])) {
    feedback_w[Y] = 0;
    ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'feedback_w[Y]', setting it to 0!!!");
  } else if (feedback_w[Y] > max_tilt_angle_) {
    feedback_w[Y] = max_tilt_angle_;
    y_saturated   = true;
  } else if (feedback_w[Y] < -max_tilt_angle_) {
    feedback_w[Y] = -max_tilt_angle_;
    y_saturated   = true;
  }

  // | ---------------- validate the Z component ---------------- |

  // check the world Z controller
  double z_saturated = false;
  if (!std::isfinite(feedback_w[Z])) {
    feedback_w[Z] = 0;
    ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'feedback_w[Z]', setting it to 0!!!");
  } else if (feedback_w[Z] > thrust_saturation_) {
    feedback_w[Z] = thrust_saturation_;
    z_saturated   = true;
    ROS_WARN("[NsfController]: saturating thrust to %.2f", thrust_saturation_);
  } else if (feedback_w[Z] < 0.0) {
    feedback_w[Z] = 0;
    z_saturated   = true;
    ROS_WARN("[NsfController]: saturating thrust to %.2f", 0.0);
  }

  if (x_saturated) {
    ROS_WARN_THROTTLE(1.0, "[NsfController]: X is saturated");
  }

  if (y_saturated) {
    ROS_WARN_THROTTLE(1.0, "[NsfController]: Y is saturated");
  }

  if (z_saturated) {
    ROS_WARN_THROTTLE(1.0, "[NsfController]: Z is saturated");
  }

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    if (x_saturated && mrs_lib::sign(feedback_w[X]) == mrs_lib::sign(Ep[X])) {
      integration_switch[X] = 0;
    }

    if (y_saturated && mrs_lib::sign(feedback_w[Y]) == mrs_lib::sign(Ep[Y])) {
      integration_switch[Y] = 0;
    }

    // integrate the world error
    Iw_w_ += kiwxy_ * (Ep.cwiseProduct(integration_switch)).head(2) * dt;

    // saturate the world
    double world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NSF's world X integral is being saturated!");
    }

    // saturate the world
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NSF's world Y integral is being saturated!");
    }
  }

  // --------------------------------------------------------------
  // |                  integrate the body error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    // rotate the control errors to the body
    Eigen::Vector2d Ep_body = rotate2d(Ep.head(2), yaw);

    // integrate the body error
    Ib_b_ += kibxy_ * Ep_body * dt;

    // saturate the body
    double body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NSF's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NSF's body roll integral is being saturated!");
    }
  }

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    if (!z_saturated) {
      uav_mass_difference_ += km_ * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[NsfController]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[NsfController]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  // --------------------------------------------------------------
  // |            report on the values of the integrals           |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_integrals_);

    // report in the internal representation of the disturbance -> tilt angle
    double rad_deg = 180.0 / M_PI;

    ROS_INFO_THROTTLE(5.0, "[NsfController]: disturbance in the tilt represenation");
    ROS_INFO_THROTTLE(5.0, "[NsfController]: world error integral: x %.2f deg, y %.2f deg, lim: %.2f deg", rad_deg * Iw_w_[X], rad_deg * Iw_w_[Y],
                      rad_deg * kiwxy_lim_);
    ROS_INFO_THROTTLE(5.0, "[NsfController]: body error integral:  x %.2f deg, y %.2f deg, lim: %.2f deg", rad_deg * Ib_b_[X], rad_deg * Ib_b_[Y],
                      rad_deg * kibxy_lim_);

    // report in the more universal representation -> force
    double hover_force = total_mass * _g_;

    ROS_INFO_THROTTLE(5.0, "[NsfController]: disturbance in the force represenation");
    ROS_INFO_THROTTLE(5.0, "[NsfController]: world error integral: x %.2f N, y %.2f N, lim: %.2f N", hover_force * sin(Iw_w_[X]), hover_force * sin(Iw_w_[Y]),
                      hover_force * sin(kiwxy_lim_));
    ROS_INFO_THROTTLE(5.0, "[NsfController]: body error integral:  x %.2f N, y %.2f N, lim: %.2f N", hover_force * sin(Ib_b_[X]), hover_force * sin(Ib_b_[Y]),
                      hover_force * sin(kibxy_lim_));
  }

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // rotate the feedback to the body frame
  Eigen::Vector2d feedback_b = rotate2d(feedback_w.head(2), yaw);

  output_command->euler_attitude.x   = feedback_b[1];
  output_command->euler_attitude.y   = feedback_b[0];
  output_command->euler_attitude.z   = reference->yaw;  // ISSUE: this will not work with custom heading estimator
  output_command->euler_attitude_set = true;

  output_command->quater_attitude.x = 0;
  output_command->quater_attitude.y = 0;
  output_command->quater_attitude.z = 0;
  output_command->quater_attitude.w = 1;

  output_command->quater_attitude_set = false;
  output_command->attitude_rate_set   = false;

  output_command->thrust = feedback_w[2];

  output_command->mode_mask = output_command->MODE_EULER_ATTITUDE;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = _g_ * total_mass * sin(Ib_b_[0]);
  output_command->disturbance_by_b = _g_ * total_mass * sin(Ib_b_[1]);

  output_command->disturbance_bx_w = _g_ * total_mass * sin(Ib_w[0]);
  output_command->disturbance_by_w = _g_ * total_mass * sin(Ib_w[1]);

  output_command->disturbance_wx_w = _g_ * total_mass * sin(Iw_w_[0]);
  output_command->disturbance_wy_w = _g_ * total_mass * sin(Iw_w_[1]);

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "NsfController";

  last_attitude_cmd_ = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus NsfController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void NsfController::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg) {

  ROS_INFO("[NfsController]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w_[0];
  world_integrals.vector.y = Iw_w_[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(msg->header.frame_id, world_integrals);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = res.value().vector.x;
    Iw_w_[1] = res.value().vector.y;
  } else {

    ROS_ERROR_THROTTLE(1.0, "[NfsController]: could not transform world integral to the new frame");

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = 0;
    Iw_w_[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void NsfController::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w_ = Eigen::Vector2d::Zero(2);
  Ib_b_ = Eigen::Vector2d::Zero(2);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void NsfController::callbackDrs(mrs_controllers::nsf_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_desired_gains);

    drs_gains_ = config;
  }

  ROS_INFO("[NsfController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void NsfController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerGainsFilter", _gains_filter_timer_rate_, 0.05, event);

  double gain_coeff                 = 1;
  bool   bypass_filter              = mute_lateral_gains_ || mutex_lateral_gains_after_toggle_;
  mutex_lateral_gains_after_toggle_ = false;

  if (mute_lateral_gains_) {
    gain_coeff = _mute_coefficitent_;
  }

  {
    std::scoped_lock lock(mutex_gains, mutex_desired_gains);

    kpxy_      = calculateGainChange(kpxy_, drs_gains_.kpxy * gain_coeff, bypass_filter, "kpxy_");
    kvxy_      = calculateGainChange(kvxy_, drs_gains_.kvxy * gain_coeff, bypass_filter, "kvxy_");
    kaxy_      = calculateGainChange(kaxy_, drs_gains_.kaxy * gain_coeff, bypass_filter, "kaxy_");
    kiwxy_     = calculateGainChange(kiwxy_, drs_gains_.kiwxy * gain_coeff, bypass_filter, "kiwxy_");
    kibxy_     = calculateGainChange(kibxy_, drs_gains_.kibxy * gain_coeff, bypass_filter, "kibxy_");
    kpz_       = calculateGainChange(kpz_, drs_gains_.kpz, false, "kpz_");
    kvz_       = calculateGainChange(kvz_, drs_gains_.kvz, false, "kvz_");
    kaz_       = calculateGainChange(kaz_, drs_gains_.kaz, false, "kaz_");
    km_        = calculateGainChange(km_, drs_gains_.km, false, "km_");
    kiwxy_lim_ = calculateGainChange(kiwxy_lim_, drs_gains_.kiwxy_lim, false, "kiwxy_lim_");
    kibxy_lim_ = calculateGainChange(kibxy_lim_, drs_gains_.kibxy_lim, false, "kibxy_lim_");
    km_lim_    = calculateGainChange(km_lim_, drs_gains_.km_lim, false, "km_lim_");
  }
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
    ROS_INFO_THROTTLE(1.0, "[NsfController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* rotate2d() //{ */

Eigen::Vector2d NsfController::rotate2d(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace nsf_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::nsf_controller::NsfController, mrs_uav_manager::Controller)

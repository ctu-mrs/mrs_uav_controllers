#define VERSION "0.0.5.0"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_manager/Controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_controllers/nsf_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/geometry_utils.h>

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
  void initialize(const ros::NodeHandle &parent_nh, const std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
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

  double max_tilt_angle_;
  double thrust_saturation_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

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

void NsfController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
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

  // height gains
  param_loader.load_param("default_gains/vertical/kp", kpz_);
  param_loader.load_param("default_gains/vertical/kv", kvz_);
  param_loader.load_param("default_gains/vertical/ka", kaz_);

  // mass estimator
  param_loader.load_param("default_gains/mass_estimator/km", km_);
  param_loader.load_param("default_gains/mass_estimator/km_lim", km_lim_);

  // integrator limits
  param_loader.load_param("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.load_param("default_gains/horizontal/kib_lim", kibxy_lim_);

  // constraints
  param_loader.load_param("max_tilt_angle", max_tilt_angle_);
  param_loader.load_param("thrust_saturation", thrust_saturation_);

  // gain filtering
  param_loader.load_param("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // gain muting
  param_loader.load_param("gain_mute_coefficient", _gain_mute_coefficient_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[NsfController]: could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180.0) * M_PI;

  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = sqrt(_uav_mass_ * _g_) * _motor_params_.A + _motor_params_.B;

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

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[NsfController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool NsfController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[NsfController]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *last_attitude_cmd;
    uav_mass_difference_     = last_attitude_cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    Ib_b_[0] = asin(-last_attitude_cmd->disturbance_bx_b / (_g_ * last_attitude_cmd->total_mass));
    Ib_b_[1] = asin(-last_attitude_cmd->disturbance_by_b / (_g_ * last_attitude_cmd->total_mass));

    Iw_w_[0] = asin(-last_attitude_cmd->disturbance_wx_w / (_g_ * last_attitude_cmd->total_mass));
    Iw_w_[1] = asin(-last_attitude_cmd->disturbance_wy_w / (_g_ * last_attitude_cmd->total_mass));

    ROS_INFO(
        "[NsfController]: setting the mass difference and disturbances from the last AttitudeCmd: mass difference: %.2f kg, Db_b: %.2f, %.2f N, Dw_w: %.2f, "
        "%.2f N",
        uav_mass_difference_, last_attitude_cmd->disturbance_bx_b, last_attitude_cmd->disturbance_by_b, last_attitude_cmd->disturbance_wx_w,
        last_attitude_cmd->disturbance_wx_w);

    ROS_INFO("[NsfController]: activated with a last controller's command.");
  }

  first_iteration_ = true;
  gains_muted_     = true;

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
                                                                const mrs_msgs::PositionCommand::ConstPtr &control_reference) {

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
  Eigen::Vector3d Rp(control_reference->position.x, -control_reference->position.y, control_reference->position.z);
  Eigen::Vector3d Rv(control_reference->velocity.x, -control_reference->velocity.y, control_reference->velocity.z);

  // Op - position in global frame
  // Op - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, -uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, -uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // | -------------- calculate the control errors -------------- |

  Eigen::Vector3d Ep = Rp - Op;
  Eigen::Vector3d Ev = Rv - Ov;

  // | ----------- calculate the current Euler angles ----------- |

  auto [roll, pitch, yaw] = mrs_lib::AttitudeConvertor(uav_state->pose.orientation).getRPY();

  // | -------------- recaltulate the hover thrust -------------- |

  double total_mass = _uav_mass_ + uav_mass_difference_;

  hover_thrust_ = sqrt(total_mass * _g_) * _motor_params_.A + _motor_params_.B;

  // --------------------------------------------------------------
  // |                   calculate the feedback                   |
  // --------------------------------------------------------------

  Eigen::Vector2d Ib_w = Eigen::Vector2d(0, 0);  // body integral in the world

  // get the position control error in the fcu_untilted frame
  {

    geometry_msgs::Vector3Stamped Ib_b_stamped;

    Ib_b_stamped.header.stamp    = ros::Time::now();
    Ib_b_stamped.header.frame_id = "fcu_untilted";
    Ib_b_stamped.vector.x        = Ib_b_(0);
    Ib_b_stamped.vector.y        = Ib_b_(1);
    Ib_b_stamped.vector.z        = Ib_b_(2);

    auto res = common_handlers_->transformer->transformSingle(uav_state_.header.frame_id, Ib_b_stamped);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[NsfController]: could not transform the position error to fcu_untilted");
    }
  }

  filterGains(control_reference->disable_position_gains, dt);

  // create vectors of gains
  Eigen::Vector3d kp, kv, ka;

  {
    std::scoped_lock lock(mutex_gains_);

    kp << kpxy_, kpxy_, kpz_;
    kv << kvxy_, kvxy_, kvz_;
    ka << kaxy_, kaxy_, kaz_;
  }

  // calculate the feed forwared acceleration
  Eigen::Vector3d feed_forward(asin((control_reference->acceleration.x * cos(pitch) * cos(roll)) / _g_),
                               asin((-control_reference->acceleration.y * cos(pitch) * cos(roll)) / _g_),
                               control_reference->acceleration.z * (hover_thrust_ / _g_));

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
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

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
    std::scoped_lock lock(mutex_gains_);

    // rotate the control errors to the body
    Eigen::Vector2d Ep_fcu_untilted = Eigen::Vector2d(0, 0);  // position error in the untilted frame of the UAV

    // get the position control error in the fcu_untilted frame
    {

      geometry_msgs::Vector3Stamped Ep_stamped;

      Ep_stamped.header.stamp    = ros::Time::now();
      Ep_stamped.header.frame_id = uav_state_.header.frame_id;
      Ep_stamped.vector.x        = Ep(0);
      Ep_stamped.vector.y        = Ep(1);
      Ep_stamped.vector.z        = Ep(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[NsfController]: could not transform the position error to fcu_untilted");
      }
    }

    // integrate the body error
    Ib_b_ += kibxy_ * Ep_fcu_untilted * dt;

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
    std::scoped_lock lock(mutex_gains_);

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
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // rotate the feedback to the body frame
  Eigen::Vector2d feedback_b = Eigen::Vector2d(0, 0);

  // transform the feedback to the body untilted frame
  {

    geometry_msgs::Vector3Stamped Ep_stamped;

    Ep_stamped.header.stamp    = ros::Time::now();
    Ep_stamped.header.frame_id = uav_state_.header.frame_id;
    Ep_stamped.vector.x        = feedback_w(0);
    Ep_stamped.vector.y        = feedback_w(1);
    Ep_stamped.vector.z        = 0;

    auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

    if (res) {
      feedback_b[0] = res.value().vector.x;
      feedback_b[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[So3Controller]: could not transform the feedback to fcu_untilted");
    }
  }

  // roll, pitch, yaw -> quaternion
  output_command->attitude  = mrs_lib::AttitudeConvertor(feedback_b[1], feedback_b[0], control_reference->yaw);
  output_command->mode_mask = output_command->MODE_ATTITUDE;

  output_command->thrust = feedback_w[2];

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = -_g_ * total_mass * sin(Ib_b_[0]);
  output_command->disturbance_by_b = -_g_ * total_mass * sin(Ib_b_[1]);

  output_command->disturbance_bx_w = -_g_ * total_mass * sin(Ib_w[0]);
  output_command->disturbance_by_w = -_g_ * total_mass * sin(Ib_w[1]);

  output_command->disturbance_wx_w = -_g_ * total_mass * sin(Iw_w_[0]);
  output_command->disturbance_wy_w = -_g_ * total_mass * sin(Iw_w_[1]);

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

void NsfController::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) {

  ROS_INFO("[NfsController]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w_[0];
  world_integrals.vector.y = Iw_w_[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(new_uav_state->header.frame_id, world_integrals);

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
    std::scoped_lock lock(mutex_drs_params_);

    drs_gains_ = config;
  }

  ROS_INFO("[NsfController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void NsfController::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kpxy_  = calculateGainChange(dt, kpxy_, drs_gains_.kpxy * gain_coeff, bypass_filter, "kpxy", updated);
    kvxy_  = calculateGainChange(dt, kvxy_, drs_gains_.kvxy * gain_coeff, bypass_filter, "kvxy", updated);
    kaxy_  = calculateGainChange(dt, kaxy_, drs_gains_.kaxy * gain_coeff, bypass_filter, "kaxy", updated);
    kiwxy_ = calculateGainChange(dt, kiwxy_, drs_gains_.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
    kibxy_ = calculateGainChange(dt, kibxy_, drs_gains_.kibxy * gain_coeff, bypass_filter, "kibxy", updated);
    kpz_   = calculateGainChange(dt, kpz_, drs_gains_.kpz * gain_coeff, bypass_filter, "kpz", updated);
    kvz_   = calculateGainChange(dt, kvz_, drs_gains_.kvz * gain_coeff, bypass_filter, "kvz", updated);
    kaz_   = calculateGainChange(dt, kaz_, drs_gains_.kaz * gain_coeff, bypass_filter, "kaz", updated);
    km_    = calculateGainChange(dt, km_, drs_gains_.km * gain_coeff, bypass_filter, "km", updated);

    kiwxy_lim_ = calculateGainChange(dt, kiwxy_lim_, drs_gains_.kiwxy_lim, false, "kiwxy_lim", updated);
    kibxy_lim_ = calculateGainChange(dt, kibxy_lim_, drs_gains_.kibxy_lim, false, "kibxy_lim", updated);
    km_lim_    = calculateGainChange(dt, km_lim_, drs_gains_.km_lim, false, "km_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_gains;

      new_drs_gains.kpxy  = kpxy_;
      new_drs_gains.kvxy  = kvxy_;
      new_drs_gains.kaxy  = kaxy_;
      new_drs_gains.kiwxy = kiwxy_;
      new_drs_gains.kibxy = kibxy_;
      new_drs_gains.kpz   = kpz_;
      new_drs_gains.kvz   = kvz_;
      new_drs_gains.kaz   = kaz_;
      new_drs_gains.km    = km_;

      new_drs_gains.kiwxy_lim = kiwxy_lim_;
      new_drs_gains.kibxy_lim = kibxy_lim_;
      new_drs_gains.km_lim    = km_lim_;

      drs_->updateConfig(new_drs_gains);
    }
  }
}

//}

/* calculateGainChange() //{ */

double NsfController::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name,
                                          bool &updated) {

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
    ROS_INFO_THROTTLE(1.0, "[NsfController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace nsf_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::nsf_controller::NsfController, mrs_uav_manager::Controller)

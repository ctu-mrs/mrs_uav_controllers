#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>

#include <common.h>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_controllers/se3_controllerConfig.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <geometry_msgs/Vector3Stamped.h>

//}

#define OUTPUT_ATTITUDE_RATE 0
#define OUTPUT_ATTITUDE_QUATERNION 1

namespace mrs_uav_controllers
{

namespace se3_controller
{

/* //{ class Se3Controller */

class Se3Controller : public mrs_uav_managers::Controller {

public:
  ~Se3Controller(){};

  void initialize(const ros::NodeHandle& parent_nh, const std::string name, const std::string name_space,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  ControlOutput update(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                            mutex_drs_;
  typedef mrs_uav_controllers::se3_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>  Drs_t;
  boost::shared_ptr<Drs_t>                          drs_;
  void                                              callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, uint32_t level);
  DrsConfig_t                                       drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  bool                          got_constraints_ = false;

  // | ---------- thrust generation and mass estimation --------- |

  double _uav_mass_;
  double uav_mass_difference_;

  // gains that are used and already filtered
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
  double kaxy_;       // acceleration xy gain (feed forward, =1)
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double kpz_;        // position z gain
  double kvz_;        // velocity z gain
  double kaz_;        // acceleration z gain (feed forward, =1)
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kqxy_;       // pitch/roll attitude gain
  double kqz_;        // yaw attitude gain

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  // | ----------------------- gain muting ---------------------- |

  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;

  // | --------------------- gain filtering --------------------- |

  void filterGains(const bool mute_gains, const double dt);

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool& updated);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ------------ controller limits and saturations ----------- |

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;

  double _throttle_saturation_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  // | ----------------------- output mode ---------------------- |

  int        output_mode_;  // attitude_rate / acceleration
  std::mutex mutex_output_mode_;

  // | ------------------------ profiler_ ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

  // | ------------------------- rampup ------------------------- |

  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_throttle_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time_;
  ros::Time rampup_last_time_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void Se3Controller::initialize(const ros::NodeHandle& parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                               std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = common_handlers->getMass();

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "Se3Controller");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Se3Controller]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  // lateral gains
  param_loader.loadParam("default_gains/horizontal/kp", kpxy_);
  param_loader.loadParam("default_gains/horizontal/kv", kvxy_);
  param_loader.loadParam("default_gains/horizontal/ka", kaxy_);

  param_loader.loadParam("default_gains/horizontal/kiw", kiwxy_);
  param_loader.loadParam("default_gains/horizontal/kib", kibxy_);

  // | ------------------------- rampup ------------------------- |

  param_loader.loadParam("rampup/enabled", _rampup_enabled_);
  param_loader.loadParam("rampup/speed", _rampup_speed_);

  // height gains
  param_loader.loadParam("default_gains/vertical/kp", kpz_);
  param_loader.loadParam("default_gains/vertical/kv", kvz_);
  param_loader.loadParam("default_gains/vertical/ka", kaz_);

  // attitude gains
  param_loader.loadParam("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.loadParam("default_gains/vertical/attitude/kq", kqz_);

  // mass estimator
  param_loader.loadParam("default_gains/mass_estimator/km", km_);
  param_loader.loadParam("default_gains/mass_estimator/km_lim", km_lim_);

  // integrator limits
  param_loader.loadParam("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.loadParam("default_gains/horizontal/kib_lim", kibxy_lim_);

  // constraints
  param_loader.loadParam("constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam("constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);
  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[Se3Controller]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  param_loader.loadParam("constraints/thrust_saturation", _throttle_saturation_);

  // gain filtering
  param_loader.loadParam("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.loadParam("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // gain muting
  param_loader.loadParam("gain_mute_coefficient", _gain_mute_coefficient_);

  // output mode
  param_loader.loadParam("output_mode", output_mode_);

  param_loader.loadParam("rotation_matrix", drs_params_.rotation_type);

  // angular rate feed forward
  param_loader.loadParam("angular_rate_feedforward/parasitic_pitch_roll", drs_params_.pitch_roll_heading_rate_compensation);
  param_loader.loadParam("angular_rate_feedforward/jerk", drs_params_.jerk_feedforward);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Se3Controller]: could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[Se3Controller]: output mode has to be {0, 1}!");
    ros::shutdown();
  }

  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | --------------- dynamic reconfigure server --------------- |

  drs_params_.kpxy             = kpxy_;
  drs_params_.kvxy             = kvxy_;
  drs_params_.kaxy             = kaxy_;
  drs_params_.kiwxy            = kiwxy_;
  drs_params_.kibxy            = kibxy_;
  drs_params_.kpz              = kpz_;
  drs_params_.kvz              = kvz_;
  drs_params_.kaz              = kaz_;
  drs_params_.kqxy             = kqxy_;
  drs_params_.kqz              = kqz_;
  drs_params_.kiwxy_lim        = kiwxy_lim_;
  drs_params_.kibxy_lim        = kibxy_lim_;
  drs_params_.km               = km_;
  drs_params_.km_lim           = km_lim_;
  drs_params_.output_mode      = output_mode_;
  drs_params_.jerk_feedforward = true;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&Se3Controller::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "Se3Controller", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[Se3Controller]: initialized, version %s", VERSION);

  is_initialized_ = true;
}  // namespace se3_controller

//}

/* //{ activate() */

bool Se3Controller::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  if (last_control_output.diagnostics.mass_estimator) {
    uav_mass_difference_ = last_control_output.diagnostics.mass_difference;
    ROS_INFO("[Se3Controller]: setting mass difference from the last control output: %.2f kg", uav_mass_difference_);
  }

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  if (last_control_output.diagnostics.disturbance_estimator) {
    Ib_b_[0] = -last_control_output_.diagnostics.disturbance_bx_b;
    Ib_b_[1] = -last_control_output_.diagnostics.disturbance_by_b;

    Iw_w_[0] = -last_control_output_.diagnostics.disturbance_wx_w;
    Iw_w_[1] = -last_control_output_.diagnostics.disturbance_wy_w;

    ROS_INFO(
        "[Se3Controller]: setting disturbances from the last control output: Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, %.2f N",
        Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);
  }

  // did the last controller use manual throttle control?
  auto throttle_last_controller = common::extractThrottle(last_control_output);

  // rampup check
  if (_rampup_enabled_ && throttle_last_controller) {

    double hover_throttle =
        mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, last_control_output.diagnostics.total_mass * common_handlers_->g);
    double thrust_difference = hover_throttle - throttle_last_controller.value();

    if (thrust_difference > 0) {
      rampup_direction_ = 1;
    } else if (thrust_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[Se3Controller]: activating rampup with initial throttle: %.4f, target: %.4f", throttle_last_controller.value(), hover_throttle);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_throttle_   = throttle_last_controller.value();

    rampup_duration_ = fabs(thrust_difference) / _rampup_speed_;
  }

  first_iteration_ = true;
  gains_muted_     = true;

  ROS_INFO("[Se3Controller]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void Se3Controller::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[Se3Controller]: deactivated");
}

//}

/* //{ update() */

Se3Controller::ControlOutput Se3Controller::update(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("Se3Controller::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;
  }

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!is_active_) {
    last_control_output_.control_output = {};
    return last_control_output_;
  }

  if (!tracker_command) {
    last_control_output_.control_output = {};
    return last_control_output_;
  }

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {

    last_update_time_ = uav_state.header.stamp;

    first_iteration_ = false;

    ROS_INFO("[Se3Controller]: first iteration");

    return {activation_control_output_};

  } else {

    dt                = (uav_state.header.stamp - last_update_time_).toSec();
    last_update_time_ = uav_state.header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[Se3Controller]: the last odometry message came too close (%.2f s)!", dt);

    if (last_control_output_.control_output) {
      return last_control_output_;
    } else {
      return {activation_control_output_};
    }
  }

  // | -------------- clean the last control output ------------- |

  last_control_output_.control_output = {};

  // | ----------------- get the current heading ---------------- |

  double uav_heading = 0;

  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    // TODO get heading alternatively
    ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not calculate the UAV heading");
  }

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rw = Eigen::Vector3d::Zero(3);

  if (tracker_command->use_position_vertical || tracker_command->use_position_horizontal) {

    if (tracker_command->use_position_horizontal) {
      Rp[0] = tracker_command->position.x;
      Rp[1] = tracker_command->position.y;
    } else {
      Rv[0] = 0;
      Rv[1] = 0;
    }

    if (tracker_command->use_position_vertical) {
      Rp[2] = tracker_command->position.z;
    } else {
      Rv[2] = 0;
    }
  }

  if (tracker_command->use_velocity_horizontal) {
    Rv[0] = tracker_command->velocity.x;
    Rv[1] = tracker_command->velocity.y;
  } else {
    Rv[0] = 0;
    Rv[1] = 0;
  }

  if (tracker_command->use_velocity_vertical) {
    Rv[2] = tracker_command->velocity.z;
  } else {
    Rv[2] = 0;
  }

  if (tracker_command->use_acceleration) {
    Ra << tracker_command->acceleration.x, tracker_command->acceleration.y, tracker_command->acceleration.z;
  } else {
    Ra << 0, 0, 0;
  }

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  Eigen::Vector3d Ov(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);

  // R - current uav attitude
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);

  // | -------------- calculate the control errors -------------- |

  // position control error
  Eigen::Vector3d Ep = Eigen::Vector3d::Zero(3);

  if (tracker_command->use_position_horizontal || tracker_command->use_position_vertical) {
    Ep = Op - Rp;
  }

  // velocity control error
  Eigen::Vector3d Ev = Eigen::Vector3d::Zero(3);

  if (tracker_command->use_velocity_horizontal || tracker_command->use_velocity_vertical ||
      tracker_command->use_position_vertical) {  // even wehn use_position_vertical to provide dampening
    Ev = Ov - Rv;
  }

  // | --------------------- load the gains --------------------- |

  filterGains(tracker_command->disable_position_gains, dt);

  Eigen::Vector3d Ka = Eigen::Vector3d::Zero(3);
  Eigen::Array3d  Kp = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kv = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kq = Eigen::Array3d::Zero(3);

  {
    std::scoped_lock lock(mutex_gains_);

    if (tracker_command->use_position_horizontal) {
      Kp[0] = kpxy_;
      Kp[1] = kpxy_;
    } else {
      Kp[0] = 0;
      Kp[1] = 0;
    }

    if (tracker_command->use_position_vertical) {
      Kp[2] = kpz_;
    } else {
      Kp[2] = 0;
    }

    if (tracker_command->use_velocity_horizontal) {
      Kv[0] = kvxy_;
      Kv[1] = kvxy_;
    } else {
      Kv[0] = 0;
      Kv[1] = 0;
    }

    // special case: if want to control z-pos but not the velocity => at least provide z dampening, therefore kvz_
    if (tracker_command->use_velocity_vertical || tracker_command->use_position_vertical) {
      Kv[2] = kvz_;
    } else {
      Kv[2] = 0;
    }

    if (tracker_command->use_acceleration) {
      Ka << kaxy_, kaxy_, kaz_;
    } else {
      Ka << 0, 0, 0;
    }

    // Those gains are set regardless of tracker_command setting,
    // because we need to control the attitude.
    Kq << kqxy_, kqxy_, kqz_;
  }

  Kp = Kp * (_uav_mass_ + uav_mass_difference_);
  Kv = Kv * (_uav_mass_ + uav_mass_difference_);

  // | --------------- desired orientation matrix --------------- |

  // get body integral in the world frame

  Eigen::Vector2d Ib_w = Eigen::Vector2d(0, 0);

  {

    geometry_msgs::Vector3Stamped Ib_b_stamped;

    Ib_b_stamped.header.stamp    = ros::Time::now();
    Ib_b_stamped.header.frame_id = "fcu_untilted";
    Ib_b_stamped.vector.x        = Ib_b_(0);
    Ib_b_stamped.vector.y        = Ib_b_(1);
    Ib_b_stamped.vector.z        = 0;

    auto res = common_handlers_->transformer->transformSingle(Ib_b_stamped, uav_state_.header.frame_id);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the Ib_b_ to the world frame");
    }
  }

  // construct the desired force vector

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array();
  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
  }

  Eigen::Vector3d f = position_feedback + velocity_feedback + integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[Se3Controller]: NaN detected in variable 'theta', returning empty command");

    return last_control_output_;
  }

  if (_tilt_angle_failsafe_enabled_ && theta > _tilt_angle_failsafe_) {

    ROS_ERROR("[Se3Controller]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta,
              (180.0 / M_PI) * _tilt_angle_failsafe_);
    ROS_INFO("[Se3Controller]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
    ROS_INFO("[Se3Controller]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
    ROS_INFO("[Se3Controller]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
    ROS_INFO("[Se3Controller]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[Se3Controller]: tracker_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", tracker_command->position.x, tracker_command->position.y,
             tracker_command->position.z, tracker_command->heading);
    ROS_INFO("[Se3Controller]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state.pose.position.x, uav_state.pose.position.y,
             uav_state.pose.position.z, uav_heading);

    return last_control_output_;
  }

  // saturate the angle

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  if (fabs(constraints.tilt) > 1e-3 && theta > constraints.tilt) {
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta / M_PI) * 180.0,
                      (constraints.tilt / M_PI) * 180.0);
    theta = constraints.tilt;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

  // | --------- construct the desired rotational matrix -------- |

  Eigen::Matrix3d Rd;

  if (tracker_command->use_orientation) {

    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(tracker_command->orientation);

    if (tracker_command->use_heading) {
      try {
        Rd = mrs_lib::AttitudeConverter(Rd).setHeading(tracker_command->heading);
      }
      catch (...) {
        ROS_ERROR("[Se3Controller]: could not set the desired heading");
      }
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (tracker_command->use_heading) {
      bxd << cos(tracker_command->heading), sin(tracker_command->heading), 0;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: desired heading was not specified, using current heading instead!");
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    Rd = common::so3transform(f_norm, bxd, drs_params.rotation_type == 1);
  }

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------

  // orientation error
  Eigen::Matrix3d E = Eigen::Matrix3d::Zero();

  if (!tracker_command->use_attitude_rate) {
    E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
  }

  Eigen::Vector3d Eq;

  // clang-format off
  Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
  // clang-format on

  double thrust_force = f.dot(R.col(2));
  double throttle     = 0;

  if (!tracker_command->use_thrust) {
    if (thrust_force >= 0) {
      throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, thrust_force);
    } else {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: just so you know, the desired thrust force is negative (%.2f)", thrust_force);
    }
  } else {
    // the thrust is overriden from the tracker command
    throttle = tracker_command->thrust;
  }

  // saturate the thrust
  if (!std::isfinite(throttle)) {

    throttle = 0;
    ROS_ERROR("[Se3Controller]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");

  } else if (throttle > _throttle_saturation_) {

    throttle = _throttle_saturation_;
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: saturating thrust to %.2f", _throttle_saturation_);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->position.x,
                      tracker_command->position.y, tracker_command->position.z, tracker_command->heading);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->velocity.x,
                      tracker_command->velocity.y, tracker_command->velocity.z, tracker_command->heading_rate);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->acceleration.x,
                      tracker_command->acceleration.y, tracker_command->acceleration.z, tracker_command->heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->jerk.x, tracker_command->jerk.y,
                      tracker_command->jerk.z, tracker_command->heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state.pose.position.x, uav_state.pose.position.y,
                      uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");

  } else if (throttle < 0.0) {

    throttle = 0.0;
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: saturating thrust to 0");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->position.x,
                      tracker_command->position.y, tracker_command->position.z, tracker_command->heading);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->velocity.x,
                      tracker_command->velocity.y, tracker_command->velocity.z, tracker_command->heading_rate);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->acceleration.x,
                      tracker_command->acceleration.y, tracker_command->acceleration.z, tracker_command->heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command->jerk.x, tracker_command->jerk.y,
                      tracker_command->jerk.z, tracker_command->heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state.pose.position.x, uav_state.pose.position.y,
                      uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
  }

  // prepare the attitude feedback
  Eigen::Vector3d q_feedback = -Kq * Eq.array();

  if (tracker_command->use_attitude_rate) {
    Rw << tracker_command->attitude_rate.x, tracker_command->attitude_rate.y, tracker_command->attitude_rate.z;
  } else if (tracker_command->use_heading_rate) {

    // to fill in the feed forward yaw rate
    double desired_yaw_rate = 0;

    try {
      desired_yaw_rate = mrs_lib::AttitudeConverter(Rd).getYawRateIntrinsic(tracker_command->heading_rate);
    }
    catch (...) {
      ROS_ERROR("[Se3Controller]: exception caught while calculating the desired_yaw_rate feedforward");
    }

    Rw << 0, 0, desired_yaw_rate;
  }

  // feedforward angular acceleration
  Eigen::Vector3d q_feedforward = Eigen::Vector3d(0, 0, 0);

  if (drs_params.jerk_feedforward) {

    Eigen::Matrix3d I;
    I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    Eigen::Vector3d desired_jerk = Eigen::Vector3d(tracker_command->jerk.x, tracker_command->jerk.y, tracker_command->jerk.z);
    q_feedforward                = (I.transpose() * Rd.transpose() * desired_jerk) / (thrust_force / total_mass);
  }

  // angular feedback + angular rate feedforward
  Eigen::Vector3d t = q_feedback + Rw + q_feedforward;

  // compensate for the parasitic heading rate created by the desired pitch and roll rate
  Eigen::Vector3d rp_heading_rate_compensation = Eigen::Vector3d(0, 0, 0);

  if (drs_params.pitch_roll_heading_rate_compensation) {

    Eigen::Vector3d q_feedback_yawless = t;
    q_feedback_yawless(2)              = 0;  // nullyfy the effect of the original yaw feedback

    double parasitic_heading_rate = 0;

    try {
      parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeadingRate(q_feedback_yawless);
    }
    catch (...) {
      ROS_ERROR("[Se3Controller]: exception caught while calculating the parasitic heading rate!");
    }

    try {
      rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
    }
    catch (...) {
      ROS_ERROR("[Se3Controller]: exception caught while calculating the parasitic heading rate compensation!");
    }
  }

  t += rp_heading_rate_compensation;

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (tracker_command->use_position_horizontal) {
      Iw_w_ -= kiwxy_ * Ep.head(2) * dt;
    } else if (tracker_command->use_velocity_horizontal) {
      Iw_w_ -= kiwxy_ * Ev.head(2) * dt;
    }

    // saturate the world X
    bool world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's world Y integral is being saturated!");
    }
  }

  //}

  /* body error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the body error                  |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    Eigen::Vector2d Ep_fcu_untilted = Eigen::Vector2d(0, 0);  // position error in the untilted frame of the UAV
    Eigen::Vector2d Ev_fcu_untilted = Eigen::Vector2d(0, 0);  // velocity error in the untilted frame of the UAV

    // get the position control error in the fcu_untilted frame
    {

      geometry_msgs::Vector3Stamped Ep_stamped;

      Ep_stamped.header.stamp    = ros::Time::now();
      Ep_stamped.header.frame_id = uav_state_.header.frame_id;
      Ep_stamped.vector.x        = Ep(0);
      Ep_stamped.vector.y        = Ep(1);
      Ep_stamped.vector.z        = Ep(2);

      auto res = common_handlers_->transformer->transformSingle(Ep_stamped, "fcu_untilted");

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the position error to fcu_untilted");
      }
    }

    // get the velocity control error in the fcu_untilted frame
    {
      geometry_msgs::Vector3Stamped Ev_stamped;

      Ev_stamped.header.stamp    = ros::Time::now();
      Ev_stamped.header.frame_id = uav_state_.header.frame_id;
      Ev_stamped.vector.x        = Ev(0);
      Ev_stamped.vector.y        = Ev(1);
      Ev_stamped.vector.z        = Ev(2);

      auto res = common_handlers_->transformer->transformSingle(Ev_stamped, "fcu_untilted");

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (tracker_command->use_position_horizontal) {
      Ib_b_ -= kibxy_ * Ep_fcu_untilted * dt;
    } else if (tracker_command->use_velocity_horizontal) {
      Ib_b_ -= kibxy_ * Ev_fcu_untilted * dt;
    }

    // saturate the body
    bool body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's body roll integral is being saturated!");
    }
  }

  //}

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    if (tracker_command->use_position_vertical && !rampup_active_) {
      uav_mass_difference_ -= km_ * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {

    Eigen::Matrix3d des_orientation = mrs_lib::AttitudeConverter(Rd);
    Eigen::Vector3d thrust_vector   = thrust_force * des_orientation.col(2);

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
    double world_accel_z = (thrust_vector[2] / total_mass) - common_handlers_->g;

    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state.header.frame_id;
    world_accel.vector.x        = world_accel_x;
    world_accel.vector.y        = world_accel_y;
    world_accel.vector.z        = world_accel_z;

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {

      desired_x_accel = res.value().vector.x;
      desired_y_accel = res.value().vector.y;
      desired_z_accel = res.value().vector.z;
    }
  }

  // | --------------- saturate the attitude rate --------------- |

  if (got_constraints_) {

    auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

    if (t[0] > constraints.roll_rate) {
      t[0] = constraints.roll_rate;
    } else if (t[0] < -constraints.roll_rate) {
      t[0] = -constraints.roll_rate;
    }

    if (t[1] > constraints.pitch_rate) {
      t[1] = constraints.pitch_rate;
    } else if (t[1] < -constraints.pitch_rate) {
      t[1] = -constraints.pitch_rate;
    }

    if (t[2] > constraints.yaw_rate) {
      t[2] = constraints.yaw_rate;
    } else if (t[2] < -constraints.yaw_rate) {
      t[2] = -constraints.yaw_rate;
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: missing dynamics constraints");
  }

  // | --------------- fill the resulting command --------------- |

  // fill the desired orientation for the tilt error check
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(Rd);

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(desired_x_accel, desired_y_accel, desired_z_accel);

  auto output_mode = common::getHighestOuput(common_handlers_->control_output_modalities);

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_ = false;

      ROS_INFO("[Se3Controller]: rampup finished");

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_throttle_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      throttle = rampup_throttle_;

      ROS_INFO_THROTTLE(0.1, "[Se3Controller]: ramping up throttle, %.4f", throttle);
    }
  }

  if (output_mode == common::ATTITUDE_RATE) {

    mrs_msgs::HwApiAttitudeRateCmd cmd;

    cmd.body_rate.x = t[0];
    cmd.body_rate.y = t[1];
    cmd.body_rate.z = t[2];

    cmd.throttle = throttle;

    last_control_output_.control_output = cmd;

  } else if (output_mode == common::ATTITUDE) {

    mrs_msgs::HwApiAttitudeCmd cmd;

    cmd.orientation = mrs_lib::AttitudeConverter(Rd);

    cmd.throttle = throttle;

    last_control_output_.control_output = cmd;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: the controller does not support the required output modality");
  }

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = rampup_active_;

  last_control_output_.diagnostics.mass_estimator = true;
  last_control_output_.diagnostics.mass_difference = uav_mass_difference_;
  last_control_output_.diagnostics.total_mass      = total_mass;

  last_control_output_.diagnostics.disturbance_estimator = true;

  last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_[0];
  last_control_output_.diagnostics.disturbance_by_b = -Ib_b_[1];

  last_control_output_.diagnostics.disturbance_bx_w = -Ib_w[0];
  last_control_output_.diagnostics.disturbance_by_w = -Ib_w[1];

  last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_[0];
  last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_[1];

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3Controller";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus Se3Controller::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void Se3Controller::switchOdometrySource(const mrs_msgs::UavState& new_uav_state) {

  ROS_INFO("[Se3Controller]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w_[0];
  world_integrals.vector.y = Iw_w_[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(world_integrals, new_uav_state.header.frame_id);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = res.value().vector.x;
    Iw_w_[1] = res.value().vector.y;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform world integral to the new frame");

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = 0;
    Iw_w_[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void Se3Controller::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w_ = Eigen::Vector2d::Zero(2);
  Ib_b_ = Eigen::Vector2d::Zero(2);
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr Se3Controller::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  got_constraints_ = true;

  ROS_INFO("[Se3Controller]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void Se3Controller::callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_, mutex_output_mode_);

    drs_params_ = config;

    output_mode_ = config.output_mode;
  }

  ROS_INFO("[Se3Controller]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void Se3Controller::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kpxy_  = calculateGainChange(dt, kpxy_, drs_params_.kpxy * gain_coeff, bypass_filter, "kpxy", updated);
    kvxy_  = calculateGainChange(dt, kvxy_, drs_params_.kvxy * gain_coeff, bypass_filter, "kvxy", updated);
    kaxy_  = calculateGainChange(dt, kaxy_, drs_params_.kaxy * gain_coeff, bypass_filter, "kaxy", updated);
    kiwxy_ = calculateGainChange(dt, kiwxy_, drs_params_.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
    kibxy_ = calculateGainChange(dt, kibxy_, drs_params_.kibxy * gain_coeff, bypass_filter, "kibxy", updated);
    kpz_   = calculateGainChange(dt, kpz_, drs_params_.kpz * gain_coeff, bypass_filter, "kpz", updated);
    kvz_   = calculateGainChange(dt, kvz_, drs_params_.kvz * gain_coeff, bypass_filter, "kvz", updated);
    kaz_   = calculateGainChange(dt, kaz_, drs_params_.kaz * gain_coeff, bypass_filter, "kaz", updated);
    kqxy_  = calculateGainChange(dt, kqxy_, drs_params_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kqz_   = calculateGainChange(dt, kqz_, drs_params_.kqz * gain_coeff, bypass_filter, "kqz", updated);
    km_    = calculateGainChange(dt, km_, drs_params_.km * gain_coeff, bypass_filter, "km", updated);

    kiwxy_lim_ = calculateGainChange(dt, kiwxy_lim_, drs_params_.kiwxy_lim, false, "kiwxy_lim", updated);
    kibxy_lim_ = calculateGainChange(dt, kibxy_lim_, drs_params_.kibxy_lim, false, "kibxy_lim", updated);
    km_lim_    = calculateGainChange(dt, km_lim_, drs_params_.km_lim, false, "km_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_params = drs_params_;

      new_drs_params.kpxy        = kpxy_;
      new_drs_params.kvxy        = kvxy_;
      new_drs_params.kaxy        = kaxy_;
      new_drs_params.kiwxy       = kiwxy_;
      new_drs_params.kibxy       = kibxy_;
      new_drs_params.kpz         = kpz_;
      new_drs_params.kvz         = kvz_;
      new_drs_params.kaz         = kaz_;
      new_drs_params.kqxy        = kqxy_;
      new_drs_params.kqz         = kqz_;
      new_drs_params.kiwxy_lim   = kiwxy_lim_;
      new_drs_params.kibxy_lim   = kibxy_lim_;
      new_drs_params.km          = km_;
      new_drs_params.km_lim      = km_lim_;
      new_drs_params.output_mode = output_mode_;

      drs_->updateConfig(new_drs_params);
    }
  }
}

//}

/* calculateGainChange() //{ */

double Se3Controller::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name,
                                          bool& updated) {

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
    ROS_INFO_THROTTLE(1.0, "[Se3Controller]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace se3_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::se3_controller::Se3Controller, mrs_uav_managers::Controller)

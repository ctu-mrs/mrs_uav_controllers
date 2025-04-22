/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <common.h>
#include <pid.hpp>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_controllers/se3_controllerConfig.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

#include <geometry_msgs/Vector3Stamped.h>

//}

#define OUTPUT_ACTUATORS 0
#define OUTPUT_CONTROL_GROUP 1
#define OUTPUT_ATTITUDE_RATE 2
#define OUTPUT_ATTITUDE 3

namespace mrs_uav_controllers
{

namespace se3_controller
{

/* structs //{ */

typedef struct
{
  double kpxy;           // position xy gain
  double kvxy;           // velocity xy gain
  double kaxy;           // acceleration xy gain (feed forward, =1)
  double kiwxy;          // world xy integral gain
  double kibxy;          // body xy integral gain
  double kiwxy_lim;      // world xy integral limit
  double kibxy_lim;      // body xy integral limit
  double kpz;            // position z gain
  double kvz;            // velocity z gain
  double kaz;            // acceleration z gain (feed forward, =1)
  double km;             // mass estimator gain
  double km_lim;         // mass estimator limit
  double kq_roll_pitch;  // pitch/roll attitude gain
  double kq_yaw;         // yaw attitude gain
  double kw_roll_pitch;  // attitude rate gain
  double kw_yaw;         // attitude rate gain
} Gains_t;

//}

/* //{ class Se3Controller */

class Se3Controller : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

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

  // | ----------------------- controllers ---------------------- |

  void positionPassthrough(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  void PIDVelocityOutput(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command, const common::CONTROL_OUTPUT& control_output,
                         const double& dt);

  void SE3Controller(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command, const double& dt,
                     const common::CONTROL_OUTPUT& output_modality);

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;
  double uav_mass_difference_;

  Gains_t gains_;

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  ros::Timer timer_gains_;
  void       timerGains(const ros::TimerEvent& event);

  double _gain_filtering_rate_;

  // | ----------------------- gain muting ---------------------- |

  std::atomic<bool> mute_gains_            = false;
  std::atomic<bool> mute_gains_by_tracker_ = false;
  double            _gain_mute_coefficient_;

  // | --------------------- gain filtering --------------------- |

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool& updated);

  double getHeadingSafely(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ------------ controller limits and saturations ----------- |

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;

  double _throttle_saturation_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

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

  // | ---------------------- position pid ---------------------- |

  double _pos_pid_p_;
  double _pos_pid_i_;
  double _pos_pid_d_;

  double _hdg_pid_p_;
  double _hdg_pid_i_;
  double _hdg_pid_d_;

  PIDController position_pid_x_;
  PIDController position_pid_y_;
  PIDController position_pid_z_;
  PIDController position_pid_heading_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool Se3Controller::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                               std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  ros::Time::waitForValid();

  // | ---------- loading params using the parent's nh ---------- |

  mrs_lib::ParamLoader param_loader_parent(common_handlers->parent_nh, "ControlManager");

  param_loader_parent.loadParam("enable_profiler", _profiler_enabled_);

  if (!param_loader_parent.loadedSuccessfully()) {
    ROS_ERROR("[MidairActivationController]: Could not load all parameters!");
    return false;
  }

  // | -------------------- loading my params ------------------- |

  private_handlers->param_loader->addYamlFile(ros::package::getPath("mrs_uav_controllers") + "/config/private/se3_controller.yaml");
  private_handlers->param_loader->addYamlFile(ros::package::getPath("mrs_uav_controllers") + "/config/public/se3_controller.yaml");

  const std::string yaml_namespace = "mrs_uav_controllers/se3_controller/";

  // lateral gains
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/kp", gains_.kpxy);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/kv", gains_.kvxy);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/ka", gains_.kaxy);

  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/kiw", gains_.kiwxy);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/kib", gains_.kibxy);

  // | ------------------------- rampup ------------------------- |

  private_handlers->param_loader->loadParam(yaml_namespace + "se3/rampup/enabled", _rampup_enabled_);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/rampup/speed", _rampup_speed_);

  // height gains
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/vertical/kp", gains_.kpz);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/vertical/kv", gains_.kvz);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/vertical/ka", gains_.kaz);

  // attitude gains
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/attitude/kq_roll_pitch", gains_.kq_roll_pitch);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/attitude/kq_yaw", gains_.kq_yaw);

  // attitude rate gains
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/attitude_rate_gains/kw_roll_pitch", gains_.kw_roll_pitch);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/attitude_rate_gains/kw_yaw", gains_.kw_yaw);

  // mass estimator
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/mass_estimator/km", gains_.km);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/mass_estimator/km_lim", gains_.km_lim);

  // integrator limits
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/kiw_lim", gains_.kiwxy_lim);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/default_gains/horizontal/kib_lim", gains_.kibxy_lim);

  // constraints
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);

  _tilt_angle_failsafe_ = M_PI * (_tilt_angle_failsafe_ / 180.0);

  if (_tilt_angle_failsafe_enabled_ && std::abs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[Se3Controller]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    return false;
  }

  private_handlers->param_loader->loadParam(yaml_namespace + "se3/constraints/throttle_saturation", _throttle_saturation_);

  // gain filtering
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/gain_filtering/perc_change_rate", _gains_filter_change_rate_);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/gain_filtering/min_change_rate", _gains_filter_min_change_rate_);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/gain_filtering/rate", _gain_filtering_rate_);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/gain_filtering/gain_mute_coefficient", _gain_mute_coefficient_);

  // output mode
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/preferred_output", drs_params_.preferred_output_mode);

  private_handlers->param_loader->loadParam(yaml_namespace + "se3/rotation_matrix", drs_params_.rotation_type);

  // angular rate feed forward
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/angular_rate_feedforward/parasitic_pitch_roll",
                                            drs_params_.pitch_roll_heading_rate_compensation);
  private_handlers->param_loader->loadParam(yaml_namespace + "se3/angular_rate_feedforward/jerk", drs_params_.jerk_feedforward);

  // | ------------------- position pid params ------------------ |

  private_handlers->param_loader->loadParam(yaml_namespace + "position_controller/translation_gains/p", _pos_pid_p_);
  private_handlers->param_loader->loadParam(yaml_namespace + "position_controller/translation_gains/i", _pos_pid_i_);
  private_handlers->param_loader->loadParam(yaml_namespace + "position_controller/translation_gains/d", _pos_pid_d_);

  private_handlers->param_loader->loadParam(yaml_namespace + "position_controller/heading_gains/p", _hdg_pid_p_);
  private_handlers->param_loader->loadParam(yaml_namespace + "position_controller/heading_gains/i", _hdg_pid_i_);
  private_handlers->param_loader->loadParam(yaml_namespace + "position_controller/heading_gains/d", _hdg_pid_d_);

  // | ------------------ finish loading params ----------------- |

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[Se3Controller]: could not load all parameters!");
    return false;
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(drs_params_.preferred_output_mode == OUTPUT_ACTUATORS || drs_params_.preferred_output_mode == OUTPUT_CONTROL_GROUP ||
        drs_params_.preferred_output_mode == OUTPUT_ATTITUDE_RATE || drs_params_.preferred_output_mode == OUTPUT_ATTITUDE)) {
    ROS_ERROR("[Se3Controller]: preferred output mode has to be {0, 1, 2, 3}!");
    return false;
  }

  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | --------------- dynamic reconfigure server --------------- |

  drs_params_.kpxy             = gains_.kpxy;
  drs_params_.kvxy             = gains_.kvxy;
  drs_params_.kaxy             = gains_.kaxy;
  drs_params_.kiwxy            = gains_.kiwxy;
  drs_params_.kibxy            = gains_.kibxy;
  drs_params_.kpz              = gains_.kpz;
  drs_params_.kvz              = gains_.kvz;
  drs_params_.kaz              = gains_.kaz;
  drs_params_.kq_roll_pitch    = gains_.kq_roll_pitch;
  drs_params_.kq_yaw           = gains_.kq_yaw;
  drs_params_.kiwxy_lim        = gains_.kiwxy_lim;
  drs_params_.kibxy_lim        = gains_.kibxy_lim;
  drs_params_.km               = gains_.km;
  drs_params_.km_lim           = gains_.km_lim;
  drs_params_.jerk_feedforward = true;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&Se3Controller::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------- timers ------------------------- |

  timer_gains_ = nh_.createTimer(ros::Rate(_gain_filtering_rate_), &Se3Controller::timerGains, this, false, false);

  // | ---------------------- position pid ---------------------- |

  position_pid_x_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_y_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_z_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_heading_.setParams(_hdg_pid_p_, _hdg_pid_d_, _hdg_pid_i_, -1, 0.1);

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(common_handlers_->parent_nh, "Se3Controller", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[Se3Controller]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool Se3Controller::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  double activation_mass = _uav_mass_;

  if (activation_control_output_.diagnostics.mass_estimator) {
    uav_mass_difference_ = activation_control_output_.diagnostics.mass_difference;
    activation_mass += uav_mass_difference_;
    ROS_INFO("[Se3Controller]: setting mass difference from the last control output: %.2f kg", uav_mass_difference_);
  }

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  if (activation_control_output_.diagnostics.disturbance_estimator) {
    Ib_b_(0) = -activation_control_output_.diagnostics.disturbance_bx_b;
    Ib_b_(1) = -activation_control_output_.diagnostics.disturbance_by_b;

    Iw_w_(0) = -activation_control_output_.diagnostics.disturbance_wx_w;
    Iw_w_(1) = -activation_control_output_.diagnostics.disturbance_wy_w;

    ROS_INFO(
        "[Se3Controller]: setting disturbances from the last control output: Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, %.2f N",
        Ib_b_(0), Ib_b_(1), Iw_w_(0), Iw_w_(1));
  }

  // did the last controller use manual throttle control?
  auto throttle_last_controller = common::extractThrottle(activation_control_output_);

  // rampup check
  if (_rampup_enabled_ && throttle_last_controller) {

    double hover_throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, activation_mass * common_handlers_->g);

    double throttle_difference = hover_throttle - throttle_last_controller.value();

    if (throttle_difference > 0) {
      rampup_direction_ = 1;
    } else if (throttle_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[Se3Controller]: activating rampup with initial throttle: %.4f, target: %.4f", throttle_last_controller.value(), hover_throttle);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_throttle_   = throttle_last_controller.value();

    rampup_duration_ = std::abs(throttle_difference) / _rampup_speed_;
  }

  first_iteration_ = true;
  mute_gains_      = true;

  timer_gains_.start();

  // | ------------------ finish the activation ----------------- |

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

  timer_gains_.stop();

  ROS_INFO("[Se3Controller]: deactivated");
}

//}

/* updateInactive() //{ */

void Se3Controller::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateWhenActive() */

Se3Controller::ControlOutput Se3Controller::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("updateActive");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("Se3Controller::updateActive", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (std::abs(dt) < 0.001) {

    ROS_DEBUG("[Se3Controller]: the last odometry message came too close (%.2f s)!", dt);

    dt = 0.01;
  }

  // | ----------- obtain the lowest possible modality ---------- |

  auto lowest_modality = common::getLowestOuput(common_handlers_->control_output_modalities);

  if (!lowest_modality) {

    ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: output modalities are empty! This error should never appear.");

    return last_control_output_;
  }

  // | ----- we might prefer some output mode over the other ---- |

  if (drs_params.preferred_output_mode == OUTPUT_ATTITUDE_RATE && common_handlers_->control_output_modalities.attitude_rate) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3Controller]: prioritizing attitude rate output");
    lowest_modality = common::ATTITUDE_RATE;
  } else if (drs_params.preferred_output_mode == OUTPUT_ATTITUDE && common_handlers_->control_output_modalities.attitude) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3Controller]: prioritizing attitude output");
    lowest_modality = common::ATTITUDE;
  } else if (drs_params.preferred_output_mode == OUTPUT_CONTROL_GROUP && common_handlers_->control_output_modalities.control_group) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3Controller]: prioritizing control group output");
    lowest_modality = common::CONTROL_GROUP;
  } else if (drs_params.preferred_output_mode == OUTPUT_ACTUATORS && common_handlers_->control_output_modalities.actuators) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3Controller]: prioritizing actuators output");
    lowest_modality = common::ACTUATORS_CMD;
  }

  switch (lowest_modality.value()) {

    case common::POSITION: {
      positionPassthrough(uav_state, tracker_command);
      break;
    }

    case common::VELOCITY_HDG: {
      PIDVelocityOutput(uav_state, tracker_command, common::VELOCITY_HDG, dt);
      break;
    }

    case common::VELOCITY_HDG_RATE: {
      PIDVelocityOutput(uav_state, tracker_command, common::VELOCITY_HDG_RATE, dt);
      break;
    }

    case common::ACCELERATION_HDG: {
      SE3Controller(uav_state, tracker_command, dt, common::ACCELERATION_HDG);
      break;
    }

    case common::ACCELERATION_HDG_RATE: {
      SE3Controller(uav_state, tracker_command, dt, common::ACCELERATION_HDG_RATE);
      break;
    }

    case common::ATTITUDE: {
      SE3Controller(uav_state, tracker_command, dt, common::ATTITUDE);
      break;
    }

    case common::ATTITUDE_RATE: {
      SE3Controller(uav_state, tracker_command, dt, common::ATTITUDE_RATE);
      break;
    }

    case common::CONTROL_GROUP: {
      SE3Controller(uav_state, tracker_command, dt, common::CONTROL_GROUP);
      break;
    }

    case common::ACTUATORS_CMD: {
      SE3Controller(uav_state, tracker_command, dt, common::ACTUATORS_CMD);
      break;
    }

    default: {
    }
  }

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

  world_integrals.vector.x = Iw_w_(0);
  world_integrals.vector.y = Iw_w_(1);
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(world_integrals, new_uav_state.header.frame_id);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_(0) = res.value().vector.x;
    Iw_w_(1) = res.value().vector.y;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform world integral to the new frame");

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_(0) = 0;
    Iw_w_(1) = 0;
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

  ROS_INFO("[Se3Controller]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                         controllers                        |
// --------------------------------------------------------------

/* SE3Controller() //{ */

void Se3Controller::SE3Controller(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command, const double& dt,
                                  const common::CONTROL_OUTPUT& output_modality) {

  auto drs_params  = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  auto gains       = mrs_lib::get_mutexed(mutex_gains_, gains_);

  // | ----------------- get the current heading ---------------- |

  double uav_heading = getHeadingSafely(uav_state, tracker_command);

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);

  if (tracker_command.use_position_vertical || tracker_command.use_position_horizontal) {

    if (tracker_command.use_position_horizontal) {
      Rp(0) = tracker_command.position.x;
      Rp(1) = tracker_command.position.y;
    } else {
      Rv(0) = 0;
      Rv(1) = 0;
    }

    if (tracker_command.use_position_vertical) {
      Rp(2) = tracker_command.position.z;
    } else {
      Rv(2) = 0;
    }
  }

  if (tracker_command.use_velocity_horizontal) {
    Rv(0) = tracker_command.velocity.x;
    Rv(1) = tracker_command.velocity.y;
  } else {
    Rv(0) = 0;
    Rv(1) = 0;
  }

  if (tracker_command.use_velocity_vertical) {
    Rv(2) = tracker_command.velocity.z;
  } else {
    Rv(2) = 0;
  }

  if (tracker_command.use_acceleration) {
    Ra << tracker_command.acceleration.x, tracker_command.acceleration.y, tracker_command.acceleration.z;
  } else {
    Ra << 0, 0, 0;
  }

  // | ------ store the estimated values from the uav state ----- |

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
  Eigen::Vector3d Ep(0, 0, 0);

  if (tracker_command.use_position_horizontal || tracker_command.use_position_vertical) {
    Ep = Rp - Op;
  }

  // velocity control error
  Eigen::Vector3d Ev(0, 0, 0);

  if (tracker_command.use_velocity_horizontal || tracker_command.use_velocity_vertical ||
      tracker_command.use_position_vertical) {  // use_position_vertical = true, not a mistake, this provides dampening
    Ev = Rv - Ov;
  }

  // | --------------------- load the gains --------------------- |

  mute_gains_by_tracker_ = tracker_command.disable_position_gains;

  Eigen::Vector3d Ka(0, 0, 0);
  Eigen::Array3d  Kp(0, 0, 0);
  Eigen::Array3d  Kv(0, 0, 0);
  Eigen::Array3d  Kq(0, 0, 0);
  Eigen::Array3d  Kw(0, 0, 0);

  {
    std::scoped_lock lock(mutex_gains_);

    if (tracker_command.use_position_horizontal) {
      Kp(0) = gains.kpxy;
      Kp(1) = gains.kpxy;
    } else {
      Kp(0) = 0;
      Kp(1) = 0;
    }

    if (tracker_command.use_position_vertical) {
      Kp(2) = gains.kpz;
    } else {
      Kp(2) = 0;
    }

    if (tracker_command.use_velocity_horizontal) {
      Kv(0) = gains.kvxy;
      Kv(1) = gains.kvxy;
    } else {
      Kv(0) = 0;
      Kv(1) = 0;
    }

    // special case: if want to control z-pos but not the velocity => at least provide z dampening, therefore kvz_
    if (tracker_command.use_velocity_vertical || tracker_command.use_position_vertical) {
      Kv(2) = gains.kvz;
    } else {
      Kv(2) = 0;
    }

    if (tracker_command.use_acceleration) {
      Ka << gains.kaxy, gains.kaxy, gains.kaz;
    } else {
      Ka << 0, 0, 0;
    }

    if (!tracker_command.use_attitude_rate) {
      Kq << gains.kq_roll_pitch, gains.kq_roll_pitch, gains.kq_yaw;
    }

    Kw(0) = gains.kw_roll_pitch;
    Kw(1) = gains.kw_roll_pitch;
    Kw(2) = gains.kw_yaw;
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
      Ib_w(0) = res.value().vector.x;
      Ib_w(1) = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the Ib_b_ to the world frame");
    }
  }

  // construct the desired force vector

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);
  Eigen::Vector3d position_feedback = Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = Kv * Ev.array();
  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w(0) + Iw_w_(0), Ib_w(1) + Iw_w_(1), 0;
  }

  // --------------------------------------------------------------
  // |                 integrators and estimators                 |
  // --------------------------------------------------------------

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (tracker_command.use_position_horizontal) {
      Iw_w_ += gains.kiwxy * Ep.head(2) * dt;
    } else if (tracker_command.use_velocity_horizontal) {
      Iw_w_ += gains.kiwxy * Ev.head(2) * dt;
    }

    // saturate the world X
    bool world_integral_saturated = false;
    if (!std::isfinite(Iw_w_(0))) {
      Iw_w_(0) = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Iw_w_(0)', setting it to 0!!!");
    } else if (Iw_w_(0) > gains.kiwxy_lim) {
      Iw_w_(0)                 = gains.kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w_(0) < -gains.kiwxy_lim) {
      Iw_w_(0)                 = -gains.kiwxy_lim;
      world_integral_saturated = true;
    }

    if (gains.kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_(1))) {
      Iw_w_(1) = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Iw_w_(1)', setting it to 0!!!");
    } else if (Iw_w_(1) > gains.kiwxy_lim) {
      Iw_w_(1)                 = gains.kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w_(1) < -gains.kiwxy_lim) {
      Iw_w_(1)                 = -gains.kiwxy_lim;
      world_integral_saturated = true;
    }

    if (gains.kiwxy_lim >= 0 && world_integral_saturated) {
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
        Ep_fcu_untilted(0) = res.value().vector.x;
        Ep_fcu_untilted(1) = res.value().vector.y;
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
        Ev_fcu_untilted(0) = res.value().vector.x;
        Ev_fcu_untilted(1) = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (tracker_command.use_position_horizontal) {
      Ib_b_ += gains.kibxy * Ep_fcu_untilted * dt;
    } else if (tracker_command.use_velocity_horizontal) {
      Ib_b_ += gains.kibxy * Ev_fcu_untilted * dt;
    }

    // saturate the body
    bool body_integral_saturated = false;
    if (!std::isfinite(Ib_b_(0))) {
      Ib_b_(0) = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Ib_b_(0)', setting it to 0!!!");
    } else if (Ib_b_(0) > gains.kibxy_lim) {
      Ib_b_(0)                = gains.kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b_(0) < -gains.kibxy_lim) {
      Ib_b_(0)                = -gains.kibxy_lim;
      body_integral_saturated = true;
    }

    if (gains.kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_(1))) {
      Ib_b_(1) = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Ib_b_(1)', setting it to 0!!!");
    } else if (Ib_b_(1) > gains.kibxy_lim) {
      Ib_b_(1)                = gains.kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b_(1) < -gains.kibxy_lim) {
      Ib_b_(1)                = -gains.kibxy_lim;
      body_integral_saturated = true;
    }

    if (gains.kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's body roll integral is being saturated!");
    }
  }

  //}

  if (output_modality == common::ACCELERATION_HDG || output_modality == common::ACCELERATION_HDG_RATE) {

    Eigen::Vector3d des_acc = (position_feedback + velocity_feedback + integral_feedback) / total_mass + Ra;

    if (output_modality == common::ACCELERATION_HDG) {

      mrs_msgs::HwApiAccelerationHdgCmd cmd;

      cmd.acceleration.x = des_acc(0);
      cmd.acceleration.y = des_acc(1);
      cmd.acceleration.z = des_acc(2);

      cmd.heading = tracker_command.heading;

      last_control_output_.control_output = cmd;

    } else {

      double des_hdg_ff = 0;

      if (tracker_command.use_heading_rate) {
        des_hdg_ff = tracker_command.heading_rate;
      }

      mrs_msgs::HwApiAccelerationHdgRateCmd cmd;

      cmd.acceleration.x = des_acc(0);
      cmd.acceleration.y = des_acc(1);
      cmd.acceleration.z = des_acc(2);

      position_pid_heading_.setSaturation(constraints.heading_speed);

      double hdg_err = mrs_lib::geometry::sradians::diff(tracker_command.heading, uav_heading);

      double des_hdg_rate = position_pid_heading_.update(hdg_err, dt) + des_hdg_ff;

      cmd.heading_rate = des_hdg_rate;

      last_control_output_.desired_heading_rate = des_hdg_rate;

      last_control_output_.control_output = cmd;
    }

    // | -------------- unbiased desired acceleration ------------- |

    Eigen::Vector3d unbiased_des_acc(0, 0, 0);

    {

      Eigen::Vector3d unbiased_des_acc_world = (position_feedback + velocity_feedback) / total_mass + Ra;

      geometry_msgs::Vector3Stamped world_accel;

      world_accel.header.stamp    = ros::Time::now();
      world_accel.header.frame_id = uav_state.header.frame_id;
      world_accel.vector.x        = unbiased_des_acc_world(0);
      world_accel.vector.y        = unbiased_des_acc_world(1);
      world_accel.vector.z        = unbiased_des_acc_world(2);

      auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

      if (res) {
        unbiased_des_acc << res.value().vector.x, res.value().vector.y, res.value().vector.z;
      }
    }

    // fill the unbiased desired accelerations
    last_control_output_.desired_unbiased_acceleration = unbiased_des_acc;

    // | ----------------- fill in the diagnostics ---------------- |

    last_control_output_.diagnostics.ramping_up = false;

    last_control_output_.diagnostics.mass_estimator  = false;
    last_control_output_.diagnostics.mass_difference = 0;
    last_control_output_.diagnostics.total_mass      = total_mass;

    last_control_output_.diagnostics.disturbance_estimator = true;

    last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_(0);
    last_control_output_.diagnostics.disturbance_by_b = -Ib_b_(1);

    last_control_output_.diagnostics.disturbance_bx_w = -Ib_w(0);
    last_control_output_.diagnostics.disturbance_by_w = -Ib_w(1);

    last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_(0);
    last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_(1);

    last_control_output_.diagnostics.controller_enforcing_constraints = false;

    last_control_output_.diagnostics.controller = "Se3Controller";

    return;
  }

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    if (tracker_command.use_position_vertical && !rampup_active_) {
      uav_mass_difference_ += gains.km * Ep(2) * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > gains.km_lim) {
      uav_mass_difference_ = gains.km_lim;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -gains.km_lim) {
      uav_mass_difference_ = -gains.km_lim;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  Eigen::Vector3d f = position_feedback + velocity_feedback + integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f(2) < 0) {

    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f(2));

    f << 0, 0, 1;
  }

  // | ------------------- sanitize tilt angle ------------------ |

  double tilt_safety_limit = _tilt_angle_failsafe_enabled_ ? _tilt_angle_failsafe_ : std::numeric_limits<double>::max();

  auto f_normed_sanitized = common::sanitizeDesiredForce(f.normalized(), tilt_safety_limit, constraints.tilt, "Se3Controller");

  if (!f_normed_sanitized) {

    ROS_INFO("[Se3Controller]: position feedback: [%.2f, %.2f, %.2f]", position_feedback(0), position_feedback(1), position_feedback(2));
    ROS_INFO("[Se3Controller]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback(0), velocity_feedback(1), velocity_feedback(2));
    ROS_INFO("[Se3Controller]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback(0), integral_feedback(1), integral_feedback(2));
    ROS_INFO("[Se3Controller]: tracker_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", tracker_command.position.x, tracker_command.position.y,
             tracker_command.position.z, tracker_command.heading);
    ROS_INFO("[Se3Controller]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state.pose.position.x, uav_state.pose.position.y,
             uav_state.pose.position.z, uav_heading);

    return;
  }

  Eigen::Vector3d f_normed = f_normed_sanitized.value();

  // --------------------------------------------------------------
  // |               desired orientation + throttle               |
  // --------------------------------------------------------------

  // | ------------------- desired orientation ------------------ |

  Eigen::Matrix3d Rd;

  if (tracker_command.use_orientation) {

    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(tracker_command.orientation);

    if (tracker_command.use_heading) {
      try {
        Rd = mrs_lib::AttitudeConverter(Rd).setHeading(tracker_command.heading);
      }
      catch (...) {
        ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not set the desired heading");
      }
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (tracker_command.use_heading) {
      bxd << cos(tracker_command.heading), sin(tracker_command.heading), 0;
    } else {
      ROS_WARN_THROTTLE(10.0, "[Se3Controller]: desired heading was not specified, using current heading instead!");
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    Rd = common::so3transform(f_normed, bxd, drs_params.rotation_type == 1);
  }

  // | -------------------- desired throttle -------------------- |

  double desired_thrust_force = f.dot(R.col(2));
  double throttle             = 0;

  if (tracker_command.use_throttle) {

    // the throttle is overriden from the tracker command
    throttle = tracker_command.throttle;

  } else if (rampup_active_) {

    // deactivate the rampup when the times up
    if (std::abs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_ = false;

      ROS_INFO("[Se3Controller]: rampup finished");

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_throttle_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      ROS_INFO_THROTTLE(0.1, "[Se3Controller]: ramping up throttle, %.4f", throttle);
    }

    throttle = rampup_throttle_;

  } else {

    if (desired_thrust_force >= 0) {
      throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, desired_thrust_force);
    } else {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: just so you know, the desired throttle force is negative (%.2f)", desired_thrust_force);
    }
  }

  // | ------------------- throttle saturation ------------------ |

  bool throttle_saturated = false;

  if (!std::isfinite(throttle)) {

    ROS_ERROR("[Se3Controller]: NaN detected in variable 'throttle'!!!");
    return;

  } else if (throttle > _throttle_saturation_) {
    throttle = _throttle_saturation_;
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: saturating throttle to %.2f", _throttle_saturation_);
  } else if (throttle < 0.0) {
    throttle = 0.0;
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: saturating throttle to 0.0");
  }

  if (throttle_saturated) {
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.position.x, tracker_command.position.y,
                      tracker_command.position.z, tracker_command.heading);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.velocity.x, tracker_command.velocity.y,
                      tracker_command.velocity.z, tracker_command.heading_rate);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.acceleration.x,
                      tracker_command.acceleration.y, tracker_command.acceleration.z, tracker_command.heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.jerk.x, tracker_command.jerk.y,
                      tracker_command.jerk.z, tracker_command.heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state.pose.position.x, uav_state.pose.position.y,
                      uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
  }

  // | -------------- unbiased desired acceleration ------------- |

  Eigen::Vector3d unbiased_des_acc(0, 0, 0);

  {
    Eigen::Vector3d unbiased_des_acc_world = (position_feedback + velocity_feedback) / total_mass + Ra;

    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state.header.frame_id;
    world_accel.vector.x        = unbiased_des_acc_world(0);
    world_accel.vector.y        = unbiased_des_acc_world(1);
    world_accel.vector.z        = unbiased_des_acc_world(2);

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {
      unbiased_des_acc << res.value().vector.x, res.value().vector.y, res.value().vector.z;
    }
  }

  // | --------------- fill the resulting command --------------- |

  // fill the desired orientation for the tilt error check
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(Rd);

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = unbiased_des_acc;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = rampup_active_;

  last_control_output_.diagnostics.mass_estimator  = true;
  last_control_output_.diagnostics.mass_difference = uav_mass_difference_;
  last_control_output_.diagnostics.total_mass      = total_mass;

  last_control_output_.diagnostics.disturbance_estimator = true;

  last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_(0);
  last_control_output_.diagnostics.disturbance_by_b = -Ib_b_(1);

  last_control_output_.diagnostics.disturbance_bx_w = -Ib_w(0);
  last_control_output_.diagnostics.disturbance_by_w = -Ib_w(1);

  last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_(0);
  last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_(1);

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3Controller";

  // | ------------ construct the attitude reference ------------ |

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.stamp       = ros::Time::now();
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(Rd);
  attitude_cmd.throttle    = throttle;

  if (output_modality == common::ATTITUDE) {

    last_control_output_.control_output = attitude_cmd;

    return;
  }

  // --------------------------------------------------------------
  // |                      attitude control                      |
  // --------------------------------------------------------------

  Eigen::Vector3d rate_feedforward = Eigen::Vector3d::Zero(3);

  if (tracker_command.use_attitude_rate) {

    rate_feedforward << tracker_command.attitude_rate.x, tracker_command.attitude_rate.y, tracker_command.attitude_rate.z;

  } else if (tracker_command.use_heading_rate) {

    // to fill in the feed forward yaw rate
    double desired_yaw_rate = 0;

    try {
      desired_yaw_rate = mrs_lib::AttitudeConverter(Rd).getYawRateIntrinsic(tracker_command.heading_rate);
    }
    catch (...) {
      ROS_ERROR("[Se3Controller]: exception caught while calculating the desired_yaw_rate feedforward");
    }

    rate_feedforward << 0, 0, desired_yaw_rate;
  }

  // | ------------ jerk feedforward -> angular rate ------------ |

  Eigen::Vector3d jerk_feedforward = Eigen::Vector3d(0, 0, 0);

  if (tracker_command.use_jerk && drs_params.jerk_feedforward) {

    ROS_DEBUG_THROTTLE(1.0, "[Se3Controller]: using jerk feedforward");

    Eigen::Matrix3d I;
    I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    Eigen::Vector3d desired_jerk = Eigen::Vector3d(tracker_command.jerk.x, tracker_command.jerk.y, tracker_command.jerk.z);
    jerk_feedforward             = (I.transpose() * Rd.transpose() * desired_jerk) / (desired_thrust_force / total_mass);
  }

  // | --------------- run the attitude controller -------------- |

  Eigen::Vector3d attitude_rate_saturation(constraints.roll_rate, constraints.pitch_rate, constraints.yaw_rate);

  auto attitude_rate_command = common::attitudeController(uav_state, attitude_cmd, jerk_feedforward + rate_feedforward, attitude_rate_saturation, Kq,
                                                          drs_params.pitch_roll_heading_rate_compensation);

  if (!attitude_rate_command) {
    return;
  }

  // | --------- fill in the already known attitude rate -------- |

  {
    try {
      last_control_output_.desired_heading_rate = mrs_lib::AttitudeConverter(R).getHeadingRate(attitude_rate_command->body_rate);
    }
    catch (...) {
    }
  }

  // | ---------- construct the attitude rate reference --------- |

  if (output_modality == common::ATTITUDE_RATE) {

    last_control_output_.control_output = attitude_rate_command;

    return;
  }

  // --------------------------------------------------------------
  // |                    Attitude rate control                   |
  // --------------------------------------------------------------

  Kw = common_handlers_->detailed_model_params->inertia.diagonal().array() * Kw;

  auto control_group_command = common::attitudeRateController(uav_state, attitude_rate_command.value(), Kw);

  if (!control_group_command) {
    return;
  }

  if (output_modality == common::CONTROL_GROUP) {

    last_control_output_.control_output = control_group_command;

    return;
  }

  // --------------------------------------------------------------
  // |                        output mixer                        |
  // --------------------------------------------------------------

  mrs_msgs::HwApiActuatorCmd actuator_cmd = common::actuatorMixer(control_group_command.value(), common_handlers_->detailed_model_params->control_group_mixer);

  last_control_output_.control_output = actuator_cmd;

  return;
}

//}

/* positionPassthrough() //{ */

void Se3Controller::positionPassthrough(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  if (!tracker_command.use_position_vertical || !tracker_command.use_position_horizontal || !tracker_command.use_heading) {
    ROS_ERROR("[Se3Controller]: the tracker did not provide position+hdg reference");
    return;
  }

  mrs_msgs::HwApiPositionCmd cmd;

  cmd.header.frame_id = uav_state.header.frame_id;
  cmd.header.stamp    = ros::Time::now();

  cmd.position = tracker_command.position;
  cmd.heading  = tracker_command.heading;

  last_control_output_.control_output = cmd;

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_heading_rate          = {};

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = false;

  last_control_output_.diagnostics.mass_estimator  = false;
  last_control_output_.diagnostics.mass_difference = 0;

  last_control_output_.diagnostics.disturbance_estimator = false;

  last_control_output_.diagnostics.disturbance_bx_b = 0;
  last_control_output_.diagnostics.disturbance_by_b = 0;

  last_control_output_.diagnostics.disturbance_bx_w = 0;
  last_control_output_.diagnostics.disturbance_by_w = 0;

  last_control_output_.diagnostics.disturbance_wx_w = 0;
  last_control_output_.diagnostics.disturbance_wy_w = 0;

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3Controller";
}

//}

/* PIDVelocityOutput() //{ */

void Se3Controller::PIDVelocityOutput(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command,
                                      const common::CONTROL_OUTPUT& control_output, const double& dt) {

  if (!tracker_command.use_position_vertical || !tracker_command.use_position_horizontal || !tracker_command.use_heading) {
    ROS_ERROR("[Se3Controller]: the tracker did not provide position+hdg reference");
    return;
  }

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  auto gains       = mrs_lib::get_mutexed(mutex_gains_, gains_);

  Eigen::Vector3d pos_ref = Eigen::Vector3d(tracker_command.position.x, tracker_command.position.y, tracker_command.position.z);
  Eigen::Vector3d pos     = Eigen::Vector3d(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);

  double hdg_ref = tracker_command.heading;
  double hdg     = getHeadingSafely(uav_state, tracker_command);

  // | ------------------ velocity feedforward ------------------ |

  Eigen::Vector3d vel_ff(0, 0, 0);

  if (tracker_command.use_velocity_horizontal && tracker_command.use_velocity_vertical) {
    vel_ff = Eigen::Vector3d(tracker_command.velocity.x, tracker_command.velocity.y, tracker_command.velocity.z);
  }

  // | -------------------------- gains ------------------------- |

  Eigen::Vector3d Kp;

  {
    std::scoped_lock lock(mutex_gains_);

    Kp << gains.kpxy, gains.kpxy, gains.kpz;
  }

  // | --------------------- control errors --------------------- |

  Eigen::Vector3d Ep = pos_ref - pos;

  // | --------------------------- pid -------------------------- |

  position_pid_x_.setSaturation(constraints.horizontal_speed);
  position_pid_y_.setSaturation(constraints.horizontal_speed);
  position_pid_z_.setSaturation(std::min(constraints.vertical_ascending_speed, constraints.vertical_descending_speed));

  double des_vel_x = position_pid_x_.update(Ep(0), dt);
  double des_vel_y = position_pid_y_.update(Ep(1), dt);
  double des_vel_z = position_pid_z_.update(Ep(2), dt);

  // | -------------------- position feedback ------------------- |

  Eigen::Vector3d des_vel = Eigen::Vector3d(des_vel_x, des_vel_y, des_vel_z) + vel_ff;

  if (control_output == common::VELOCITY_HDG) {

    // | --------------------- fill the output -------------------- |

    mrs_msgs::HwApiVelocityHdgCmd cmd;

    cmd.header.frame_id = uav_state.header.frame_id;
    cmd.header.stamp    = ros::Time::now();

    cmd.velocity.x = des_vel(0);
    cmd.velocity.y = des_vel(1);
    cmd.velocity.z = des_vel(2);

    cmd.heading = tracker_command.heading;

    last_control_output_.control_output = cmd;

  } else if (control_output == common::VELOCITY_HDG_RATE) {

    position_pid_heading_.setSaturation(constraints.heading_speed);

    double hdg_err = mrs_lib::geometry::sradians::diff(hdg_ref, hdg);

    double des_hdg_rate = position_pid_heading_.update(hdg_err, dt);

    // | --------------------------- ff --------------------------- |

    double des_hdg_ff = 0;

    if (tracker_command.use_heading_rate) {
      des_hdg_ff = tracker_command.heading_rate;
    }

    // | --------------------- fill the output -------------------- |

    mrs_msgs::HwApiVelocityHdgRateCmd cmd;

    cmd.header.frame_id = uav_state.header.frame_id;
    cmd.header.stamp    = ros::Time::now();

    cmd.velocity.x = des_vel(0);
    cmd.velocity.y = des_vel(1);
    cmd.velocity.z = des_vel(2);

    cmd.heading_rate = des_hdg_rate + des_hdg_ff;

    last_control_output_.control_output = cmd;
  } else {

    ROS_ERROR("[Se3Controller]: the required output of the position PID is not supported");
    return;
  }

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_heading_rate          = {};

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = false;

  last_control_output_.diagnostics.mass_estimator  = false;
  last_control_output_.diagnostics.mass_difference = 0;

  last_control_output_.diagnostics.disturbance_estimator = false;

  last_control_output_.diagnostics.disturbance_bx_b = 0;
  last_control_output_.diagnostics.disturbance_by_b = 0;

  last_control_output_.diagnostics.disturbance_bx_w = 0;
  last_control_output_.diagnostics.disturbance_by_w = 0;

  last_control_output_.diagnostics.disturbance_wx_w = 0;
  last_control_output_.diagnostics.disturbance_wy_w = 0;

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3Controller";
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |


/* //{ callbackDrs() */

void Se3Controller::callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[Se3Controller]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGains() //{ */

void Se3Controller::timerGains(const ros::TimerEvent& event) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerGains", _gain_filtering_rate_, 1.0, event);
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("Se3Controller::timerGains", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);
  auto gains      = mrs_lib::get_mutexed(mutex_gains_, gains_);

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains_ || mute_gains_by_tracker_);
  double gain_coeff    = (mute_gains_ || mute_gains_by_tracker_) ? _gain_mute_coefficient_ : 1.0;

  mute_gains_ = false;

  double dt = (event.current_real - event.last_real).toSec();

  if (!std::isfinite(dt) || (dt <= 0) || (dt > 5 * (1.0 / _gain_filtering_rate_))) {
    return;
  }

  bool updated = false;

  gains.kpxy          = calculateGainChange(dt, gains.kpxy, drs_params.kpxy * gain_coeff, bypass_filter, "kpxy", updated);
  gains.kvxy          = calculateGainChange(dt, gains.kvxy, drs_params.kvxy * gain_coeff, bypass_filter, "kvxy", updated);
  gains.kaxy          = calculateGainChange(dt, gains.kaxy, drs_params.kaxy * gain_coeff, bypass_filter, "kaxy", updated);
  gains.kiwxy         = calculateGainChange(dt, gains.kiwxy, drs_params.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
  gains.kibxy         = calculateGainChange(dt, gains.kibxy, drs_params.kibxy * gain_coeff, bypass_filter, "kibxy", updated);
  gains.kpz           = calculateGainChange(dt, gains.kpz, drs_params.kpz * gain_coeff, bypass_filter, "kpz", updated);
  gains.kvz           = calculateGainChange(dt, gains.kvz, drs_params.kvz * gain_coeff, bypass_filter, "kvz", updated);
  gains.kaz           = calculateGainChange(dt, gains.kaz, drs_params.kaz * gain_coeff, bypass_filter, "kaz", updated);
  gains.kq_roll_pitch = calculateGainChange(dt, gains.kq_roll_pitch, drs_params.kq_roll_pitch * gain_coeff, bypass_filter, "kq_roll_pitch", updated);
  gains.kq_yaw        = calculateGainChange(dt, gains.kq_yaw, drs_params.kq_yaw * gain_coeff, bypass_filter, "kq_yaw", updated);
  gains.km            = calculateGainChange(dt, gains.km, drs_params.km * gain_coeff, bypass_filter, "km", updated);

  // do not apply muting on these gains
  gains.kiwxy_lim = calculateGainChange(dt, gains.kiwxy_lim, drs_params.kiwxy_lim, false, "kiwxy_lim", updated);
  gains.kibxy_lim = calculateGainChange(dt, gains.kibxy_lim, drs_params.kibxy_lim, false, "kibxy_lim", updated);
  gains.km_lim    = calculateGainChange(dt, gains.km_lim, drs_params.km_lim, false, "km_lim", updated);

  mrs_lib::set_mutexed(mutex_gains_, gains, gains_);

  // set the gains back to dynamic reconfigure
  // and only do it when some filtering occurs
  if (updated) {

    drs_params.kpxy          = gains.kpxy;
    drs_params.kvxy          = gains.kvxy;
    drs_params.kaxy          = gains.kaxy;
    drs_params.kiwxy         = gains.kiwxy;
    drs_params.kibxy         = gains.kibxy;
    drs_params.kpz           = gains.kpz;
    drs_params.kvz           = gains.kvz;
    drs_params.kaz           = gains.kaz;
    drs_params.kq_roll_pitch = gains.kq_roll_pitch;
    drs_params.kq_yaw        = gains.kq_yaw;
    drs_params.kiwxy_lim     = gains.kiwxy_lim;
    drs_params.kibxy_lim     = gains.kibxy_lim;
    drs_params.km            = gains.km;
    drs_params.km_lim        = gains.km_lim;

    drs_->updateConfig(drs_params);

    ROS_INFO_THROTTLE(10.0, "[Se3Controller]: gains have been updated");
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

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

    if (std::abs(current_value) < 1e-6) {
      change *= gains_filter_max_change;
    } else {

      saturated_change = change;

      change_in_perc = ((current_value + saturated_change) / current_value) - 1.0;

      if (change_in_perc > gains_filter_max_change) {
        saturated_change = current_value * gains_filter_max_change;
      } else if (change_in_perc < -gains_filter_max_change) {
        saturated_change = current_value * -gains_filter_max_change;
      }

      if (std::abs(saturated_change) < std::abs(change) * gains_filter_min_change) {
        change *= gains_filter_min_change;
      } else {
        change = saturated_change;
      }
    }
  }

  if (std::abs(change) > 1e-3) {
    ROS_DEBUG("[Se3Controller]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

/* getHeadingSafely() //{ */

double Se3Controller::getHeadingSafely(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  try {
    return mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
  }

  try {
    return mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
  }
  catch (...) {
  }

  if (tracker_command.use_heading) {
    return tracker_command.heading;
  }

  return 0;
}

//}

}  // namespace se3_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::se3_controller::Se3Controller, mrs_uav_managers::Controller)

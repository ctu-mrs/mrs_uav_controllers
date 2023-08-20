/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <common.h>
#include <pid.hpp>

#include <mrs_uav_managers/controller.h>

#include <mpc_controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_controllers/mpc_controllerConfig.h>

#include <std_srvs/SetBool.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
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

namespace mpc_controller
{

/* //{ class MpcController */

class MpcController : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState &uav_state, const std::optional<mrs_msgs::TrackerCommand> &tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::string name_;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                            mutex_drs_;
  typedef mrs_uav_controllers::mpc_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>  Drs_t;
  boost::shared_ptr<Drs_t>                          drs_;
  void                                              callbackDrs(mrs_uav_controllers::mpc_controllerConfig &config, uint32_t level);
  DrsConfig_t                                       drs_params_;

  // | ----------------------- controllers ---------------------- |

  void positionPassthrough(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command);

  void PIDVelocityOutput(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command, const common::CONTROL_OUTPUT &control_output,
                         const double &dt);

  void MPC(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command, const double &dt,
           const common::CONTROL_OUTPUT &output_modality);

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | -------- throttle generation and mass estimation --------- |

  double _uav_mass_;
  double uav_mass_difference_;

  // | ------------------- configurable gains ------------------- |

  // gains that are used and already filtered
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kq_rp_;      // pitch/roll attitude gain
  double kq_y_;       // yaw attitude gain
  double kw_rp_;      // attitude rate gain
  double kw_y_;       // attitude rate gain

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  // | --------------------- gain filtering --------------------- |

  void filterGains(const bool mute_gains, const double dt);

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool &updated);

  double getHeadingSafely(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ----------------------- gain muting ---------------------- |

  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;

  // | ------------ controller limits and saturations ----------- |

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;

  double _throttle_saturation_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | ----------------- integral terms enabler ----------------- |

  ros::ServiceServer service_set_integral_terms_;
  bool               callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool               integral_terms_enabled_ = true;

  // | --------------------- MPC controller --------------------- |

  // number of states
  int _n_states_;

  // time steps
  double _dt1_;  // the first time step
  double _dt2_;  // all the other steps

  // the last control input
  double mpc_solver_x_u_ = 0;
  double mpc_solver_y_u_ = 0;
  double mpc_solver_z_u_ = 0;

  int _horizon_length_;

  // constraints
  double _max_speed_horizontal_, _max_acceleration_horizontal_, _max_jerk_;
  double _max_speed_vertical_, _max_acceleration_vertical_, _max_u_vertical_;

  // Q and S matrix diagonals for horizontal
  std::vector<double> _mat_Q_, _mat_S_;

  // Q and S matrix diagonals for vertical
  std::vector<double> _mat_Q_z_, _mat_S_z_;

  // MPC solver handlers
  std::shared_ptr<mrs_mpc_solvers::mpc_controller::Solver> mpc_solver_x_;
  std::shared_ptr<mrs_mpc_solvers::mpc_controller::Solver> mpc_solver_y_;
  std::shared_ptr<mrs_mpc_solvers::mpc_controller::Solver> mpc_solver_z_;

  // MPC solver params
  bool _mpc_solver_verbose_ = false;
  int  _mpc_solver_max_iterations_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler;
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

bool MpcController::initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                               std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers_->getMass();
  name_      = private_handlers_->runtime_name;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  bool success = true;

  success *= private_handlers->loadConfigFile(ros::package::getPath("mrs_uav_controllers") + "/config/private/mpc_controller.yaml");
  success *= private_handlers->loadConfigFile(ros::package::getPath("mrs_uav_controllers") + "/config/public/mpc_controller.yaml");
  success *= private_handlers->loadConfigFile(ros::package::getPath("mrs_uav_controllers") + "/config/private/" + private_handlers->name_space + ".yaml");
  success *= private_handlers->loadConfigFile(ros::package::getPath("mrs_uav_controllers") + "/config/public/" + private_handlers->name_space + ".yaml");

  if (!success) {
    return false;
  }

  mrs_lib::ParamLoader param_loader_parent(common_handlers->parent_nh, "ControlManager");

  param_loader_parent.loadParam("enable_profiler", _profiler_enabled_);

  if (!param_loader_parent.loadedSuccessfully()) {
    ROS_ERROR("[MpcController]: Could not load all parameters!");
    return false;
  }

  mrs_lib::ParamLoader param_loader(nh_, "MpcController");

  const std::string yaml_namespace = "mrs_uav_controllers/" + private_handlers_->name_space + "/";

  // load the dynamicall model parameters
  param_loader.loadParam(yaml_namespace + "mpc/mpc_model/number_of_states", _n_states_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_model/dt1", _dt1_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_model/dt2", _dt2_);

  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/horizon_length", _horizon_length_);

  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/horizontal/max_speed", _max_speed_horizontal_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/horizontal/max_acceleration", _max_acceleration_horizontal_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/horizontal/max_jerk", _max_jerk_);

  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/horizontal/Q", _mat_Q_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/horizontal/S", _mat_S_);

  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/vertical/max_speed", _max_speed_vertical_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/vertical/max_acceleration", _max_acceleration_vertical_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/vertical/max_u", _max_u_vertical_);

  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/vertical/Q", _mat_Q_z_);
  param_loader.loadParam(yaml_namespace + "mpc/mpc_parameters/vertical/S", _mat_S_z_);

  param_loader.loadParam(yaml_namespace + "mpc/solver/verbose", _mpc_solver_verbose_);
  param_loader.loadParam(yaml_namespace + "mpc/solver/max_iterations", _mpc_solver_max_iterations_);

  // | ------------------------- rampup ------------------------- |

  param_loader.loadParam(yaml_namespace + "so3/rampup/enabled", _rampup_enabled_);
  param_loader.loadParam(yaml_namespace + "so3/rampup/speed", _rampup_speed_);

  // | --------------------- integral gains --------------------- |

  param_loader.loadParam(yaml_namespace + "so3/gains/integral_gains/kiw", kiwxy_);
  param_loader.loadParam(yaml_namespace + "so3/gains/integral_gains/kib", kibxy_);

  // integrator limits
  param_loader.loadParam(yaml_namespace + "so3/gains/integral_gains/kiw_lim", kiwxy_lim_);
  param_loader.loadParam(yaml_namespace + "so3/gains/integral_gains/kib_lim", kibxy_lim_);

  // | ------------- height and attitude controller ------------- |

  // attitude gains
  param_loader.loadParam(yaml_namespace + "so3/gains/attitude/roll_pitch", kq_rp_);
  param_loader.loadParam(yaml_namespace + "so3/gains/attitude/yaw", kq_y_);

  // attitude rate gains
  param_loader.loadParam(yaml_namespace + "so3/gains/attitude_rate/roll_pitch", kw_rp_);
  param_loader.loadParam(yaml_namespace + "so3/gains/attitude_rate/yaw", kw_y_);

  // mass estimator
  param_loader.loadParam(yaml_namespace + "so3/mass_estimator/km", km_);
  param_loader.loadParam(yaml_namespace + "so3/mass_estimator/km_lim", km_lim_);

  // constraints
  param_loader.loadParam(yaml_namespace + "so3/constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam(yaml_namespace + "so3/constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);

  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[%s]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low", name_.c_str());
    return false;
  }

  param_loader.loadParam(yaml_namespace + "so3/constraints/throttle_saturation", _throttle_saturation_);

  // gain filtering
  param_loader.loadParam(yaml_namespace + "so3/gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.loadParam(yaml_namespace + "so3/gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // gain muting
  param_loader.loadParam(yaml_namespace + "so3/gain_mute_coefficient", _gain_mute_coefficient_);

  // angular rate feed forward
  param_loader.loadParam(yaml_namespace + "so3/angular_rate_feedforward/jerk", drs_params_.jerk_feedforward);

  // output mode
  param_loader.loadParam(yaml_namespace + "so3/preferred_output", drs_params_.preferred_output_mode);

  // | ------------------- position pid params ------------------ |

  param_loader.loadParam(yaml_namespace + "position_controller/translation_gains/p", _pos_pid_p_);
  param_loader.loadParam(yaml_namespace + "position_controller/translation_gains/i", _pos_pid_i_);
  param_loader.loadParam(yaml_namespace + "position_controller/translation_gains/d", _pos_pid_d_);

  param_loader.loadParam(yaml_namespace + "position_controller/heading_gains/p", _hdg_pid_p_);
  param_loader.loadParam(yaml_namespace + "position_controller/heading_gains/i", _hdg_pid_i_);
  param_loader.loadParam(yaml_namespace + "position_controller/heading_gains/d", _hdg_pid_d_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", this->name_.c_str());
    return false;
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(drs_params_.preferred_output_mode == OUTPUT_ACTUATORS || drs_params_.preferred_output_mode == OUTPUT_CONTROL_GROUP ||
        drs_params_.preferred_output_mode == OUTPUT_ATTITUDE_RATE || drs_params_.preferred_output_mode == OUTPUT_ATTITUDE)) {
    ROS_ERROR("[%s]: preferred output mode has to be {0, 1, 2, 3}!", this->name_.c_str());
    return false;
  }

  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | ----------------- prepare the MPC solver ----------------- |

  mpc_solver_x_ = std::make_shared<mrs_mpc_solvers::mpc_controller::Solver>(name_, _mpc_solver_verbose_, _mpc_solver_max_iterations_, _mat_Q_, _mat_S_, _dt1_,
                                                                            _dt2_, 0, 1.0);
  mpc_solver_y_ = std::make_shared<mrs_mpc_solvers::mpc_controller::Solver>(name_, _mpc_solver_verbose_, _mpc_solver_max_iterations_, _mat_Q_, _mat_S_, _dt1_,
                                                                            _dt2_, 0, 1.0);
  mpc_solver_z_ = std::make_shared<mrs_mpc_solvers::mpc_controller::Solver>(name_, _mpc_solver_verbose_, _mpc_solver_max_iterations_, _mat_Q_z_, _mat_S_z_,
                                                                            _dt1_, _dt2_, 0.5, 0.5);

  // | --------------- dynamic reconfigure server --------------- |

  drs_params_.kiwxy     = kiwxy_;
  drs_params_.kibxy     = kibxy_;
  drs_params_.kqxy      = kq_rp_;
  drs_params_.kqz       = kq_y_;
  drs_params_.km        = km_;
  drs_params_.km_lim    = km_lim_;
  drs_params_.kiwxy_lim = kiwxy_lim_;
  drs_params_.kibxy_lim = kibxy_lim_;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MpcController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- service servers -------------------- |

  service_set_integral_terms_ = nh_.advertiseService("set_integral_terms_in", &MpcController::callbackSetIntegralTerms, this);

  // | ---------------------- position pid ---------------------- |

  position_pid_x_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_y_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_z_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_heading_.setParams(_hdg_pid_p_, _hdg_pid_d_, _hdg_pid_i_, -1, 0.1);

  // | ------------------------ profiler ------------------------ |

  profiler = mrs_lib::Profiler(common_handlers->parent_nh, "MpcController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized", this->name_.c_str());

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool MpcController::activate(const ControlOutput &last_control_output) {

  activation_control_output_ = last_control_output;

  double activation_mass = _uav_mass_;

  if (activation_control_output_.diagnostics.mass_estimator) {
    uav_mass_difference_ = activation_control_output_.diagnostics.mass_difference;
    activation_mass += uav_mass_difference_;
    ROS_INFO("[%s]: setting mass difference from the last control output: %.2f kg", this->name_.c_str(), uav_mass_difference_);
  }

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  if (activation_control_output_.diagnostics.disturbance_estimator) {
    Ib_b_[0] = -activation_control_output_.diagnostics.disturbance_bx_b;
    Ib_b_[1] = -activation_control_output_.diagnostics.disturbance_by_b;

    Iw_w_[0] = -activation_control_output_.diagnostics.disturbance_wx_w;
    Iw_w_[1] = -activation_control_output_.diagnostics.disturbance_wy_w;

    ROS_INFO(
        "[%s]: setting disturbances from the last control output: Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, %.2f N",
        this->name_.c_str(), Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);
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

    ROS_INFO("[%s]: activating rampup with initial throttle: %.4f, target: %.4f", name_.c_str(), throttle_last_controller.value(), hover_throttle);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_throttle_   = throttle_last_controller.value();

    rampup_duration_ = fabs(throttle_difference) / _rampup_speed_;
  }

  first_iteration_ = true;
  gains_muted_     = true;

  ROS_INFO("[%s]: activated", this->name_.c_str());

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void MpcController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[%s]: deactivated", this->name_.c_str());
}

//}

/* updateInactive() //{ */

void MpcController::updateInactive(const mrs_msgs::UavState &uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateWhenAcctive() */

MpcController::ControlOutput MpcController::updateActive(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler.createRoutine("update");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MpcController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
  }

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[%s]: the last odometry message came too close (%.2f s)!", name_.c_str(), dt);

    dt = 0.01;
  }

  // | ----------- obtain the lowest possible modality ---------- |

  auto lowest_modality = common::getLowestOuput(common_handlers_->control_output_modalities);

  if (!lowest_modality) {

    ROS_ERROR_THROTTLE(1.0, "[%s]: output modalities are empty! This error should never appear.", name_.c_str());

    return last_control_output_;
  }

  // | ----- we might prefer some output mode over the other ---- |

  if (drs_params.preferred_output_mode == OUTPUT_ATTITUDE_RATE && common_handlers_->control_output_modalities.attitude_rate) {
    ROS_DEBUG_THROTTLE(1.0, "[%s]: prioritizing attitude rate output", name_.c_str());
    lowest_modality = common::ATTITUDE_RATE;
  } else if (drs_params.preferred_output_mode == OUTPUT_ATTITUDE && common_handlers_->control_output_modalities.attitude) {
    ROS_DEBUG_THROTTLE(1.0, "[%s]: prioritizing attitude output", name_.c_str());
    lowest_modality = common::ATTITUDE;
  } else if (drs_params.preferred_output_mode == OUTPUT_CONTROL_GROUP && common_handlers_->control_output_modalities.control_group) {
    ROS_DEBUG_THROTTLE(1.0, "[%s]: prioritizing control group output", name_.c_str());
    lowest_modality = common::CONTROL_GROUP;
  } else if (drs_params.preferred_output_mode == OUTPUT_ACTUATORS && common_handlers_->control_output_modalities.actuators) {
    ROS_DEBUG_THROTTLE(1.0, "[%s]: prioritizing actuators output", name_.c_str());
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
      MPC(uav_state, tracker_command, dt, common::ACCELERATION_HDG);
      break;
    }

    case common::ACCELERATION_HDG_RATE: {
      MPC(uav_state, tracker_command, dt, common::ACCELERATION_HDG_RATE);
      break;
    }

    case common::ATTITUDE: {
      MPC(uav_state, tracker_command, dt, common::ATTITUDE);
      break;
    }

    case common::ATTITUDE_RATE: {
      MPC(uav_state, tracker_command, dt, common::ATTITUDE_RATE);
      break;
    }

    case common::CONTROL_GROUP: {
      MPC(uav_state, tracker_command, dt, common::CONTROL_GROUP);
      break;
    }

    case common::ACTUATORS_CMD: {
      MPC(uav_state, tracker_command, dt, common::ACTUATORS_CMD);
      break;
    }

    default: {
      break;
    }
  }

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus MpcController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MpcController::switchOdometrySource(const mrs_msgs::UavState &new_uav_state) {

  ROS_INFO("[%s]: switching the odometry source", this->name_.c_str());

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

    ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform world integral to the new frame", this->name_.c_str());

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = 0;
    Iw_w_[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void MpcController::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w_ = Eigen::Vector2d::Zero(2);
  Ib_b_ = Eigen::Vector2d::Zero(2);
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr MpcController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[%s]: updating constraints", this->name_.c_str());

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                         controllers                        |
// --------------------------------------------------------------

/* Mpc() //{ */

void MpcController::MPC(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command, const double &dt,
                        const common::CONTROL_OUTPUT &output_modality) {

  auto drs_params  = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

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

  Rp << tracker_command.position.x, tracker_command.position.y, tracker_command.position.z;  // fill the desired position
  Rv << tracker_command.velocity.x, tracker_command.velocity.y, tracker_command.velocity.z;

  // | ------ store the estimated values from the uav state ----- |

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  Eigen::Vector3d Ov(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);

  // R - current uav attitude
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);

  // --------------------------------------------------------------
  // |                     MPC lateral control                    |
  // --------------------------------------------------------------

  // | --------------- calculate the control erros -------------- |

  Eigen::Vector3d Ep = Rp - Op;
  Eigen::Vector3d Ev = Rv - Ov;

  // | ------------------- initial conditions ------------------- |

  Eigen::MatrixXd initial_x = Eigen::MatrixXd::Zero(3, 1);
  Eigen::MatrixXd initial_y = Eigen::MatrixXd::Zero(3, 1);
  Eigen::MatrixXd initial_z = Eigen::MatrixXd::Zero(3, 1);

  /* initial x //{ */

  {
    double acceleration;
    double velocity;
    double coef = 1.5;

    if (fabs(uav_state.acceleration.linear.x) < coef * _max_acceleration_horizontal_) {
      acceleration = uav_state.acceleration.linear.x;
    } else {
      acceleration = tracker_command.acceleration.x;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry x acceleration exceeds constraints (%.2f > %.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.acceleration.linear.x), coef, _max_acceleration_horizontal_);
    }

    if (fabs(uav_state.velocity.linear.x) < coef * _max_speed_horizontal_) {
      velocity = uav_state.velocity.linear.x;
    } else {
      velocity = tracker_command.velocity.x;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry x velocity exceeds constraints (%.2f > %0.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.velocity.linear.x), coef, _max_speed_horizontal_);
    }

    initial_x << uav_state.pose.position.x, velocity, acceleration;
  }

  //}

  /* initial y //{ */

  {
    double acceleration;
    double velocity;
    double coef = 1.5;

    if (fabs(uav_state.acceleration.linear.y) < coef * _max_acceleration_horizontal_) {
      acceleration = uav_state.acceleration.linear.y;
    } else {
      acceleration = tracker_command.acceleration.y;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry y acceleration exceeds constraints (%.2f > %.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.acceleration.linear.y), coef, _max_acceleration_horizontal_);
    }

    if (fabs(uav_state.velocity.linear.y) < coef * _max_speed_horizontal_) {
      velocity = uav_state.velocity.linear.y;
    } else {
      velocity = tracker_command.velocity.y;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry y velocity exceeds constraints (%.2f > %0.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.velocity.linear.y), coef, _max_speed_horizontal_);
    }

    initial_y << uav_state.pose.position.y, velocity, acceleration;
  }

  //}

  /* initial z //{ */

  {
    double acceleration;
    double velocity;
    double coef = 1.5;

    if (fabs(uav_state.acceleration.linear.z) < coef * _max_acceleration_horizontal_) {
      acceleration = uav_state.acceleration.linear.z;
    } else {
      acceleration = tracker_command.acceleration.z;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry z acceleration exceeds constraints (%.2f > %.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.acceleration.linear.z), coef, _max_acceleration_horizontal_);
    }

    if (fabs(uav_state.velocity.linear.z) < coef * _max_speed_vertical_) {
      velocity = uav_state.velocity.linear.z;
    } else {
      velocity = tracker_command.velocity.z;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry z velocity exceeds constraints (%.2f > %0.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.velocity.linear.z), coef, _max_speed_vertical_);
    }

    initial_z << uav_state.pose.position.z, velocity, acceleration;
  }

  //}

  // | ---------------------- set reference --------------------- |

  Eigen::MatrixXd mpc_reference_x = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);
  Eigen::MatrixXd mpc_reference_y = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);
  Eigen::MatrixXd mpc_reference_z = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);

  // prepare the full reference vector
  for (int i = 0; i < _horizon_length_; i++) {

    mpc_reference_x((i * _n_states_) + 0, 0) = tracker_command.position.x;
    mpc_reference_y((i * _n_states_) + 0, 0) = tracker_command.position.y;
    mpc_reference_z((i * _n_states_) + 0, 0) = tracker_command.position.z;
  }

  // | ------------------ set the penalizations ----------------- |

  std::vector<double> temp_Q_horizontal = _mat_Q_;
  std::vector<double> temp_Q_vertical   = _mat_Q_z_;

  std::vector<double> temp_S_horizontal = _mat_S_;
  std::vector<double> temp_S_vertical   = _mat_S_z_;

  if (!tracker_command.use_position_horizontal) {
    temp_Q_horizontal[0] = 0;
    temp_S_horizontal[0] = 0;
  }

  if (!tracker_command.use_velocity_horizontal) {
    temp_Q_horizontal[1] = 0;
    temp_S_horizontal[1] = 0;
  }

  if (!tracker_command.use_position_vertical) {
    temp_Q_vertical[0] = 0;
    temp_S_vertical[0] = 0;
  }

  if (!tracker_command.use_velocity_vertical) {
    temp_Q_vertical[1] = 0;
    temp_S_vertical[1] = 0;
  }

  // | ------------------------ optimize ------------------------ |

  mpc_solver_x_->setQ(temp_Q_horizontal);
  mpc_solver_x_->setS(temp_S_horizontal);
  mpc_solver_x_->setParams();
  mpc_solver_x_->setLastInput(mpc_solver_x_u_);
  mpc_solver_x_->loadReference(mpc_reference_x);
  mpc_solver_x_->setLimits(_max_speed_horizontal_, 999, _max_acceleration_horizontal_, _max_jerk_, _dt1_, _dt2_);
  mpc_solver_x_->setInitialState(initial_x);
  [[maybe_unused]] int iters_x = mpc_solver_x_->solveMPC();
  mpc_solver_x_u_              = mpc_solver_x_->getFirstControlInput();

  mpc_solver_y_->setQ(temp_Q_horizontal);
  mpc_solver_y_->setS(temp_S_horizontal);
  mpc_solver_y_->setParams();
  mpc_solver_y_->setLastInput(mpc_solver_y_u_);
  mpc_solver_y_->loadReference(mpc_reference_y);
  mpc_solver_y_->setLimits(_max_speed_horizontal_, 999, _max_acceleration_horizontal_, _max_jerk_, _dt1_, _dt2_);
  mpc_solver_y_->setInitialState(initial_y);
  [[maybe_unused]] int iters_y = mpc_solver_y_->solveMPC();
  mpc_solver_y_u_              = mpc_solver_y_->getFirstControlInput();

  mpc_solver_z_->setQ(temp_Q_vertical);
  mpc_solver_z_->setS(temp_S_vertical);
  mpc_solver_z_->setParams();
  mpc_solver_z_->setLastInput(mpc_solver_z_u_);
  mpc_solver_z_->loadReference(mpc_reference_z);
  mpc_solver_z_->setLimits(_max_speed_vertical_, _max_acceleration_vertical_, _max_u_vertical_, 999.0, _dt1_, _dt2_);
  mpc_solver_z_->setInitialState(initial_z);
  [[maybe_unused]] int iters_z = mpc_solver_z_->solveMPC();
  mpc_solver_z_u_              = mpc_solver_z_->getFirstControlInput();

  // | ----------- disable lateral feedback if needed ----------- |

  if (tracker_command.disable_position_gains) {
    mpc_solver_x_u_ = 0;
    mpc_solver_y_u_ = 0;
  }

  // | --------------------- load the gains --------------------- |

  filterGains(tracker_command.disable_position_gains, dt);

  Eigen::Array3d Kw(0, 0, 0);
  Eigen::Array3d Kq;

  {
    std::scoped_lock lock(mutex_gains_);

    Kq << kq_rp_, kq_rp_, kq_y_;

    Kw[0] = kw_rp_;
    Kw[1] = kw_rp_;
    Kw[2] = kw_y_;
  }

  // | ---------- desired orientation matrix and force ---------- |

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
      ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform the Ib_b_ to the world frame", name_.c_str());
    }
  }

  // construct the desired force vector

  if (tracker_command.use_acceleration) {
    Ra << tracker_command.acceleration.x + mpc_solver_x_u_, tracker_command.acceleration.y + mpc_solver_y_u_, tracker_command.acceleration.z + mpc_solver_z_u_;
  } else {
    Ra << mpc_solver_x_u_, mpc_solver_y_u_, mpc_solver_z_u_;
  }

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);

  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
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

    // antiwindup
    double temp_gain = kiwxy_;
    if (!tracker_command.disable_antiwindups) {
      if (rampup_active_ || sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2)) > 0.3) {
        temp_gain = 0;
        ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for world integral kicks in", this->name_.c_str());
      }
    }

    if (integral_terms_enabled_) {
      if (tracker_command.use_position_horizontal) {
        Iw_w_ += temp_gain * Ep.head(2) * dt;
      } else if (tracker_command.use_velocity_horizontal) {
        Iw_w_ += temp_gain * Ev.head(2) * dt;
      }
    }

    // saturate the world X
    bool world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!", this->name_.c_str());
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: MPC's world X integral is being saturated!", this->name_.c_str());
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!", this->name_.c_str());
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: MPC's world Y integral is being saturated!", this->name_.c_str());
    }
  }

  //}

  /* body error integrator //{ */

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
        ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform the position error to fcu_untilted", name_.c_str());
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
        ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform the velocity error to fcu_untilted", name_.c_str());
      }
    }

    // integrate the body error

    // antiwindup
    double temp_gain = kibxy_;
    if (!tracker_command.disable_antiwindups) {
      if (rampup_active_ || sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2)) > 0.3) {
        temp_gain = 0;
        ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for body integral kicks in", this->name_.c_str());
      }
    }

    if (integral_terms_enabled_) {
      if (tracker_command.use_position_horizontal) {
        Ib_b_ += temp_gain * Ep_fcu_untilted * dt;
      } else if (tracker_command.use_velocity_horizontal) {
        Ib_b_ += temp_gain * Ev_fcu_untilted * dt;
      }
    }

    // saturate the body X
    bool body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!", this->name_.c_str());
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: MPC's body pitch integral is being saturated!", this->name_.c_str());
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!", this->name_.c_str());
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: MPC's body roll integral is being saturated!", this->name_.c_str());
    }
  }

  //}

  Eigen::Vector3d des_acc = integral_feedback / total_mass + Ra;

  if (output_modality == common::ACCELERATION_HDG || output_modality == common::ACCELERATION_HDG_RATE) {

    if (output_modality == common::ACCELERATION_HDG) {

      mrs_msgs::HwApiAccelerationHdgCmd cmd;

      cmd.acceleration.x = des_acc[0];
      cmd.acceleration.y = des_acc[1];
      cmd.acceleration.z = des_acc[2];

      cmd.heading = tracker_command.heading;

      last_control_output_.control_output = cmd;

    } else {

      double des_hdg_ff = 0;

      if (tracker_command.use_heading_rate) {
        des_hdg_ff = tracker_command.heading_rate;
      }

      mrs_msgs::HwApiAccelerationHdgRateCmd cmd;

      cmd.acceleration.x = des_acc[0];
      cmd.acceleration.y = des_acc[1];
      cmd.acceleration.z = des_acc[2];

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

      geometry_msgs::Vector3Stamped world_accel;

      world_accel.header.stamp    = ros::Time::now();
      world_accel.header.frame_id = uav_state.header.frame_id;
      world_accel.vector.x        = Ra[0];
      world_accel.vector.y        = Ra[1];
      world_accel.vector.z        = Ra[2];

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

    last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_[0];
    last_control_output_.diagnostics.disturbance_by_b = -Ib_b_[1];

    last_control_output_.diagnostics.disturbance_bx_w = -Ib_w[0];
    last_control_output_.diagnostics.disturbance_by_w = -Ib_w[1];

    last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_[0];
    last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_[1];

    last_control_output_.diagnostics.controller_enforcing_constraints = true;

    last_control_output_.diagnostics.horizontal_speed_constraint = 0.5 * _max_speed_horizontal_;
    last_control_output_.diagnostics.horizontal_acc_constraint   = 0.5 * _max_acceleration_horizontal_;

    last_control_output_.diagnostics.vertical_asc_speed_constraint = 0.5 * _max_speed_vertical_;
    last_control_output_.diagnostics.vertical_asc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

    last_control_output_.diagnostics.vertical_desc_speed_constraint = 0.5 * _max_speed_vertical_;
    last_control_output_.diagnostics.vertical_desc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

    last_control_output_.diagnostics.controller = name_;

    return;
  }

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    // antiwindup
    double temp_gain = km_;
    if (rampup_active_ ||
        (fabs(uav_state.velocity.linear.z) > 0.3 && ((Ep[2] > 0 && uav_state.velocity.linear.z > 0) || (Ep[2] < 0 && uav_state.velocity.linear.z < 0)))) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for the mass kicks in", this->name_.c_str());
    }

    if (tracker_command.use_position_vertical) {
      uav_mass_difference_ += temp_gain * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!", this->name_.c_str());
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: The UAV mass difference is being saturated to %.2f!", this->name_.c_str(), uav_mass_difference_);
    }
  }

  //}

  Eigen::Vector3d f = integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[%s]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", this->name_.c_str(), f[2]);

    f << 0, 0, 1;
  }

  // | ------------------- sanitize tilt angle ------------------ |

  double tilt_safety_limit = _tilt_angle_failsafe_enabled_ ? _tilt_angle_failsafe_ : std::numeric_limits<double>::max();

  auto f_normed_sanitized = common::sanitizeDesiredForce(f.normalized(), tilt_safety_limit, constraints.tilt, "MpcController");

  if (!f_normed_sanitized) {

    ROS_INFO("[%s]: f = [%.2f, %.2f, %.2f]", this->name_.c_str(), f[0], f[1], f[2]);
    ROS_INFO("[%s]: integral feedback: [%.2f, %.2f, %.2f]", this->name_.c_str(), integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[%s]: feed forward: [%.2f, %.2f, %.2f]", this->name_.c_str(), feed_forward[0], feed_forward[1], feed_forward[2]);
    ROS_INFO("[%s]: tracker_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", this->name_.c_str(), tracker_command.position.x, tracker_command.position.y,
             tracker_command.position.z, tracker_command.heading);
    ROS_INFO("[%s]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", this->name_.c_str(), uav_state.pose.position.x, uav_state.pose.position.y,
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
        ROS_WARN_THROTTLE(1.0, "[%s]: failed to add heading to the desired orientation matrix", this->name_.c_str());
      }
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (tracker_command.use_heading) {
      bxd << cos(tracker_command.heading), sin(tracker_command.heading), 0;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[%s]: desired heading was not specified, using current heading instead!", this->name_.c_str());
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    Rd = common::so3transform(f_normed, bxd, false);
  }

  // | -------------------- desired throttle -------------------- |

  double desired_thrust_force = f.dot(R.col(2));
  double throttle             = 0;

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_ = false;

      ROS_INFO("[%s]: rampup finished", name_.c_str());

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_throttle_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      throttle = rampup_throttle_;

      ROS_INFO_THROTTLE(0.1, "[%s]: ramping up throttle, %.4f", name_.c_str(), throttle);
    }

  } else {

    if (desired_thrust_force >= 0) {
      throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, desired_thrust_force);
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: just so you know, the desired throttle force is negative (%.2f)", name_.c_str(), desired_thrust_force);
    }
  }

  // | ------------------- throttle saturation ------------------ |

  bool throttle_saturated = false;

  if (!std::isfinite(throttle)) {

    ROS_ERROR("[%s]: NaN detected in variable 'throttle'!!!", name_.c_str());
    return;

  } else if (throttle > _throttle_saturation_) {
    throttle = _throttle_saturation_;
    ROS_WARN_THROTTLE(0.1, "[%s]: saturating throttle to %.2f", name_.c_str(), _throttle_saturation_);
  } else if (throttle < 0.0) {
    throttle = 0.0;
    ROS_WARN_THROTTLE(0.1, "[%s]: saturating throttle to 0.0", name_.c_str());
  }

  if (throttle_saturated) {
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", name_.c_str());
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", name_.c_str(), tracker_command.position.x,
                      tracker_command.position.y, tracker_command.position.z, tracker_command.heading);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", name_.c_str(), tracker_command.velocity.x,
                      tracker_command.velocity.y, tracker_command.velocity.z, tracker_command.heading_rate);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", name_.c_str(), tracker_command.acceleration.x,
                      tracker_command.acceleration.y, tracker_command.acceleration.z, tracker_command.heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", name_.c_str(), tracker_command.jerk.x, tracker_command.jerk.y,
                      tracker_command.jerk.z, tracker_command.heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", name_.c_str());
    ROS_WARN_THROTTLE(0.1, "[%s]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", name_.c_str(), uav_state.pose.position.x,
                      uav_state.pose.position.y, uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[%s]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", name_.c_str(), uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", name_.c_str());
  }

  // | -------------- unbiased desired acceleration ------------- |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Vector3d thrust_vector = desired_thrust_force * Rd.col(2);

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);

    // TODO change to z from IMU?
    double world_accel_z = tracker_command.acceleration.z;

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

  // | --------------- fill the resulting command --------------- |

  // fill the desired orientation for the tilt error check
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(Rd);

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(desired_x_accel, desired_y_accel, desired_z_accel);

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = rampup_active_;

  last_control_output_.diagnostics.mass_estimator  = true;
  last_control_output_.diagnostics.mass_difference = uav_mass_difference_;
  last_control_output_.diagnostics.total_mass      = total_mass;

  last_control_output_.diagnostics.disturbance_estimator = true;

  last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_[0];
  last_control_output_.diagnostics.disturbance_by_b = -Ib_b_[1];

  last_control_output_.diagnostics.disturbance_bx_w = -Ib_w[0];
  last_control_output_.diagnostics.disturbance_by_w = -Ib_w[1];

  last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_[0];
  last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_[1];

  last_control_output_.diagnostics.controller_enforcing_constraints = true;

  last_control_output_.diagnostics.horizontal_speed_constraint = 0.5 * _max_speed_horizontal_;
  last_control_output_.diagnostics.horizontal_acc_constraint   = 0.5 * _max_acceleration_horizontal_;

  last_control_output_.diagnostics.vertical_asc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_asc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.vertical_desc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_desc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.controller = name_;

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
      ROS_ERROR("[%s]: exception caught while calculating the desired_yaw_rate feedforward", name_.c_str());
    }

    rate_feedforward << 0, 0, desired_yaw_rate;
  }

  // | ------------ jerk feedforward -> angular rate ------------ |

  Eigen::Vector3d jerk_feedforward = Eigen::Vector3d(0, 0, 0);

  if (tracker_command.use_jerk && drs_params.jerk_feedforward) {

    ROS_DEBUG_THROTTLE(1.0, "[%s]: using jerk feedforward", name_.c_str());

    Eigen::Matrix3d I;
    I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    Eigen::Vector3d desired_jerk = Eigen::Vector3d(tracker_command.jerk.x, tracker_command.jerk.y, tracker_command.jerk.z);
    jerk_feedforward             = (I.transpose() * Rd.transpose() * desired_jerk) / (desired_thrust_force / total_mass);
  }

  // | --------------- run the attitude controller -------------- |

  Eigen::Vector3d attitude_rate_saturation(constraints.roll_rate, constraints.pitch_rate, constraints.yaw_rate);

  auto attitude_rate_command = common::attitudeController(uav_state, attitude_cmd, jerk_feedforward + rate_feedforward, attitude_rate_saturation, Kq, false);

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

void MpcController::positionPassthrough(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command) {

  if (!tracker_command.use_position_vertical || !tracker_command.use_position_horizontal || !tracker_command.use_heading) {
    ROS_ERROR("[%s]: the tracker did not provide position+hdg reference", name_.c_str());
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

  last_control_output_.diagnostics.controller_enforcing_constraints = true;

  last_control_output_.diagnostics.horizontal_speed_constraint = 0.5 * _max_speed_horizontal_;
  last_control_output_.diagnostics.horizontal_acc_constraint   = 0.5 * _max_acceleration_horizontal_;

  last_control_output_.diagnostics.vertical_asc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_asc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.vertical_desc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_desc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.controller = name_;
}

//}

/* PIDVelocityOutput() //{ */

void MpcController::PIDVelocityOutput(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command,
                                      const common::CONTROL_OUTPUT &control_output, const double &dt) {

  if (!tracker_command.use_position_vertical || !tracker_command.use_position_horizontal || !tracker_command.use_heading) {
    ROS_ERROR("[%s]: the tracker did not provide position+hdg reference", name_.c_str());
    return;
  }

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  Eigen::Vector3d pos_ref = Eigen::Vector3d(tracker_command.position.x, tracker_command.position.y, tracker_command.position.z);
  Eigen::Vector3d pos     = Eigen::Vector3d(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);

  double hdg_ref = tracker_command.heading;
  double hdg     = getHeadingSafely(uav_state, tracker_command);

  // | ------------------ velocity feedforward ------------------ |

  Eigen::Vector3d vel_ff(0, 0, 0);

  if (tracker_command.use_velocity_horizontal && tracker_command.use_velocity_vertical) {
    vel_ff = Eigen::Vector3d(tracker_command.velocity.x, tracker_command.velocity.y, tracker_command.velocity.z);
  }

  // | --------------------- control errors --------------------- |

  Eigen::Vector3d Ep = pos_ref - pos;

  // | --------------------------- pid -------------------------- |

  position_pid_x_.setSaturation(constraints.horizontal_speed);
  position_pid_y_.setSaturation(constraints.horizontal_speed);
  position_pid_z_.setSaturation(std::min(constraints.vertical_ascending_speed, constraints.vertical_descending_speed));

  double des_vel_x = position_pid_x_.update(Ep[0], dt);
  double des_vel_y = position_pid_y_.update(Ep[1], dt);
  double des_vel_z = position_pid_z_.update(Ep[2], dt);

  // | -------------------- position feedback ------------------- |

  Eigen::Vector3d des_vel = Eigen::Vector3d(des_vel_x, des_vel_y, des_vel_z) + vel_ff;

  if (control_output == common::VELOCITY_HDG) {

    // | --------------------- fill the output -------------------- |

    mrs_msgs::HwApiVelocityHdgCmd cmd;

    cmd.header.frame_id = uav_state.header.frame_id;
    cmd.header.stamp    = ros::Time::now();

    cmd.velocity.x = des_vel[0];
    cmd.velocity.y = des_vel[1];
    cmd.velocity.z = des_vel[2];

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

    cmd.velocity.x = des_vel[0];
    cmd.velocity.y = des_vel[1];
    cmd.velocity.z = des_vel[2];

    cmd.heading_rate = des_hdg_rate + des_hdg_ff;

    last_control_output_.control_output = cmd;
  } else {

    ROS_ERROR("[%s]: the required output of the position PID is not supported", name_.c_str());
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

  last_control_output_.diagnostics.controller_enforcing_constraints = true;

  last_control_output_.diagnostics.horizontal_speed_constraint = 0.5 * _max_speed_horizontal_;
  last_control_output_.diagnostics.horizontal_acc_constraint   = 0.5 * _max_acceleration_horizontal_;

  last_control_output_.diagnostics.vertical_asc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_asc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.vertical_desc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_desc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.controller = name_;
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void MpcController::callbackDrs(mrs_uav_controllers::mpc_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[%s]: DRS updated gains", this->name_.c_str());
}

//}

/* //{ callbackSetIntegralTerms() */

bool MpcController::callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized_)
    return false;

  integral_terms_enabled_ = req.data;

  std::stringstream ss;

  ss << "integral terms %s" << (integral_terms_enabled_ ? "enabled" : "disabled");

  ROS_INFO_STREAM_THROTTLE(1.0, "[" << name_.c_str() << "]: " << ss.str());

  res.message = ss.str();
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void MpcController::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kq_rp_ = calculateGainChange(dt, kq_rp_, drs_params_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kq_y_  = calculateGainChange(dt, kq_y_, drs_params_.kqz * gain_coeff, bypass_filter, "kqz", updated);
    km_    = calculateGainChange(dt, km_, drs_params_.km * gain_coeff, bypass_filter, "km", updated);
    kiwxy_ = calculateGainChange(dt, kiwxy_, drs_params_.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
    kibxy_ = calculateGainChange(dt, kibxy_, drs_params_.kibxy * gain_coeff, bypass_filter, "kibxy", updated);

    km_lim_    = calculateGainChange(dt, km_lim_, drs_params_.km_lim, false, "km_lim", updated);
    kiwxy_lim_ = calculateGainChange(dt, kiwxy_lim_, drs_params_.kiwxy_lim, false, "kiwxy_lim", updated);
    kibxy_lim_ = calculateGainChange(dt, kibxy_lim_, drs_params_.kibxy_lim, false, "kibxy_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_params_ = drs_params_;

      new_drs_params_.kiwxy     = kiwxy_;
      new_drs_params_.kibxy     = kibxy_;
      new_drs_params_.kqxy      = kq_rp_;
      new_drs_params_.kqz       = kq_y_;
      new_drs_params_.km        = km_;
      new_drs_params_.km_lim    = km_lim_;
      new_drs_params_.kiwxy_lim = kiwxy_lim_;
      new_drs_params_.kibxy_lim = kibxy_lim_;

      drs_->updateConfig(new_drs_params_);
    }
  }
}

//}

/* calculateGainChange() //{ */

double MpcController::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name,
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
    ROS_INFO_THROTTLE(1.0, "[%s]: changing gain '%s' from %.2f to %.2f", name_.c_str(), name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

/* getHeadingSafely() //{ */

double MpcController::getHeadingSafely(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command) {

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

//}

}  // namespace mpc_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::mpc_controller::MpcController, mrs_uav_managers::Controller)

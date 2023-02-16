#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>

#include <common.h>

#include <mrs_uav_managers/controller.h>

#include <mpc_controller_solver.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_controllers/mpc_controllerConfig.h>

#include <std_srvs/SetBool.h>

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

namespace mpc_controller
{

/* //{ class MpcController */

class MpcController : public mrs_uav_managers::Controller {

public:
  ~MpcController(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  ControlOutput update(const mrs_msgs::UavState &uav_state, const std::optional<mrs_msgs::TrackerCommand> &tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::string name_;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

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

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  bool                          got_constraints_ = false;

  // | -------- throttle generation and mass estimation --------- |

  double _uav_mass_;
  double uav_mass_difference_;
  double hover_throttle_;

  // | ------------------- configurable gains ------------------- |

  // gains that are used and already filtered
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kqxy_;       // pitch/roll attitude gain
  double kqz_;        // yaw attitude gain

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

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;

  double _throttle_saturation_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

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
  std::unique_ptr<mrs_mpc_solvers::mpc_controller::Solver> mpc_solver_x_;
  std::unique_ptr<mrs_mpc_solvers::mpc_controller::Solver> mpc_solver_y_;
  std::unique_ptr<mrs_mpc_solvers::mpc_controller::Solver> mpc_solver_z_;

  // MPC solver params
  bool _mpc_solver_verbose_ = false;
  int  _mpc_solver_max_iterations_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler;
  bool              profiler_enabled_ = false;

  // | ----------------------- output mode ---------------------- |

  int        _output_mode_;  // attitude_rate / acceleration
  std::mutex mutex_output_mode_;

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

void MpcController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                               std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = common_handlers->getMass();

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MpcController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[%s]: the version of the binary (%s) does not match the config file (%s), please build me!", name_.c_str(), VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", profiler_enabled_);

  // load the dynamicall model parameters
  param_loader.loadParam("mpc_model/number_of_states", _n_states_);
  param_loader.loadParam("mpc_model/dt1", _dt1_);
  param_loader.loadParam("mpc_model/dt2", _dt2_);

  param_loader.loadParam("mpc_parameters/horizon_length", _horizon_length_);

  param_loader.loadParam("mpc_parameters/horizontal/max_speed", _max_speed_horizontal_);
  param_loader.loadParam("mpc_parameters/horizontal/max_acceleration", _max_acceleration_horizontal_);
  param_loader.loadParam("mpc_parameters/horizontal/max_jerk", _max_jerk_);

  param_loader.loadParam("mpc_parameters/horizontal/Q", _mat_Q_);
  param_loader.loadParam("mpc_parameters/horizontal/S", _mat_S_);

  param_loader.loadParam("mpc_parameters/vertical/max_speed", _max_speed_vertical_);
  param_loader.loadParam("mpc_parameters/vertical/max_acceleration", _max_acceleration_vertical_);
  param_loader.loadParam("mpc_parameters/vertical/max_u", _max_u_vertical_);

  param_loader.loadParam("mpc_parameters/vertical/Q", _mat_Q_z_);
  param_loader.loadParam("mpc_parameters/vertical/S", _mat_S_z_);

  param_loader.loadParam("mpc_solver/verbose", _mpc_solver_verbose_);
  param_loader.loadParam("mpc_solver/max_iterations", _mpc_solver_max_iterations_);

  // | ------------------------- rampup ------------------------- |

  param_loader.loadParam("rampup/enabled", _rampup_enabled_);
  param_loader.loadParam("rampup/speed", _rampup_speed_);

  // | --------------------- integral gains --------------------- |

  param_loader.loadParam("integral_gains/kiw", kiwxy_);
  param_loader.loadParam("integral_gains/kib", kibxy_);

  // integrator limits
  param_loader.loadParam("integral_gains/kiw_lim", kiwxy_lim_);
  param_loader.loadParam("integral_gains/kib_lim", kibxy_lim_);

  // | ------------- height and attitude controller ------------- |

  // attitude gains
  param_loader.loadParam("attitude_feedback/default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.loadParam("attitude_feedback/default_gains/vertical/attitude/kq", kqz_);

  // mass estimator
  param_loader.loadParam("mass_estimator/km", km_);
  param_loader.loadParam("mass_estimator/km_lim", km_lim_);

  // constraints
  param_loader.loadParam("constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam("constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);
  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[MpcController]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    ros::shutdown();
  }

  param_loader.loadParam("constraints/throttle_saturation", _throttle_saturation_);

  // gain filtering
  param_loader.loadParam("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.loadParam("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // gain muting
  param_loader.loadParam("gain_mute_coefficient", _gain_mute_coefficient_);

  // output mode
  param_loader.loadParam("output_mode", _output_mode_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", this->name_.c_str());
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(_output_mode_ == OUTPUT_ATTITUDE_RATE || _output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[%s]: output mode has to be {0, 1}!", this->name_.c_str());
    ros::shutdown();
  }

  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | ----------------- prepare the MPC solver ----------------- |

  mpc_solver_x_ = std::make_unique<mrs_mpc_solvers::mpc_controller::Solver>(
      mrs_mpc_solvers::mpc_controller::Solver(name_, _mpc_solver_verbose_, _mpc_solver_max_iterations_, _mat_Q_, _mat_S_, _dt1_, _dt2_, 0, 1.0));
  mpc_solver_y_ = std::make_unique<mrs_mpc_solvers::mpc_controller::Solver>(
      mrs_mpc_solvers::mpc_controller::Solver(name_, _mpc_solver_verbose_, _mpc_solver_max_iterations_, _mat_Q_, _mat_S_, _dt1_, _dt2_, 0, 1.0));
  mpc_solver_z_ = std::make_unique<mrs_mpc_solvers::mpc_controller::Solver>(
      mrs_mpc_solvers::mpc_controller::Solver(name_, _mpc_solver_verbose_, _mpc_solver_max_iterations_, _mat_Q_z_, _mat_S_z_, _dt1_, _dt2_, 0.5, 0.5));

  // | --------------- dynamic reconfigure server --------------- |

  drs_params_.kiwxy     = kiwxy_;
  drs_params_.kibxy     = kibxy_;
  drs_params_.kqxy      = kqxy_;
  drs_params_.kqz       = kqz_;
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

  // | ------------------------ profiler ------------------------ |

  profiler = mrs_lib::Profiler(nh_, "MpcController", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized, version %s", this->name_.c_str(), VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool MpcController::activate(const ControlOutput &last_control_output) {


  activation_control_output_ = last_control_output;

  if (last_control_output.diagnostics.mass_estimator) {
    uav_mass_difference_ = last_control_output.diagnostics.mass_difference;
    ROS_INFO("[%s]: setting mass difference from the last control output: %.2f kg", this->name_.c_str(), uav_mass_difference_);
  }

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  if (last_control_output.diagnostics.disturbance_estimator) {
    Ib_b_[0] = -last_control_output_.diagnostics.disturbance_bx_b;
    Ib_b_[1] = -last_control_output_.diagnostics.disturbance_by_b;

    Iw_w_[0] = -last_control_output_.diagnostics.disturbance_wx_w;
    Iw_w_[1] = -last_control_output_.diagnostics.disturbance_wy_w;

    ROS_INFO(
        "[%s]: setting disturbances from the last control output: Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, %.2f N",
        this->name_.c_str(), Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);
  }

  // did the last controller use manual throttle control?
  auto throttle_last_controller = common::extractThrottle(last_control_output);

  // rampup check
  if (_rampup_enabled_ && throttle_last_controller) {

    double hover_throttle =
        mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, last_control_output.diagnostics.total_mass * common_handlers_->g);
    double throttle_difference = hover_throttle - throttle_last_controller.value();

    if (throttle_difference > 0) {
      rampup_direction_ = 1;
    } else if (throttle_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[MpcController]: activating rampup with initial throttle: %.4f, target: %.4f", throttle_last_controller.value(), hover_throttle);

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

/* //{ update() */

MpcController::ControlOutput MpcController::update(const mrs_msgs::UavState &uav_state, const std::optional<mrs_msgs::TrackerCommand> &tracker_command) {

  mrs_lib::Routine    profiler_routine = profiler.createRoutine("update");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MpcController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;
  }

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

    ROS_INFO("[MpcController]: first iteration");

    return {activation_control_output_};

  } else {

    dt                = (uav_state.header.stamp - last_update_time_).toSec();
    last_update_time_ = uav_state.header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[%s]: the last odometry message came too close (%.2f s)!", this->name_.c_str(), dt);

    if (last_control_output_.control_output) {
      return last_control_output_;
    } else {
      return {activation_control_output_};
    }
  }

  // | ----------------- get the current heading ---------------- |

  double uav_heading = 0;

  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: could not calculate the UAV heading", name_.c_str());
  }

  if (tracker_command->disable_antiwindups) {  // TODO why is this here?
    ROS_INFO_THROTTLE(1.0, "[%s]: antiwindups disabled by tracker", name_.c_str());
  }

  // | -------------- clean the last control output ------------- |

  last_control_output_.control_output = {};

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

  Rp << tracker_command->position.x, tracker_command->position.y, tracker_command->position.z;  // fill the desired position
  Rv << tracker_command->velocity.x, tracker_command->velocity.y, tracker_command->velocity.z;

  if (tracker_command->use_heading_rate) {

    // to fill in the desired yaw rate (as the last degree of freedom), we need the desired orientation and the current desired roll and pitch rate
    double desired_yaw_rate = 0;
    try {
      desired_yaw_rate = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYawRateIntrinsic(tracker_command->heading_rate);
    }
    catch (...) {
      ROS_ERROR("[%s]: exception caught while calculating the desired_yaw_rate feedforward", name_.c_str());
    }

    Rw << 0, 0, desired_yaw_rate;
  }

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

  Eigen::Vector3d Ep = Op - Rp;
  Eigen::Vector3d Ev = Ov - Rv;

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
      acceleration = tracker_command->acceleration.x;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry x acceleration exceeds constraints (%.2f > %.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.acceleration.linear.x), coef, _max_acceleration_horizontal_);
    }

    if (fabs(uav_state.velocity.linear.x) < coef * _max_speed_horizontal_) {
      velocity = uav_state.velocity.linear.x;
    } else {
      velocity = tracker_command->velocity.x;

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
      acceleration = tracker_command->acceleration.y;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry y acceleration exceeds constraints (%.2f > %.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.acceleration.linear.y), coef, _max_acceleration_horizontal_);
    }

    if (fabs(uav_state.velocity.linear.y) < coef * _max_speed_horizontal_) {
      velocity = uav_state.velocity.linear.y;
    } else {
      velocity = tracker_command->velocity.y;

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
      acceleration = tracker_command->acceleration.z;

      ROS_ERROR_THROTTLE(1.0, "[%s]: odometry z acceleration exceeds constraints (%.2f > %.1f * %.2f m), using reference for initial condition", name_.c_str(),
                         fabs(uav_state.acceleration.linear.z), coef, _max_acceleration_horizontal_);
    }

    if (fabs(uav_state.velocity.linear.z) < coef * _max_speed_vertical_) {
      velocity = uav_state.velocity.linear.z;
    } else {
      velocity = tracker_command->velocity.z;

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

    mpc_reference_x((i * _n_states_) + 0, 0) = tracker_command->position.x;
    mpc_reference_y((i * _n_states_) + 0, 0) = tracker_command->position.y;
    mpc_reference_z((i * _n_states_) + 0, 0) = tracker_command->position.z;
  }

  // | ------------------ set the penalizations ----------------- |

  std::vector<double> temp_Q_horizontal = _mat_Q_;
  std::vector<double> temp_Q_vertical   = _mat_Q_z_;

  std::vector<double> temp_S_horizontal = _mat_S_;
  std::vector<double> temp_S_vertical   = _mat_S_z_;

  if (!tracker_command->use_position_horizontal) {
    temp_Q_horizontal[0] = 0;
    temp_S_horizontal[0] = 0;
  }

  if (!tracker_command->use_velocity_horizontal) {
    temp_Q_horizontal[1] = 0;
    temp_S_horizontal[1] = 0;
  }

  if (!tracker_command->use_position_vertical) {
    temp_Q_vertical[0] = 0;
    temp_S_vertical[0] = 0;
  }

  if (!tracker_command->use_velocity_vertical) {
    temp_Q_vertical[1] = 0;
    temp_S_vertical[1] = 0;
  }

  // | ------------------------ optimize ------------------------ |

  mpc_solver_x_->lock();
  mpc_solver_x_->setQ(temp_Q_horizontal);
  mpc_solver_x_->setS(temp_S_horizontal);
  mpc_solver_x_->setParams();
  mpc_solver_x_->setLastInput(mpc_solver_x_u_);
  mpc_solver_x_->loadReference(mpc_reference_x);
  mpc_solver_x_->setLimits(_max_speed_horizontal_, 999, _max_acceleration_horizontal_, _max_jerk_, _dt1_, _dt2_);
  mpc_solver_x_->setInitialState(initial_x);
  [[maybe_unused]] int iters_x = mpc_solver_x_->solveMPC();
  mpc_solver_x_u_              = mpc_solver_x_->getFirstControlInput();
  mpc_solver_x_->unlock();

  mpc_solver_y_->lock();
  mpc_solver_y_->setQ(temp_Q_horizontal);
  mpc_solver_y_->setS(temp_S_horizontal);
  mpc_solver_y_->setParams();
  mpc_solver_y_->setLastInput(mpc_solver_y_u_);
  mpc_solver_y_->loadReference(mpc_reference_y);
  mpc_solver_y_->setLimits(_max_speed_horizontal_, 999, _max_acceleration_horizontal_, _max_jerk_, _dt1_, _dt2_);
  mpc_solver_y_->setInitialState(initial_y);
  [[maybe_unused]] int iters_y = mpc_solver_y_->solveMPC();
  mpc_solver_y_u_              = mpc_solver_y_->getFirstControlInput();
  mpc_solver_y_->unlock();

  mpc_solver_z_->lock();
  mpc_solver_z_->setQ(temp_Q_vertical);
  mpc_solver_z_->setS(temp_S_vertical);
  mpc_solver_z_->setParams();
  mpc_solver_z_->setLastInput(mpc_solver_z_u_);
  mpc_solver_z_->loadReference(mpc_reference_z);
  mpc_solver_z_->setLimits(_max_speed_vertical_, _max_acceleration_vertical_, _max_u_vertical_, 999.0, _dt1_, _dt2_);
  mpc_solver_z_->setInitialState(initial_z);
  [[maybe_unused]] int iters_z = mpc_solver_z_->solveMPC();
  mpc_solver_z_u_              = mpc_solver_z_->getFirstControlInput();
  mpc_solver_z_->unlock();

  // | ----------- disable lateral feedback if needed ----------- |

  if (tracker_command->disable_position_gains) {
    mpc_solver_x_u_ = 0;
    mpc_solver_y_u_ = 0;
  }

  // | --------------------- load the gains --------------------- |

  filterGains(tracker_command->disable_position_gains, dt);

  Eigen::Vector3d Ka;
  Eigen::Array3d  Kq;

  {
    std::scoped_lock lock(mutex_gains_);

    Kq << kqxy_, kqxy_, kqz_;
  }

  // | -------------- recalculate the hover throttle -------------- |

  hover_throttle_ =
      mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, (_uav_mass_ + uav_mass_difference_) * common_handlers_->g);

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

  if (tracker_command->use_acceleration) {
    Ra << tracker_command->acceleration.x + mpc_solver_x_u_, tracker_command->acceleration.y + mpc_solver_y_u_,
        tracker_command->acceleration.z + mpc_solver_z_u_;
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

  Eigen::Vector3d f = integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[%s]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", this->name_.c_str(), f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[%s]: NaN detected in variable 'theta', returning empty command", this->name_.c_str());

    return last_control_output_;
  }

  if (_tilt_angle_failsafe_enabled_ && theta > _tilt_angle_failsafe_) {

    ROS_ERROR("[%s]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", this->name_.c_str(),
              (180.0 / M_PI) * theta, (180.0 / M_PI) * _tilt_angle_failsafe_);
    ROS_INFO("[%s]: f = [%.2f, %.2f, %.2f]", this->name_.c_str(), f[0], f[1], f[2]);
    ROS_INFO("[%s]: integral feedback: [%.2f, %.2f, %.2f]", this->name_.c_str(), integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[%s]: feed forward: [%.2f, %.2f, %.2f]", this->name_.c_str(), feed_forward[0], feed_forward[1], feed_forward[2]);
    ROS_INFO("[%s]: tracker_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", this->name_.c_str(), tracker_command->position.x, tracker_command->position.y,
             tracker_command->position.z, tracker_command->heading);
    ROS_INFO("[%s]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", this->name_.c_str(), uav_state.pose.position.x, uav_state.pose.position.y,
             uav_state.pose.position.z, uav_heading);

    return last_control_output_;
  }

  // saturate the angle

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  if (fabs(constraints.tilt) > 1e-3 && theta > constraints.tilt) {
    ROS_WARN_THROTTLE(1.0, "[%s]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", this->name_.c_str(), (theta / M_PI) * 180.0,
                      (constraints.tilt / M_PI) * 180.0);
    theta = constraints.tilt;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

  // | ------------- construct the rotational matrix ------------ |

  Eigen::Matrix3d Rd;

  if (tracker_command->use_orientation) {

    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(tracker_command->orientation);

    if (tracker_command->use_heading) {
      try {
        Rd = mrs_lib::AttitudeConverter(Rd).setHeading(tracker_command->heading);
      }
      catch (...) {
        ROS_WARN_THROTTLE(1.0, "[%s]: failed to add heading to the desired orientation matrix", this->name_.c_str());
      }
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (tracker_command->use_heading) {
      bxd << cos(tracker_command->heading), sin(tracker_command->heading), 0;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[%s]: desired heading was not specified, using current heading instead!", this->name_.c_str());
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    Rd = common::so3transform(f_norm, bxd, false);
  }

  // | -------------------- orientation error ------------------- |

  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;

  // clang-format off
  Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
  // clang-format on

  // | ------------------- angular rate error ------------------- |

  double throttle_force = f.dot(R.col(2));
  double throttle       = 0;

  if (throttle_force >= 0) {
    throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, throttle_force);
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: just so you know, the desired throttle force is negative (%.2f)", this->name_.c_str(), throttle_force);
  }

  // saturate the throttle
  if (!std::isfinite(throttle)) {

    throttle = 0;
    ROS_ERROR("[%s]: NaN detected in variable 'throttle', setting it to 0 and returning!!!", this->name_.c_str());

  } else if (throttle > _throttle_saturation_) {

    throttle = _throttle_saturation_;
    ROS_WARN_THROTTLE(1.0, "[%s]: saturating throttle to %.2f", this->name_.c_str(), _throttle_saturation_);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", this->name_.c_str());
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->position.x,
                      tracker_command->position.y, tracker_command->position.z, tracker_command->heading);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->velocity.x,
                      tracker_command->velocity.y, tracker_command->velocity.z, tracker_command->heading_rate);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->acceleration.x,
                      tracker_command->acceleration.y, tracker_command->acceleration.z, tracker_command->heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->jerk.x,
                      tracker_command->jerk.y, tracker_command->jerk.z, tracker_command->heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", this->name_.c_str());
    ROS_WARN_THROTTLE(0.1, "[%s]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), uav_state.pose.position.x,
                      uav_state.pose.position.y, uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[%s]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", this->name_.c_str(), uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", this->name_.c_str());

  } else if (throttle < 0.0) {

    throttle = 0.0;
    ROS_WARN_THROTTLE(1.0, "[%s]: saturating throttle to %.2f", this->name_.c_str(), 0.0);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", this->name_.c_str());
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->position.x,
                      tracker_command->position.y, tracker_command->position.z, tracker_command->heading);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->velocity.x,
                      tracker_command->velocity.y, tracker_command->velocity.z, tracker_command->heading_rate);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->acceleration.x,
                      tracker_command->acceleration.y, tracker_command->acceleration.z, tracker_command->heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[%s]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), tracker_command->jerk.x,
                      tracker_command->jerk.y, tracker_command->jerk.z, tracker_command->heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", this->name_.c_str());
    ROS_WARN_THROTTLE(0.1, "[%s]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", this->name_.c_str(), uav_state.pose.position.x,
                      uav_state.pose.position.y, uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[%s]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", this->name_.c_str(), uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[%s]: ---------------------------", this->name_.c_str());
  }

  // prepare the attitude feedback
  Eigen::Vector3d q_feedback = -Kq * Eq.array();

  // feedforward angular acceleration
  Eigen::Vector3d q_feedforward = Eigen::Vector3d(0, 0, 0);

  Eigen::Matrix3d I;
  I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
  Eigen::Vector3d desired_jerk = Eigen::Vector3d(tracker_command->jerk.x, tracker_command->jerk.y, tracker_command->jerk.z);
  q_feedforward                = (I.transpose() * Rd.transpose() * desired_jerk) / (throttle_force / total_mass);

  // angular feedback + angular rate feedforward
  Eigen::Vector3d t = q_feedback + Rw + q_feedforward;

  // compensate for the parasitic heading rate created by the desired pitch and roll rate
  Eigen::Vector3d rp_heading_rate_compensation = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d q_feedback_yawless = t;
  q_feedback_yawless(2)              = 0;  // nullyfy the effect of the original yaw feedback

  double parasitic_heading_rate = 0;

  try {
    parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeadingRate(q_feedback_yawless);
  }
  catch (...) {
    ROS_ERROR("[%s]: exception caught while calculating the parasitic heading rate", name_.c_str());
  }

  try {
    rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
  }
  catch (...) {
    ROS_ERROR("[%s]: exception caught while calculating the parasitic heading rate compensation", name_.c_str());
  }

  t += rp_heading_rate_compensation;

  // --------------------------------------------------------------
  // |                 integrators and estimators                 |
  // --------------------------------------------------------------

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
    if (!tracker_command->disable_antiwindups) {
      if (rampup_active_ || sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2)) > 0.3) {
        temp_gain = 0;
        ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for body integral kicks in", this->name_.c_str());
      }
    }

    if (integral_terms_enabled_) {
      if (tracker_command->use_position_horizontal) {
        Ib_b_ -= temp_gain * Ep_fcu_untilted * dt;
      } else if (tracker_command->use_velocity_horizontal) {
        Ib_b_ -= temp_gain * Ev_fcu_untilted * dt;
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
    if (!tracker_command->disable_antiwindups) {
      if (rampup_active_ || sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2)) > 0.3) {
        temp_gain = 0;
        ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for world integral kicks in", this->name_.c_str());
      }
    }

    if (integral_terms_enabled_) {
      if (tracker_command->use_position_horizontal) {
        Iw_w_ -= temp_gain * Ep.head(2) * dt;
      } else if (tracker_command->use_velocity_horizontal) {
        Iw_w_ -= temp_gain * Ev.head(2) * dt;
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

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    // antiwindup
    double temp_gain = km_;
    if (rampup_active_ ||
        (fabs(uav_state.velocity.linear.z) > 0.3 && ((Ep[2] < 0 && uav_state.velocity.linear.z > 0) || (Ep[2] > 0 && uav_state.velocity.linear.z < 0)))) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for the mass kicks in", this->name_.c_str());
    }

    if (tracker_command->use_position_vertical) {
      uav_mass_difference_ -= temp_gain * Ep[2] * dt;
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

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

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
    ROS_WARN_THROTTLE(1.0, "[%s]: missing dynamics constraints", this->name_.c_str());
  }

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Matrix3d des_orientation = mrs_lib::AttitudeConverter(Rd);
    Eigen::Vector3d throttle_vector = throttle_force * des_orientation.col(2);

    double world_accel_x = (throttle_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (throttle_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
    double world_accel_z = tracker_command->acceleration.z;

    // You might thing this should be here. However, if you uncomment this, the landings are going to be very slow
    // because the the estimator will produce non-zero velocity even when sitting on the ground. This will trigger
    // anti-windup, which will stop mass estimation. Therefore, the touch-down will not be detected.
    /* double world_accel_z = (throttle_vector[2] / total_mass) - common_handlers_->g; */

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

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_ = false;

      ROS_INFO("[MpcController]: rampup finished");

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_throttle_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      throttle = rampup_throttle_;

      ROS_INFO_THROTTLE(0.1, "[MpcController]: ramping up throttle, %.4f", throttle);
    }
  }

  auto output_mode = common::getHighestOuput(common_handlers_->control_output_modalities);

  if (!output_mode) {

    ROS_ERROR_THROTTLE(1.0, "[MpcController]: output modalities are empty! This error should never appear.");
    last_control_output_.control_output = {};

    return last_control_output_;
  }

  if (output_mode.value() == common::ATTITUDE_RATE) {

    mrs_msgs::HwApiAttitudeRateCmd cmd;

    cmd.body_rate.x = t[0];
    cmd.body_rate.y = t[1];
    cmd.body_rate.z = t[2];

    cmd.throttle = throttle;

    last_control_output_.control_output = cmd;

  } else if (output_mode.value() == common::ATTITUDE) {

    mrs_msgs::HwApiAttitudeCmd cmd;

    cmd.orientation = mrs_lib::AttitudeConverter(Rd);

    cmd.throttle = throttle;

    last_control_output_.control_output = cmd;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[MpcController]: the controller does not support the required output modality");
  }

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

  // set the constraints
  last_control_output_.diagnostics.controller_enforcing_constraints = true;

  last_control_output_.diagnostics.horizontal_speed_constraint = 0.5 * _max_speed_horizontal_;
  last_control_output_.diagnostics.horizontal_acc_constraint   = 0.5 * _max_acceleration_horizontal_;

  last_control_output_.diagnostics.vertical_asc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_asc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.vertical_desc_speed_constraint = 0.5 * _max_speed_vertical_;
  last_control_output_.diagnostics.vertical_desc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  last_control_output_.diagnostics.controller = this->name_;

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

  got_constraints_ = true;

  ROS_INFO("[%s]: updating constraints", this->name_.c_str());

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

void MpcController::callbackDrs(mrs_uav_controllers::mpc_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_, mutex_output_mode_);

    drs_params_ = config;
  }

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

    kqxy_  = calculateGainChange(dt, kqxy_, drs_params_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kqz_   = calculateGainChange(dt, kqz_, drs_params_.kqz * gain_coeff, bypass_filter, "kqz", updated);
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
      new_drs_params_.kqxy      = kqxy_;
      new_drs_params_.kqz       = kqz_;
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

//}

}  // namespace mpc_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::mpc_controller::MpcController, mrs_uav_managers::Controller)

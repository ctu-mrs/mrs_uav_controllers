#define VERSION "0.0.5.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_uav_manager/Controller.h>

#include <mrs_controllers/mpc_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>
#include <mrs_lib/mutex.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <mrs_controllers/cvx_wrapper.h>

//}

#define X 0
#define Y 1
#define Z 2

#define OUTPUT_ATTITUDE_RATE 1
#define OUTPUT_ATTITUDE_QUATERNION 2

#define STRING_EQUAL 0

namespace mrs_controllers
{

namespace mpc_controller
{

/* //{ class MpcController */

class MpcController : public mrs_uav_manager::Controller {

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

  bool is_initialized = false;
  bool is_active      = false;

  std::string name_;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef mrs_controllers::mpc_controllerConfig    DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(mrs_controllers::mpc_controllerConfig &config, uint32_t level);
  DrsConfig_t                                      drs_gains_;

  // | ---------- thrust generation and mass estimation --------- |

  double                       _uav_mass_;
  double                       uav_mass_difference_;
  double                       _g_;
  mrs_uav_manager::MotorParams _motor_params_;
  double                       hover_thrust_;

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
  double kwxy_;       // pitch/roll attitude rate gain
  double kwz_;        // yaw attitude rate gain

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  // | --------------------- gain filtering --------------------- |

  ros::Timer timer_gain_filter_;
  void       timerGainsFilter(const ros::TimerEvent &event);

  double _gains_filter_timer_rate_;
  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  double _gains_filter_max_change_;  // calculated from change_rate/timer_rate;
  double _gains_filter_min_change_;  // calculated from change_rate/timer_rate;

  // | ----------------------- gain muting ---------------------- |

  bool   mute_lateral_gains_              = false;
  bool   mute_lateral_gains_after_toggle_ = false;
  double _mute_coefficitent_;

  // | ------------ controller limits and saturations ----------- |

  double _tilt_angle_saturation_;
  double _tilt_angle_failsafe_;
  double _thrust_saturation_;
  double _yaw_rate_saturation_;
  double _max_tilt_angle_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

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
  double cvx_x_u = 0;
  double cvx_y_u = 0;
  double cvx_z_u = 0;

  int _horizon_length_;

  // constraints
  double _max_speed_horizontal_, max_acceleration_horizontal_, _max_jerk_;
  double _max_speed_vertical_, _max_acceleration_vertical_, _max_u_vertical_;

  // Q and S matrix diagonals for horizontal
  std::vector<double> _Q_, _S_;

  // Q and S matrix diagonals for vertical
  std::vector<double> _Q_z_, _S_z_;

  // CVXGen handlers
  std::unique_ptr<mrs_controllers::cvx_wrapper::CvxWrapper> cvx_x_;
  std::unique_ptr<mrs_controllers::cvx_wrapper::CvxWrapper> cvx_y_;
  std::unique_ptr<mrs_controllers::cvx_wrapper::CvxWrapper> cvx_z_;

  // CVXGen params
  bool _cvx_verbose_ = false;
  int  _cvx_max_iterations_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler;
  bool              profiler_enabled_ = false;

  // | ------------------ activation and output ----------------- |

  int _output_mode_;  // 1 = ATTITUDE RATES, 2 = ATTITUDE QUATERNION

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

  // | ------------------------- rampup ------------------------- |

  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_thrust_;
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

void MpcController::initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                               const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  name_            = name;
  _motor_params_   = motor_params;
  _uav_mass_       = uav_mass;
  _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MpcController");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[MpcController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // load the dynamicall model parameters
  param_loader.load_param("mpc_model/number_of_states", _n_states_);
  param_loader.load_param("mpc_model/dt1", _dt1_);
  param_loader.load_param("mpc_model/dt2", _dt2_);

  param_loader.load_param("mpc_parameters/horizon_length", _horizon_length_);

  param_loader.load_param("mpc_parameters/horizontal/max_speed", _max_speed_horizontal_);
  param_loader.load_param("mpc_parameters/horizontal/max_acceleration", max_acceleration_horizontal_);
  param_loader.load_param("mpc_parameters/horizontal/max_jerk", _max_jerk_);

  param_loader.load_param("mpc_parameters/horizontal/Q", _Q_);
  param_loader.load_param("mpc_parameters/horizontal/S", _S_);

  param_loader.load_param("mpc_parameters/vertical/max_speed", _max_speed_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_acceleration", _max_acceleration_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_u", _max_u_vertical_);

  param_loader.load_param("mpc_parameters/vertical/Q", _Q_z_);
  param_loader.load_param("mpc_parameters/vertical/S", _S_z_);

  param_loader.load_param("cvx_parameters/verbose", _cvx_verbose_);
  param_loader.load_param("cvx_parameters/max_iterations", _cvx_max_iterations_);

  // | ------------------------- rampup ------------------------- |

  param_loader.load_param("rampup/enabled", _rampup_enabled_);
  param_loader.load_param("rampup/speed", _rampup_speed_);

  // | --------------------- integral gains --------------------- |

  param_loader.load_param("integral_gains/kiw", kiwxy_);
  param_loader.load_param("integral_gains/kib", kibxy_);

  // integrator limits
  param_loader.load_param("integral_gains/kiw_lim", kiwxy_lim_);
  param_loader.load_param("integral_gains/kib_lim", kibxy_lim_);

  // | ------------- height and attitude controller ------------- |

  // attitude gains
  param_loader.load_param("attitude_feedback/default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.load_param("attitude_feedback/default_gains/vertical/attitude/kq", kqz_);

  // attitude rate gains
  param_loader.load_param("attitude_feedback/default_gains/horizontal/attitude/kw", kwxy_);
  param_loader.load_param("attitude_feedback/default_gains/vertical/attitude/kw", kwz_);

  // mass estimator
  param_loader.load_param("attitude_feedback/default_gains/weight_estimator/km", km_);
  param_loader.load_param("attitude_feedback/default_gains/weight_estimator/km_lim", km_lim_);

  // constraints
  param_loader.load_param("attitude_feedback/tilt_angle_saturation", _tilt_angle_saturation_);
  param_loader.load_param("attitude_feedback/tilt_angle_failsafe", _tilt_angle_failsafe_);
  param_loader.load_param("attitude_feedback/thrust_saturation", _thrust_saturation_);
  param_loader.load_param("attitude_feedback/yaw_rate_saturation", _yaw_rate_saturation_);

  // gain filtering
  param_loader.load_param("attitude_feedback/gains_filter/filter_rate", _gains_filter_timer_rate_);
  param_loader.load_param("attitude_feedback/gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("attitude_feedback/gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // output mode
  param_loader.load_param("output_mode", _output_mode_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", this->name_.c_str());
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  // if yaw_rate_saturation is 0 (or close), set it to something very high, so its inactive
  if (_yaw_rate_saturation_ <= 1e-3) {
    _yaw_rate_saturation_ = 10e6;
  }

  _gains_filter_max_change_ = _gains_filter_change_rate_ / _gains_filter_timer_rate_;
  _gains_filter_min_change_ = _gains_filter_min_change_rate_ / _gains_filter_timer_rate_;

  if (!(_output_mode_ == OUTPUT_ATTITUDE_RATE || _output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[%s]: output mode has to be {1, 2}!", this->name_.c_str());
    ros::shutdown();
  }

  // convert to radians
  _max_tilt_angle_ = (_max_tilt_angle_ / 180) * M_PI;

  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // | ------------------- prepare the CVXGen ------------------- |

  cvx_x_ = std::make_unique<mrs_controllers::cvx_wrapper::CvxWrapper>(
      mrs_controllers::cvx_wrapper::CvxWrapper(_cvx_verbose_, _cvx_max_iterations_, _Q_, _S_, _dt1_, _dt2_, 0, 1.0));
  cvx_y_ = std::make_unique<mrs_controllers::cvx_wrapper::CvxWrapper>(
      mrs_controllers::cvx_wrapper::CvxWrapper(_cvx_verbose_, _cvx_max_iterations_, _Q_, _S_, _dt1_, _dt2_, 0, 1.0));
  cvx_z_ = std::make_unique<mrs_controllers::cvx_wrapper::CvxWrapper>(
      mrs_controllers::cvx_wrapper::CvxWrapper(_cvx_verbose_, _cvx_max_iterations_, _Q_z_, _S_z_, _dt1_, _dt2_, 0.5, 0.5));

  // | --------------- dynamic reconfigure server --------------- |

  drs_gains_.kiwxy     = kiwxy_;
  drs_gains_.kibxy     = kibxy_;
  drs_gains_.kqxy      = kqxy_;
  drs_gains_.kqz       = kqz_;
  drs_gains_.kwxy      = kwxy_;
  drs_gains_.kwz       = kwz_;
  drs_gains_.km        = km_;
  drs_gains_.km_lim    = km_lim_;
  drs_gains_.kiwxy_lim = kiwxy_lim_;
  drs_gains_.kibxy_lim = kibxy_lim_;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_gains_);
  Drs_t::CallbackType f = boost::bind(&MpcController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- service servers -------------------- |

  service_set_integral_terms_ = nh_.advertiseService("set_integral_terms_in", &MpcController::callbackSetIntegralTerms, this);

  // | ------------------------ profiler ------------------------ |

  profiler = mrs_lib::Profiler(nh_, "MpcController", profiler_enabled_);

  // | ------------------------- timers ------------------------- |

  timer_gain_filter_ = nh_.createTimer(ros::Rate(_gains_filter_timer_rate_), &MpcController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized, version %s", this->name_.c_str(), VERSION);

  is_initialized = true;
}

//}

/* //{ activate() */

bool MpcController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[%s]: activated without getting the last controllers's command", this->name_.c_str());

    return false;

  } else {

    activation_attitude_cmd_ = *cmd;
    uav_mass_difference_     = cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    Ib_b_[0] = cmd->disturbance_bx_b;
    Ib_b_[1] = cmd->disturbance_by_b;

    Iw_w_[0] = cmd->disturbance_wx_w;
    Iw_w_[1] = cmd->disturbance_wy_w;

    ROS_INFO("[%s]: setting the mass difference and disturbances from the last AttitudeCmd: mass difference: %.2f kg, Ib_b_: %.2f, %.2f N, Iw_w_: %.2f, %.2f N",
             this->name_.c_str(), uav_mass_difference_, Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);

    ROS_INFO("[%s]: activated with the last controllers's command", this->name_.c_str());
  }

  // rampup check
  if (_rampup_enabled_) {

    double hover_thrust_     = sqrt(cmd->total_mass * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;
    double thrust_difference = hover_thrust_ - cmd->thrust;

    if (thrust_difference > 0) {
      rampup_direction_ = 1;
    } else if (thrust_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[MpcController]: activating rampup with initial thrust: %.4f, target: %.4f", cmd->thrust, hover_thrust_);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_thrust_     = cmd->thrust;

    rampup_duration_ = fabs(thrust_difference) / _rampup_speed_;
  }

  first_iteration_ = true;

  ROS_INFO("[%s]: activated", this->name_.c_str());

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void MpcController::deactivate(void) {

  is_active            = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[%s]: deactivated", this->name_.c_str());
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr MpcController::update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("update");

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }

  if (!is_active) {
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

    ROS_DEBUG("[%s]: the last odometry message came too close (%.2f s)!", this->name_.c_str(), dt);

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
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference
  Eigen::Vector3d           Rp, Rv, Ra, Rw;
  Eigen::Quaternion<double> Rq;

  Eigen::Matrix3d Rd;

  Rp << reference->position.x, reference->position.y, reference->position.z;  // fill the desired position
  Rv << reference->velocity.x, reference->velocity.y, reference->velocity.z;
  Rw << 0, 0, reference->yaw_dot;

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << uav_state->pose.orientation.x, uav_state->pose.orientation.y, uav_state->pose.orientation.z, uav_state->pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // --------------------------------------------------------------
  // |                     MPC lateral control                    |
  // --------------------------------------------------------------

  // | ------------------- initial conditions ------------------- |

  Eigen::MatrixXd initial_x = Eigen::MatrixXd::Zero(3, 1);
  Eigen::MatrixXd initial_y = Eigen::MatrixXd::Zero(3, 1);
  Eigen::MatrixXd initial_z = Eigen::MatrixXd::Zero(3, 1);

  if (fabs(uav_state->velocity.linear.x) < _max_speed_horizontal_) {
    initial_x << uav_state->pose.position.x, uav_state->velocity.linear.x, reference->acceleration.x;
  } else {
    initial_x << uav_state->pose.position.x, reference->velocity.x, reference->acceleration.x;
    ROS_ERROR_THROTTLE(1.0, "[MpcController]: odometry x velocity exceedds constraints (%.2f > %.2f m), using reference for initial condition",
                       fabs(uav_state->velocity.linear.x), _max_speed_horizontal_);
  }

  if (fabs(uav_state->velocity.linear.x) < _max_speed_horizontal_) {
    initial_y << uav_state->pose.position.y, uav_state->velocity.linear.y, reference->acceleration.y;
  } else {
    initial_y << uav_state->pose.position.y, reference->velocity.y, reference->acceleration.y;
    ROS_ERROR_THROTTLE(1.0, "[MpcController]: odometry y velocity exceedds constraints (%.2f > %.2f m), using reference for initial condition",
                       fabs(uav_state->velocity.linear.y), _max_speed_horizontal_);
  }

  if (fabs(uav_state->velocity.linear.z) < _max_speed_vertical_) {
    initial_z << uav_state->pose.position.z, uav_state->velocity.linear.z, reference->acceleration.z;
  } else {
    initial_z << uav_state->pose.position.z, reference->velocity.z, reference->acceleration.z;
    ROS_ERROR_THROTTLE(1.0, "[MpcController]: odometry z velocity exceedds constraints (%.2f > %.2f m), using reference for initial condition",
                       fabs(uav_state->velocity.linear.z), _max_speed_vertical_);
  }

  // | ---------------------- set reference --------------------- |

  Eigen::MatrixXd mpc_reference_x = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);
  Eigen::MatrixXd mpc_reference_y = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);
  Eigen::MatrixXd mpc_reference_z = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);

  // prepare the full reference vector
  for (int i = 0; i < _horizon_length_; i++) {

    mpc_reference_x((i * _n_states_) + 0, 0) = reference->position.x;
    mpc_reference_x((i * _n_states_) + 1, 0) = reference->velocity.x;
    mpc_reference_x((i * _n_states_) + 2, 0) = reference->acceleration.x;

    mpc_reference_y((i * _n_states_) + 0, 0) = reference->position.y;
    mpc_reference_y((i * _n_states_) + 1, 0) = reference->velocity.y;
    mpc_reference_y((i * _n_states_) + 2, 0) = reference->acceleration.y;

    mpc_reference_z((i * _n_states_) + 0, 0) = reference->position.z;
    mpc_reference_z((i * _n_states_) + 1, 0) = reference->velocity.z;
    mpc_reference_z((i * _n_states_) + 2, 0) = reference->acceleration.z;
  }

  // | ------------------ set the penalizations ----------------- |

  std::vector<double> temp_Q_horizontal = _Q_;
  std::vector<double> temp_Q_vertical   = _Q_z_;

  std::vector<double> temp_S_horizontal = _S_;
  std::vector<double> temp_S_vertical   = _S_z_;

  if (!reference->use_position_horizontal) {
    temp_Q_horizontal[0] = 0;
    temp_S_horizontal[0] = 0;
  }

  if (!reference->use_velocity_horizontal) {
    temp_Q_horizontal[1] = 0;
    temp_S_horizontal[1] = 0;
  }

  if (!reference->use_position_vertical) {
    temp_Q_vertical[0] = 0;
    temp_S_vertical[0] = 0;
  }

  if (!reference->use_velocity_vertical) {
    temp_Q_vertical[1] = 0;
    temp_S_vertical[1] = 0;
  }

  // | ------------------------ optimize ------------------------ |

  cvx_x_->lock();
  cvx_x_->setQ(temp_Q_horizontal);
  cvx_x_->setS(temp_S_horizontal);
  cvx_x_->setParams();
  cvx_x_->setLastInput(cvx_x_u);
  cvx_x_->loadReference(mpc_reference_x);
  cvx_x_->setLimits(_max_speed_horizontal_, 999, max_acceleration_horizontal_, _max_jerk_, _dt1_, _dt2_);
  cvx_x_->setInitialState(initial_x);
  [[maybe_unused]] int iters_x = cvx_x_->solveCvx();
  cvx_x_u                      = cvx_x_->getFirstControlInput();
  cvx_x_->unlock();

  cvx_y_->lock();
  cvx_y_->setQ(temp_Q_horizontal);
  cvx_y_->setS(temp_S_horizontal);
  cvx_y_->setParams();
  cvx_y_->setLastInput(cvx_y_u);
  cvx_y_->loadReference(mpc_reference_y);
  cvx_y_->setLimits(_max_speed_horizontal_, 999, max_acceleration_horizontal_, _max_jerk_, _dt1_, _dt2_);
  cvx_y_->setInitialState(initial_y);
  [[maybe_unused]] int iters_y = cvx_y_->solveCvx();
  cvx_y_u                      = cvx_y_->getFirstControlInput();
  cvx_y_->unlock();

  cvx_z_->lock();
  cvx_z_->setQ(temp_Q_vertical);
  cvx_z_->setS(temp_S_vertical);
  cvx_z_->setParams();
  cvx_z_->setLastInput(cvx_z_u);
  cvx_z_->loadReference(mpc_reference_z);
  cvx_z_->setLimits(_max_speed_vertical_, _max_acceleration_vertical_, _max_u_vertical_, 999.0, _dt1_, _dt2_);
  cvx_z_->setInitialState(initial_z);
  [[maybe_unused]] int iters_z = cvx_z_->solveCvx();
  cvx_z_u                      = cvx_z_->getFirstControlInput();
  cvx_z_->unlock();

  // | ----------- disable lateral feedback if needed ----------- |

  if (reference->disable_position_gains) {
    cvx_x_u = 0;
    cvx_y_u = 0;
  }

  // | --------------- calculate the control erros -------------- |

  Eigen::Vector3d Ep = Op - Rp;
  Eigen::Vector3d Ev = Ov - Rv;

  // | -------- calculate the current orientation angles -------- |

  double         uav_yaw, uav_pitch, uav_roll;
  tf::Quaternion uav_attitude;
  quaternionMsgToTF(uav_state->pose.orientation, uav_attitude);
  tf::Matrix3x3 m(uav_attitude);
  m.getRPY(uav_roll, uav_pitch, uav_yaw);

  // | --------------------- load the gains --------------------- |

  Eigen::Vector3d Ka;
  Eigen::Array3d  Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains_);

    Kq << kqxy_, kqxy_, kqz_;

    if (!reference->use_yaw) {
      Kq[2] = 0;
    }

    Kw << kwxy_, kwxy_, kwz_;
  }

  // | -------------- recalculate the hover thrust -------------- |

  hover_thrust_ = sqrt((_uav_mass_ + uav_mass_difference_) * _g_) * _motor_params_.hover_thrust_a + _motor_params_.hover_thrust_b;

  // | ---------- desired orientation matrix and force ---------- |

  Eigen::Vector2d Ib_w = Eigen::Vector2d(0, 0);

  // get body disturbance in the world frame
  {

    geometry_msgs::Vector3Stamped Ib_b_stamped;

    Ib_b_stamped.header.stamp    = ros::Time::now();
    Ib_b_stamped.header.frame_id = "fcu_untilted";
    Ib_b_stamped.vector.x        = Ib_b_(0);
    Ib_b_stamped.vector.y        = Ib_b_(1);
    Ib_b_stamped.vector.z        = 0;

    auto res = common_handlers_->transformer->transformSingle(uav_state_.header.frame_id, Ib_b_stamped);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[MpcController]: could not transform the Ib_b_ to the world frame");
    }
  }

  if (reference->use_acceleration) {
    Ra << reference->acceleration.x + cvx_x_u, reference->acceleration.y + cvx_y_u, reference->acceleration.z + cvx_z_u;
  } else {
    Ra << cvx_x_u, cvx_y_u, cvx_z_u;
  }

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward = total_mass * (Eigen::Vector3d(0, 0, _g_) + Ra);

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

    ROS_ERROR("[%s]: NaN detected in variable 'theta', returning null", this->name_.c_str());

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (_tilt_angle_failsafe_ > 1e-3 && theta > _tilt_angle_failsafe_) {

    ROS_ERROR("[%s]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", this->name_.c_str(),
              (180.0 / M_PI) * theta, (180.0 / M_PI) * _tilt_angle_failsafe_);
    ROS_INFO("[%s]: f = [%.2f, %.2f, %.2f]", this->name_.c_str(), f[0], f[1], f[2]);
    ROS_INFO("[%s]: integral feedback: [%.2f, %.2f, %.2f]", this->name_.c_str(), integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[%s]: feed forward: [%.2f, %.2f, %.2f]", this->name_.c_str(), feed_forward[0], feed_forward[1], feed_forward[2]);
    ROS_INFO("[%s]: position_cmd: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", this->name_.c_str(), reference->position.x, reference->position.y,
             reference->position.z, reference->yaw);
    ROS_INFO("[%s]: odometry: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", this->name_.c_str(), uav_state->pose.position.x, uav_state->pose.position.y,
             uav_state->pose.position.z, uav_yaw);

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle
  if (_tilt_angle_saturation_ > 1e-3 && theta > _tilt_angle_saturation_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", this->name_.c_str(), (theta / M_PI) * 180.0,
                      (_tilt_angle_saturation_ / M_PI) * 180.0);
    theta = _tilt_angle_saturation_;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

  // | ---------------------- yaw reference --------------------- |

  if (reference->use_yaw) {
    Rq.coeffs() << 0, 0, sin(reference->yaw / 2.0), cos(reference->yaw / 2.0);
  } else {
    Rq.coeffs() << 0, 0, sin(uav_yaw / 2.0), cos(uav_yaw / 2.0);
  }

  // | ------------- construct the rotational matrix ------------ |

  Rd.col(2) = f_norm;
  Rd.col(1) = Rd.col(2).cross(Rq.toRotationMatrix().col(0));
  Rd.col(1).normalize();
  Rd.col(0) = Rd.col(1).cross(Rd.col(2));

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
    ROS_WARN_THROTTLE(1.0, "[%s]: just so you know, the desired thrust force is negative (%.2f)", this->name_.c_str(), thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[%s]: NaN detected in variable 'thrust', setting it to 0 and returning!!!", this->name_.c_str());

  } else if (thrust > _thrust_saturation_) {

    thrust = _thrust_saturation_;
    ROS_WARN_THROTTLE(1.0, "[%s]: saturating thrust to %.2f", this->name_.c_str(), _thrust_saturation_);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[%s]: saturating thrust to %.2f", this->name_.c_str(), 0.0);
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // | ----------------------- gain muting ---------------------- |

  if (mute_lateral_gains_ && !reference->disable_position_gains) {
    mute_lateral_gains_after_toggle_ = true;
  }
  mute_lateral_gains_ = reference->disable_position_gains;

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

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[MpcController]: could not transform the position error to fcu_untilted");
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

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ev_stamped);

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[MpcController]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error

    // antiwindup
    double temp_gain = kibxy_;
    if (rampup_active_ || sqrt(pow(uav_state->velocity.linear.x, 2) + pow(uav_state->velocity.linear.y, 2)) > 0.3) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for body integral kicks in", this->name_.c_str());
    }

    if (integral_terms_enabled_) {
      if (reference->use_position_horizontal) {
        Ib_b_ -= temp_gain * Ep_fcu_untilted * dt;
      } else if (reference->use_velocity_horizontal) {
        Ib_b_ -= temp_gain * Ev_fcu_untilted * dt;
      }
    }

    // saturate the body X
    double body_integral_saturated = false;
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
    if (rampup_active_ || sqrt(pow(uav_state->velocity.linear.x, 2) + pow(uav_state->velocity.linear.y, 2)) > 0.3) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for world integral kicks in", this->name_.c_str());
    }

    if (integral_terms_enabled_) {
      if (reference->use_position_horizontal) {
        Iw_w_ -= temp_gain * Ep.head(2) * dt;
      } else if (reference->use_velocity_horizontal) {
        Iw_w_ -= temp_gain * Ev.head(2) * dt;
      }
    }

    // saturate the world X
    double world_integral_saturated = false;
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
        (fabs(uav_state->velocity.linear.z) > 0.3 && ((Ep[2] < 0 && uav_state->velocity.linear.z > 0) || (Ep[2] > 0 && uav_state->velocity.linear.z < 0)))) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for the mass kicks in", this->name_.c_str());
    }

    if (reference->use_position_vertical) {
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
  // |            report on the values of the integrals           |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_integrals_);

    ROS_INFO_THROTTLE(5.0, "[%s]: world error integral: x %.2f N, y %.2f N, lim: %.2f N", this->name_.c_str(), Iw_w_[X], Iw_w_[Y], kiwxy_lim_);
    ROS_INFO_THROTTLE(5.0, "[%s]: body error integral:  x %.2f N, y %.2f N, lim: %.2f N", this->name_.c_str(), Ib_b_[X], Ib_b_[Y], kibxy_lim_);
  }

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | -------------------- saturate yaw rate ------------------- |

  if (!std::isfinite(t[2])) {
    t[2] = 0;
    ROS_ERROR("[MpcController]: NaN detected in variable 't[2]', setting it to 0 and returning!!!");
  } else if (t[2] > _yaw_rate_saturation_) {
    t[2] = _yaw_rate_saturation_;
  } else if (t[2] < -_yaw_rate_saturation_) {
    t[2] = -_yaw_rate_saturation_;
  }

  // --------------------------------------------------------------
  // |              compensated desired acceleration              |
  // --------------------------------------------------------------

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Quaterniond des_quater = Eigen::Quaterniond(Rd);

    // rotate the drone's z axis
    Eigen::Vector3d uav_z_in_world = des_quater * Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d thrust_vector = thrust_force * uav_z_in_world;

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
    double world_accel_z = reference->acceleration.z;

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


  if (_output_mode_ == OUTPUT_ATTITUDE_RATE) {

    // output the desired attitude rate
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

    output_command->euler_attitude_set = false;

    output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  } else if (_output_mode_ == OUTPUT_ATTITUDE_QUATERNION) {

    // output the desired attitude
    Eigen::Quaterniond thrust_vec       = Eigen::Quaterniond(Rd);
    output_command->quater_attitude.w   = thrust_vec.w();
    output_command->quater_attitude.x   = thrust_vec.x();
    output_command->quater_attitude.y   = thrust_vec.y();
    output_command->quater_attitude.z   = thrust_vec.z();
    output_command->quater_attitude_set = true;

    output_command->euler_attitude_set = false;
    output_command->attitude_rate_set  = false;

    output_command->mode_mask = output_command->MODE_QUATER_ATTITUDE;

    ROS_WARN_THROTTLE(1.0, "[%s]: outputting attitude quaternion (this is not normal)", this->name_.c_str());
  }

  output_command->desired_acceleration.x = desired_x_accel;
  output_command->desired_acceleration.y = desired_y_accel;
  output_command->desired_acceleration.z = desired_z_accel;

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_         = false;
      output_command->thrust = thrust;

      ROS_INFO("[%s]: rampup finished", this->name_.c_str());

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_thrust_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      output_command->thrust = rampup_thrust_;

      ROS_INFO_THROTTLE(0.1, "[%s]: ramping up thrust, %.4f", this->name_.c_str(), output_command->thrust);
    }

  } else {
    output_command->thrust = thrust;
  }

  output_command->ramping_up = rampup_active_;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = Ib_b_[0];
  output_command->disturbance_by_b = Ib_b_[1];

  output_command->disturbance_bx_w = Ib_w[0];
  output_command->disturbance_by_w = Ib_w[1];

  output_command->disturbance_wx_w = Iw_w_[0];
  output_command->disturbance_wy_w = Iw_w_[1];

  // set the constraints
  output_command->controller_enforcing_constraints = true;

  output_command->horizontal_speed_constraint = 0.5 * _max_speed_horizontal_;
  output_command->horizontal_acc_constraint   = 0.5 * max_acceleration_horizontal_;

  output_command->vertical_asc_speed_constraint = 0.5 * _max_speed_vertical_;
  output_command->vertical_asc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  output_command->vertical_desc_speed_constraint = 0.5 * _max_speed_vertical_;
  output_command->vertical_desc_acc_constraint   = 0.5 * _max_acceleration_vertical_;

  output_command->controller = this->name_;

  last_attitude_cmd_ = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus MpcController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void MpcController::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg) {

  ROS_INFO("[%s]: switching the odometry source", this->name_.c_str());

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

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void MpcController::callbackDrs(mrs_controllers::mpc_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_gains_ = config;
  }

  ROS_INFO("[%s]: DRS updated gains", this->name_.c_str());
}

//}

/* //{ callbackSetIntegralTerms() */

bool MpcController::callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  integral_terms_enabled_ = req.data;

  std::stringstream ss;

  ss << "integral terms %s" << (integral_terms_enabled_ ? "enabled" : "disabled");

  ROS_INFO_STREAM_THROTTLE(1.0, "[MpcController]: " << ss.str());

  res.message = ss.str();
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void MpcController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("timerGainsFilter", _gains_filter_timer_rate_, 0.05, event);

  double gain_coeff                = 1;
  mute_lateral_gains_after_toggle_ = false;

  bool bypass_filter = mute_lateral_gains_ || mute_lateral_gains_after_toggle_;

  if (mute_lateral_gains_) {
    gain_coeff = _mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    kqxy_      = calculateGainChange(kqxy_, drs_gains_.kqxy, false, "kqxy");
    kqz_       = calculateGainChange(kqz_, drs_gains_.kqz, false, "kqz");
    kwxy_      = calculateGainChange(kwxy_, drs_gains_.kwxy, false, "kwxy");
    kwz_       = calculateGainChange(kwz_, drs_gains_.kwz, false, "kwz");
    km_        = calculateGainChange(km_, drs_gains_.km, false, "km");
    km_lim_    = calculateGainChange(km_lim_, drs_gains_.km_lim, false, "km_lim");
    kiwxy_     = calculateGainChange(kiwxy_, drs_gains_.kiwxy * gain_coeff, bypass_filter, "kiwxy");
    kibxy_     = calculateGainChange(kibxy_, drs_gains_.kibxy * gain_coeff, bypass_filter, "kibxy");
    kiwxy_lim_ = calculateGainChange(kiwxy_lim_, drs_gains_.kiwxy_lim, false, "kiwxy_lim");
    kibxy_lim_ = calculateGainChange(kibxy_lim_, drs_gains_.kibxy_lim, false, "kibxy_lim");
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* calculateGainChange() //{ */

double MpcController::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

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
    ROS_INFO_THROTTLE(1.0, "[%s]: changing gain '%s' from %.2f to %.2f", this->name_.c_str(), name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

//}

}  // namespace mpc_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::mpc_controller::MpcController, mrs_uav_manager::Controller)

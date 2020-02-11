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

  void dynamicReconfigureCallback(mrs_controllers::mpc_controllerConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  bool                                               is_initialized = false;
  bool                                               is_active      = false;
  std::string                                        name_;
  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

private:
  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                        config_mutex_;
  typedef mrs_controllers::mpc_controllerConfig Config;
  typedef dynamic_reconfigure::Server<Config>   ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>          reconfigure_server_;
  void                                          drs_callback(mrs_controllers::mpc_controllerConfig &config, uint32_t level);
  mrs_controllers::mpc_controllerConfig         drs_desired_gains;

private:
  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;
  double                       hover_thrust;

  // actual gains (used and already filtered)
  double kiwxy, kibxy;
  double kiwxy_lim, kibxy_lim;
  double km, km_lim;
  double kqxy, kqz;  // attitude gains
  double kwxy, kwz;  // attitude rate gains

  // desired gains (set by DRS)
  std::mutex mutex_gains;
  std::mutex mutex_desired_gains;

  double tilt_angle_saturation_;
  double tilt_angle_failsafe_;
  double thrust_saturation_;
  double yaw_rate_saturation_;

  double max_tilt_angle_;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;

  bool   mute_lateral_gains               = false;
  bool   mutex_lateral_gains_after_toggle = false;
  double mute_coefficitent_;

private:
  ros::ServiceServer service_set_integral_terms;
  bool               callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool               integral_terms_enabled = true;

  // --------------------------------------------------------------
  // |                       MPC controller                       |
  // --------------------------------------------------------------

private:
  int n;  // number of states

  double dt1, dt2;

  double cvx_x_u = 0;
  double cvx_y_u = 0;
  double cvx_z_u = 0;

  int horizon_length_;

  double max_speed_horizontal_, max_acceleration_horizontal_, max_jerk_;
  double max_speed_vertical_, max_acceleration_vertical_, max_u_vertical_;

  std::vector<double> Q, S;
  std::vector<double> Q_z, S_z;

  mrs_controllers::cvx_wrapper::CvxWrapper *cvx_x;
  mrs_controllers::cvx_wrapper::CvxWrapper *cvx_y;
  mrs_controllers::cvx_wrapper::CvxWrapper *cvx_z;

  bool cvx_verbose_ = false;
  int  cvx_max_iterations_;

private:
  mrs_lib::Profiler profiler;
  bool              profiler_enabled_ = false;

private:
  ros::Timer timer_gain_filter;
  void       timerGainsFilter(const ros::TimerEvent &event);

  int    gains_filter_timer_rate_;
  double gains_filter_change_rate_;
  double gains_filter_min_change_rate_;

  double gains_filter_max_change_;  // calculated from change_rate_/timer_rate_;
  double gains_filter_min_change_;  // calculated from change_rate_/timer_rate_;

private:
  int        output_mode_;  // 1 = ATTITUDE RATES, 2 = ATTITUDE QUATERNION
  std::mutex mutex_output_mode;

private:
  Eigen::Vector2d Ib_b;  // body error integral in the body frame
  Eigen::Vector2d Iw_w;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

private:
  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_thrust_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time;
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

  this->name_ = name;

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;
  this->uav_mass_     = uav_mass;
  this->g_            = g;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "MpcController");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // | --------------------- mpc controller --------------------- |

  // load the dynamicall model parameters
  param_loader.load_param("mpc_model/number_of_states", n);
  param_loader.load_param("mpc_model/dt1", dt1);
  param_loader.load_param("mpc_model/dt2", dt2);

  param_loader.load_param("mpc_parameters/horizon_length", horizon_length_);

  param_loader.load_param("mpc_parameters/horizontal/max_speed", max_speed_horizontal_);
  param_loader.load_param("mpc_parameters/horizontal/max_acceleration", max_acceleration_horizontal_);
  param_loader.load_param("mpc_parameters/horizontal/max_jerk", max_jerk_);

  param_loader.load_param("mpc_parameters/horizontal/Q", Q);
  param_loader.load_param("mpc_parameters/horizontal/S", S);

  param_loader.load_param("mpc_parameters/vertical/max_speed", max_speed_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_acceleration", max_acceleration_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_u", max_u_vertical_);

  param_loader.load_param("mpc_parameters/vertical/Q", Q_z);
  param_loader.load_param("mpc_parameters/vertical/S", S_z);

  param_loader.load_param("cvx_parameters/verbose", cvx_verbose_);
  param_loader.load_param("cvx_parameters/max_iterations", cvx_max_iterations_);

  // | ------------------------- rampup ------------------------- |

  param_loader.load_param("rampup/enabled", _rampup_enabled_);
  param_loader.load_param("rampup/speed", _rampup_speed_);

  // | --------------------- integral gains --------------------- |

  param_loader.load_param("integral_gains/kiw", kiwxy);
  param_loader.load_param("integral_gains/kib", kibxy);

  // integrator limits
  param_loader.load_param("integral_gains/kiw_lim", kiwxy_lim);
  param_loader.load_param("integral_gains/kib_lim", kibxy_lim);

  // | ------------- height and attitude controller ------------- |

  // attitude gains
  param_loader.load_param("attitude_feedback/default_gains/horizontal/attitude/kq", kqxy);
  param_loader.load_param("attitude_feedback/default_gains/vertical/attitude/kq", kqz);

  // attitude rate gains
  param_loader.load_param("attitude_feedback/default_gains/horizontal/attitude/kw", kwxy);
  param_loader.load_param("attitude_feedback/default_gains/vertical/attitude/kw", kwz);

  // mass estimator
  param_loader.load_param("attitude_feedback/default_gains/weight_estimator/km", km);
  param_loader.load_param("attitude_feedback/default_gains/weight_estimator/km_lim", km_lim);

  // constraints
  param_loader.load_param("attitude_feedback/tilt_angle_saturation", tilt_angle_saturation_);
  param_loader.load_param("attitude_feedback/tilt_angle_failsafe", tilt_angle_failsafe_);
  param_loader.load_param("attitude_feedback/thrust_saturation", thrust_saturation_);
  param_loader.load_param("attitude_feedback/yaw_rate_saturation", yaw_rate_saturation_);

  // if yaw_rate_saturation is 0 (or close), set it to something very high, so its inactive
  if (yaw_rate_saturation_ <= 1e-3) {
    yaw_rate_saturation_ = 10e6;
  }

  // gain filtering
  param_loader.load_param("attitude_feedback/gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("attitude_feedback/gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("attitude_feedback/gains_filter/min_change_rate", gains_filter_min_change_rate_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  // output mode
  param_loader.load_param("output_mode", output_mode_);

  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[%s]: output mode has to be {1, 2}!", this->name_.c_str());
  }

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", this->name_.c_str());
    ros::shutdown();
  }

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180) * M_PI;

  uav_mass_difference = 0;
  Iw_w                = Eigen::Vector2d::Zero(2);
  Ib_b                = Eigen::Vector2d::Zero(2);

  // --------------------------------------------------------------
  // |                       prepare cvxgen                       |
  // --------------------------------------------------------------

  cvx_x = new mrs_controllers::cvx_wrapper::CvxWrapper(cvx_verbose_, cvx_max_iterations_, Q, S, dt1, dt2, 0, 1.0);
  cvx_y = new mrs_controllers::cvx_wrapper::CvxWrapper(cvx_verbose_, cvx_max_iterations_, Q, S, dt1, dt2, 0, 1.0);
  cvx_z = new mrs_controllers::cvx_wrapper::CvxWrapper(cvx_verbose_, cvx_max_iterations_, Q_z, S_z, dt1, dt2, 0.5, 0.5);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_desired_gains.kiwxy     = kiwxy;
  drs_desired_gains.kibxy     = kibxy;
  drs_desired_gains.kqxy      = kqxy;
  drs_desired_gains.kqz       = kqz;
  drs_desired_gains.kwxy      = kwxy;
  drs_desired_gains.kwz       = kwz;
  drs_desired_gains.km        = km;
  drs_desired_gains.km_lim    = km_lim;
  drs_desired_gains.kiwxy_lim = kiwxy_lim;
  drs_desired_gains.kibxy_lim = kibxy_lim;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_desired_gains);
  ReconfigureServer::CallbackType f = boost::bind(&MpcController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                       Service servers                      |
  // --------------------------------------------------------------

  service_set_integral_terms = nh_.advertiseService("set_integral_terms_in", &MpcController::callbackSetIntegralTerms, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = mrs_lib::Profiler(nh_, "MpcController", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &MpcController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", this->name_.c_str());
    ros::shutdown();
  }

  ROS_INFO("[%s]: initialized", this->name_.c_str());

  is_initialized = true;
}

//}

/* //{ activate() */

bool MpcController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[%s]: activated without getting the last controllers's command.", this->name_.c_str());

    return false;

  } else {

    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;

    activation_control_command_.controller_enforcing_constraints = false;

    Ib_b[0] = cmd->disturbance_bx_b;
    Ib_b[1] = cmd->disturbance_by_b;

    Iw_w[0] = cmd->disturbance_wx_w;
    Iw_w[1] = cmd->disturbance_wy_w;

    ROS_INFO("[%s]: setting the mass difference and disturbances from the last AttitudeCmd: mass difference: %.2f kg, Ib_b: %.2f, %.2f N, Iw_w: %.2f, %.2f N",
             this->name_.c_str(), uav_mass_difference, Ib_b[0], Ib_b[1], Iw_w[0], Iw_w[1]);

    ROS_INFO("[%s]: activated with the last controllers's command.", this->name_.c_str());
  }

  // rampup check
  if (_rampup_enabled_) {

    double hover_thrust      = sqrt(cmd->total_mass * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
    double thrust_difference = hover_thrust - cmd->thrust;

    if (thrust_difference > 0) {
      rampup_direction_ = 1;
    } else if (thrust_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[MpcController]: activating rampup with initial thrust: %.4f, target: %.4f", cmd->thrust, hover_thrust);

    rampup_active_    = true;
    rampup_start_time = ros::Time::now();
    rampup_last_time_ = ros::Time::now();
    rampup_thrust_    = cmd->thrust;

    rampup_duration_ = fabs(thrust_difference) / _rampup_speed_;
  }

  first_iteration = true;

  ROS_INFO("[%s]: activated", this->name_.c_str());

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void MpcController::deactivate(void) {

  is_active           = false;
  first_iteration     = false;
  uav_mass_difference = 0;

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

  if (fabs(uav_state->velocity.linear.x) < max_speed_horizontal_) {
    initial_x << uav_state->pose.position.x, uav_state->velocity.linear.x, reference->acceleration.x;
  } else {
    initial_x << uav_state->pose.position.x, reference->velocity.x, reference->acceleration.x;
    ROS_ERROR_THROTTLE(1.0, "[MpcController]: odometry x velocity exceedds constraints (%.2f > %.2f m), using reference for initial condition",
                       fabs(uav_state->velocity.linear.x), max_speed_horizontal_);
  }

  if (fabs(uav_state->velocity.linear.x) < max_speed_horizontal_) {
    initial_y << uav_state->pose.position.y, uav_state->velocity.linear.y, reference->acceleration.y;
  } else {
    initial_y << uav_state->pose.position.y, reference->velocity.y, reference->acceleration.y;
    ROS_ERROR_THROTTLE(1.0, "[MpcController]: odometry y velocity exceedds constraints (%.2f > %.2f m), using reference for initial condition",
                       fabs(uav_state->velocity.linear.y), max_speed_horizontal_);
  }

  if (fabs(uav_state->velocity.linear.z) < max_speed_vertical_) {
    initial_z << uav_state->pose.position.z, uav_state->velocity.linear.z, reference->acceleration.z;
  } else {
    initial_z << uav_state->pose.position.z, reference->velocity.z, reference->acceleration.z;
    ROS_ERROR_THROTTLE(1.0, "[MpcController]: odometry z velocity exceedds constraints (%.2f > %.2f m), using reference for initial condition",
                       fabs(uav_state->velocity.linear.z), max_speed_vertical_);
  }

  // | ---------------------- set reference --------------------- |

  Eigen::MatrixXd mpc_reference_x = Eigen::MatrixXd::Zero(horizon_length_ * n, 1);
  Eigen::MatrixXd mpc_reference_y = Eigen::MatrixXd::Zero(horizon_length_ * n, 1);
  Eigen::MatrixXd mpc_reference_z = Eigen::MatrixXd::Zero(horizon_length_ * n, 1);

  // prepare the full reference vector
  for (int i = 0; i < horizon_length_; i++) {

    mpc_reference_x((i * n) + 0, 0) = reference->position.x;
    mpc_reference_x((i * n) + 1, 0) = reference->velocity.x;
    mpc_reference_x((i * n) + 2, 0) = reference->acceleration.x;

    mpc_reference_y((i * n) + 0, 0) = reference->position.y;
    mpc_reference_y((i * n) + 1, 0) = reference->velocity.y;
    mpc_reference_y((i * n) + 2, 0) = reference->acceleration.y;

    mpc_reference_z((i * n) + 0, 0) = reference->position.z;
    mpc_reference_z((i * n) + 1, 0) = reference->velocity.z;
    mpc_reference_z((i * n) + 2, 0) = reference->acceleration.z;
  }

  // | ------------------ set the penalizations ----------------- |

  std::vector<double> temp_Q_horizontal = Q;
  std::vector<double> temp_Q_vertical   = Q_z;

  std::vector<double> temp_S_horizontal = S;
  std::vector<double> temp_S_vertical   = S_z;

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

  cvx_x->lock();
  cvx_x->setQ(temp_Q_horizontal);
  cvx_x->setS(temp_S_horizontal);
  cvx_x->setParams();
  cvx_x->setLastInput(cvx_x_u);
  cvx_x->loadReference(mpc_reference_x);
  cvx_x->setLimits(max_speed_horizontal_, 999, max_acceleration_horizontal_, max_jerk_, dt1, dt2);
  cvx_x->setInitialState(initial_x);
  [[maybe_unused]] int iters_x = cvx_x->solveCvx();
  cvx_x_u                      = cvx_x->getFirstControlInput();
  cvx_x->unlock();

  cvx_y->lock();
  cvx_y->setQ(temp_Q_horizontal);
  cvx_y->setS(temp_S_horizontal);
  cvx_y->setParams();
  cvx_y->setLastInput(cvx_y_u);
  cvx_y->loadReference(mpc_reference_y);
  cvx_y->setLimits(max_speed_horizontal_, 999, max_acceleration_horizontal_, max_jerk_, dt1, dt2);
  cvx_y->setInitialState(initial_y);
  [[maybe_unused]] int iters_y = cvx_y->solveCvx();
  cvx_y_u                      = cvx_y->getFirstControlInput();
  cvx_y->unlock();

  cvx_z->lock();
  cvx_z->setQ(temp_Q_vertical);
  cvx_z->setS(temp_S_vertical);
  cvx_z->setParams();
  cvx_z->setLastInput(cvx_z_u);
  cvx_z->loadReference(mpc_reference_z);
  cvx_z->setLimits(max_speed_vertical_, max_acceleration_vertical_, max_u_vertical_, 999.0, dt1, dt2);
  cvx_z->setInitialState(initial_z);
  [[maybe_unused]] int iters_z = cvx_z->solveCvx();
  cvx_z_u                      = cvx_z->getFirstControlInput();
  cvx_z->unlock();

  // --------------------------------------------------------------
  // |           disble lateral feedback during takeoff           |
  // --------------------------------------------------------------

  if (reference->disable_position_gains) {
    cvx_x_u = 0;
    cvx_y_u = 0;
  }

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  Eigen::Vector3d Ep = Op - Rp;
  Eigen::Vector3d Ev = Ov - Rv;

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = uav_state->header.stamp;

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (uav_state->header.stamp - last_update).toSec();
    last_update = uav_state->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[%s]: the last odometry message came too close! %f", this->name_.c_str(), dt);

    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  // --------------------------------------------------------------
  // |          caculate the current oritentation angles          |
  // --------------------------------------------------------------

  double         uav_yaw, uav_pitch, uav_roll;
  tf::Quaternion uav_attitude;
  quaternionMsgToTF(uav_state->pose.orientation, uav_attitude);
  tf::Matrix3x3 m(uav_attitude);
  m.getRPY(uav_roll, uav_pitch, uav_yaw);

  // --------------------------------------------------------------
  // |                            gains                           |
  // --------------------------------------------------------------
  //
  Eigen::Vector3d Ka;
  Eigen::Array3d  Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains);

    Kq << kqxy, kqxy, kqz;

    if (!reference->use_yaw) {
      Kq[2] = 0;
    }

    Kw << kwxy, kwxy, kwz;
  }

  // --------------------------------------------------------------
  // |                recalculate the hover thrust                |
  // --------------------------------------------------------------

  hover_thrust = sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

  // --------------------------------------------------------------
  // |                 desired orientation matrix                 |
  // --------------------------------------------------------------

  Eigen::Vector2d Ib_w = rotate2d(Ib_b, uav_yaw);

  if (reference->use_acceleration) {
    Ra << reference->acceleration.x + cvx_x_u, reference->acceleration.y + cvx_y_u, reference->acceleration.z + cvx_z_u;
  } else {
    Ra << cvx_x_u, cvx_y_u, cvx_z_u;
  }

  double total_mass = uav_mass_ + uav_mass_difference;

  Eigen::Vector3d feed_forward = total_mass * (Eigen::Vector3d(0, 0, g_) + Ra);

  Eigen::Vector3d integral_feedback;

  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w[0], Ib_w[1] + Iw_w[1], 0;
  }

  Eigen::Vector3d f = integral_feedback + feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[%s]: the calculated downwards desired force is negative (%.2f) -> mitigating the flip", this->name_.c_str(), f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in the spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[%s]: NaN detected in variable \"theta\", returning null", this->name_.c_str());

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (tilt_angle_failsafe_ > 1e-3 && theta > tilt_angle_failsafe_) {

    ROS_ERROR("[%s]: The produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", this->name_.c_str(),
              (180.0 / M_PI) * theta, (180.0 / M_PI) * tilt_angle_failsafe_);
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
  if (tilt_angle_saturation_ > 1e-3 && theta > tilt_angle_saturation_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: tilt is being saturated, desired: %f deg, saturated %f deg", this->name_.c_str(), (theta / M_PI) * 180.0,
                      (tilt_angle_saturation_ / M_PI) * 180.0);
    theta = tilt_angle_saturation_;
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

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------

  /* orientation error */
  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;
  Eq << (E(2, 1) - E(1, 2)) / 2.0, (E(0, 2) - E(2, 0)) / 2.0, (E(1, 0) - E(0, 1)) / 2.0;

  // --------------------------------------------------------------
  // |                     angular rate error                     |
  // --------------------------------------------------------------
  //
  Eigen::Vector3d Ew;
  Ew = R.transpose() * (Ow - Rw);

  double thrust_force = f.dot(R.col(2));
  double thrust       = 0;

  if (thrust_force >= 0) {
    thrust = sqrt((thrust_force / 10.0) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Just so you know, the desired thrust force is negative (%.2f)", this->name_.c_str(), thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[%s]: NaN detected in variable \"thrust\", setting it to 0 and returning!!!", this->name_.c_str());

  } else if (thrust > thrust_saturation_) {

    thrust = thrust_saturation_;
    ROS_WARN_THROTTLE(1.0, "[%s]: saturating thrust to %.2f", this->name_.c_str(), thrust_saturation_);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[%s]: saturating thrust to %.2f", this->name_.c_str(), 0.0);
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  if (mute_lateral_gains && !reference->disable_position_gains) {
    mutex_lateral_gains_after_toggle = true;
  }
  mute_lateral_gains = reference->disable_position_gains;

  // --------------------------------------------------------------
  // |                  integrate the body error                 |
  // --------------------------------------------------------------

  /* body error integrator //{ */

  {
    std::scoped_lock lock(mutex_gains);

    // rotate the control errors to the body
    Eigen::Vector2d Ep_body = rotate2d(Ep.head(2), -uav_yaw);
    Eigen::Vector2d Ev_body = rotate2d(Ev.head(2), -uav_yaw);

    // integrate the body error

    // antiwindup
    double temp_gain = kibxy;
    if (rampup_active_ || sqrt(pow(uav_state->velocity.linear.x, 2) + pow(uav_state->velocity.linear.y, 2)) > 0.3) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for body integral kicks in", this->name_.c_str());
    }

    if (integral_terms_enabled) {
      if (reference->use_position_horizontal) {
        Ib_b -= temp_gain * Ep_body * dt;
      } else if (reference->use_velocity_horizontal) {
        Ib_b -= temp_gain * Ev_body * dt;
      }
    }

    // saturate the body
    double body_integral_saturated = false;
    if (!std::isfinite(Ib_b[0])) {
      Ib_b[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable \"Ib_b[0]\", setting it to 0!!!", this->name_.c_str());
    } else if (Ib_b[0] > kibxy_lim) {
      Ib_b[0]                 = kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b[0] < -kibxy_lim) {
      Ib_b[0]                 = -kibxy_lim;
      body_integral_saturated = true;
    }

    if (kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: MPC's's body pitch integral is being saturated!", this->name_.c_str());
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b[1])) {
      Ib_b[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable \"Ib_b[1]\", setting it to 0!!!", this->name_.c_str());
    } else if (Ib_b[1] > kibxy_lim) {
      Ib_b[1]                 = kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b[1] < -kibxy_lim) {
      Ib_b[1]                 = -kibxy_lim;
      body_integral_saturated = true;
    }

    if (kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: MPC's's body roll integral is being saturated!", this->name_.c_str());
    }
  }

  //}

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error

    // antiwindup
    double temp_gain = kiwxy;
    if (rampup_active_ || sqrt(pow(uav_state->velocity.linear.x, 2) + pow(uav_state->velocity.linear.y, 2)) > 0.3) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for world integral kicks in", this->name_.c_str());
    }

    if (integral_terms_enabled) {
      if (reference->use_position_horizontal) {
        Iw_w -= temp_gain * Ep.head(2) * dt;
      } else if (reference->use_velocity_horizontal) {
        Iw_w -= temp_gain * Ev.head(2) * dt;
      }
    }

    // saturate the world
    double world_integral_saturated = false;
    if (!std::isfinite(Iw_w[0])) {
      Iw_w[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable \"Iw_w[0]\", setting it to 0!!!", this->name_.c_str());
    } else if (Iw_w[0] > kiwxy_lim) {
      Iw_w[0]                  = kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w[0] < -kiwxy_lim) {
      Iw_w[0]                  = -kiwxy_lim;
      world_integral_saturated = true;
    }

    if (kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: SO3's world X integral is being saturated!", this->name_.c_str());
    }

    // saturate the world
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w[1])) {
      Iw_w[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in variable \"Iw_w[1]\", setting it to 0!!!", this->name_.c_str());
    } else if (Iw_w[1] > kiwxy_lim) {
      Iw_w[1]                  = kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w[1] < -kiwxy_lim) {
      Iw_w[1]                  = -kiwxy_lim;
      world_integral_saturated = true;
    }

    if (kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: SO3's world Y integral is being saturated!", this->name_.c_str());
    }
  }

  //}

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    // antiwindup
    double temp_gain = km;
    if (rampup_active_ ||
        (fabs(uav_state->velocity.linear.z) > 0.3 && ((Ep[2] < 0 && uav_state->velocity.linear.z > 0) || (Ep[2] > 0 && uav_state->velocity.linear.z < 0)))) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[%s]: anti-windup for the mass kicks in", this->name_.c_str());
    }

    if (reference->use_position_vertical) {
      uav_mass_difference -= temp_gain * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference)) {
      uav_mass_difference = 0;
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!", this->name_.c_str());
    } else if (uav_mass_difference > km_lim) {
      uav_mass_difference = km_lim;
      uav_mass_saturated  = true;
    } else if (uav_mass_difference < -km_lim) {
      uav_mass_difference = -km_lim;
      uav_mass_saturated  = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[%s]: The uav_mass_difference is being saturated to %0.2f!", this->name_.c_str(), uav_mass_difference);
    }
  }

  //}

  // --------------------------------------------------------------
  // |            report on the values of the integrals           |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_integrals_);

    ROS_INFO_THROTTLE(5.0, "[%s]: world error integral: x %.2f N, y %.2f N, lim: %.2f N", this->name_.c_str(), Iw_w[X], Iw_w[Y], kiwxy_lim);
    ROS_INFO_THROTTLE(5.0, "[%s]: body error integral:  x %.2f N, y %.2f N, lim: %.2f N", this->name_.c_str(), Ib_b[X], Ib_b[Y], kibxy_lim);
  }

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | -------------------- saturate yaw rate ------------------- |

  if (!std::isfinite(t[2])) {
    t[2] = 0;
    ROS_ERROR("[MpcController]: NaN detected in variable \"t[2]\", setting it to 0 and returning!!!");
  } else if (t[2] > yaw_rate_saturation_) {
    t[2] = yaw_rate_saturation_;
  } else if (t[2] < -yaw_rate_saturation_) {
    t[2] = -yaw_rate_saturation_;
  }

  {
    std::scoped_lock lock(mutex_output_mode);

    if (output_mode_ == OUTPUT_ATTITUDE_RATE) {

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

    } else if (output_mode_ == OUTPUT_ATTITUDE_QUATERNION) {

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

      ROS_WARN_THROTTLE(1.0, "[%s]: outputting attitude quaternion", this->name_.c_str());
    }

    output_command->desired_acceleration.x = f[0] / total_mass;
    output_command->desired_acceleration.y = f[1] / total_mass;
    output_command->desired_acceleration.z = f[2] / total_mass;
  }

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time).toSec()) >= rampup_duration_) {

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

  output_command->mass_difference = uav_mass_difference;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = Ib_b[0];
  output_command->disturbance_by_b = Ib_b[1];

  output_command->disturbance_bx_w = Ib_w[0];
  output_command->disturbance_by_w = Ib_w[1];

  output_command->disturbance_wx_w = Iw_w[0];
  output_command->disturbance_wy_w = Iw_w[1];

  // set the constraints
  output_command->controller_enforcing_constraints = true;

  output_command->horizontal_speed_constraint = 0.5 * max_speed_horizontal_;
  output_command->horizontal_acc_constraint   = 0.5 * max_acceleration_horizontal_;

  output_command->vertical_asc_speed_constraint = 0.5 * max_speed_vertical_;
  output_command->vertical_asc_acc_constraint   = 0.5 * max_acceleration_vertical_;

  output_command->vertical_desc_speed_constraint = 0.5 * max_speed_vertical_;
  output_command->vertical_desc_acc_constraint   = 0.5 * max_acceleration_vertical_;

  output_command->controller = this->name_;

  last_output_command = output_command;

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

  world_integrals.vector.x = Iw_w[0];
  world_integrals.vector.y = Iw_w[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(msg->header.frame_id, world_integrals);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w[0] = res.value().vector.x;
    Iw_w[1] = res.value().vector.y;
  } else {

    ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform world integral to the new frame", this->name_.c_str());

    std::scoped_lock lock(mutex_integrals_);

    Iw_w[0] = 0;
    Iw_w[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void MpcController::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w = Eigen::Vector2d::Zero(2);
  Ib_b = Eigen::Vector2d::Zero(2);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void MpcController::dynamicReconfigureCallback(mrs_controllers::mpc_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_desired_gains);

    drs_desired_gains = config;
  }

  ROS_INFO("[%s]: DRS updated gains", this->name_.c_str());
}

//}

/* //{ callbackSetIntegralTerms() */

bool MpcController::callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  integral_terms_enabled = req.data;

  char message[100];

  sprintf((char *)&message, "Integral terms %s", integral_terms_enabled ? "ENABLED" : "DISABLED");
  res.message = message;
  res.success = true;
  ROS_INFO("[%s]: %s", this->name_.c_str(), message);

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void MpcController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.05, event);

  double gain_coeff                = 1;
  mutex_lateral_gains_after_toggle = false;

  bool bypass_filter = mute_lateral_gains || mutex_lateral_gains_after_toggle;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains, mutex_desired_gains);

    kqxy      = calculateGainChange(kqxy, drs_desired_gains.kqxy, false, "kqxy");
    kqz       = calculateGainChange(kqz, drs_desired_gains.kqz, false, "kqz");
    kwxy      = calculateGainChange(kwxy, drs_desired_gains.kwxy, false, "kwxy");
    kwz       = calculateGainChange(kwz, drs_desired_gains.kwz, false, "kwz");
    km        = calculateGainChange(km, drs_desired_gains.km, false, "km");
    km_lim    = calculateGainChange(km_lim, drs_desired_gains.km_lim, false, "km_lim");
    kiwxy     = calculateGainChange(kiwxy, drs_desired_gains.kiwxy * gain_coeff, bypass_filter, "kiwxy");
    kibxy     = calculateGainChange(kibxy, drs_desired_gains.kibxy * gain_coeff, bypass_filter, "kibxy");
    kiwxy_lim = calculateGainChange(kiwxy_lim, drs_desired_gains.kiwxy_lim, false, "kiwxy_lim");
    kibxy_lim = calculateGainChange(kibxy_lim, drs_desired_gains.kibxy_lim, false, "kibxy_lim");
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
    ROS_INFO_THROTTLE(1.0, "[%s]: changing gain \"%s\" from %f to %f", this->name_.c_str(), name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* rotate2d() //{ */

Eigen::Vector2d MpcController::rotate2d(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace mpc_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::mpc_controller::MpcController, mrs_uav_manager::Controller)

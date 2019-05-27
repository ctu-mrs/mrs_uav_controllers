/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_msgs/ControllerStatus.h>
#include <mrs_uav_manager/Controller.h>

#include <mrs_controllers/mpc_gainsConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

//}

#define X 0
#define Y 1
#define Z 2

#define PI 3.141592653

namespace mrs_controllers
{

namespace mpc_controller
{

/* //{ class MpcController */

class MpcController : public mrs_uav_manager::Controller {

public:
  MpcController(void);

  void initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus::Ptr     getStatus();

  void dynamicReconfigureCallback(mrs_controllers::mpc_gainsConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  bool reset(void);

private:
  bool is_initialized = false;
  bool is_active      = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                      config_mutex_;
  typedef mrs_controllers::mpc_gainsConfig    Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>        reconfigure_server_;
  void                                        drs_callback(mrs_controllers::mpc_gainsConfig &config, uint32_t level);
  mrs_controllers::mpc_gainsConfig            drs_desired_gains;

private:
  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;
  double                       hover_thrust;

  double roll, pitch, yaw;

  double yaw_offset;

  // actual gains (used and already filtered)
  double kpz, kvz, kaz;
  double km, km_lim;
  double kqxy, kqz;  // attitude gains
  double kwxy, kwz;  // attitude rate gains

  // desired gains (set by DRS)
  std::mutex mutex_gains;
  std::mutex mutex_desired_gains;

  double max_tilt_angle_;

  mrs_msgs::AttitudeCommand::ConstPtr last_output_command;
  mrs_msgs::AttitudeCommand           activation_control_command_;

  ros::Time last_update;
  bool      first_iteration = true;

  bool   mute_lateral_gains               = false;
  bool   mutex_lateral_gains_after_toggle = false;
  double mute_coefficitent_;

  // --------------------------------------------------------------
  // |                       MPC controller                       |
  // --------------------------------------------------------------

private:
  int n;            // number of states
  int m;            // number of inputs
  int horizon_len;  // lenght of the prediction horizon
  int n_variables;  // number of variables in the optimization

  double dt;

  Eigen::MatrixXd A;               // system matrix
  Eigen::MatrixXd B;               // input matrix
  Eigen::MatrixXd U;               // matrix for reshaping inputs
  Eigen::MatrixXd A_roof;          // BIG main matrix
  Eigen::MatrixXd B_roof;          // BIG input matrix
  Eigen::MatrixXd B_roof_reduced;  // BIG input matrix reduced by U
  Eigen::MatrixXd Q;               // small penalization matrix for large error
  Eigen::MatrixXd P;               // small penalization of input actions
  Eigen::MatrixXd Q_roof;          // BIG matrix of coeficients of quadratic penalization for large error
  Eigen::MatrixXd P_roof;          // BIG matrix of coeficients of penalization of inputs
  Eigen::MatrixXd H_inv;           // inversion of the main matrix of the quadratic form
  Eigen::MatrixXd H;               // inversion of the main matrix of the quadratic form

  Eigen::MatrixXd X_0;
  Eigen::MatrixXd c;
  Eigen::MatrixXd u_cf;
  Eigen::MatrixXd u;
  Eigen::MatrixXd states;

  Eigen::MatrixXd mpc_reference;  // reference for the controller
  Eigen::MatrixXd x;              // current state of the uav

  Eigen::MatrixXd outputTrajectory;

private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;

private:
  ros::Timer timer_gain_filter;
  void       timerGainsFilter(const ros::TimerEvent &event);

  int    gains_filter_timer_rate_;
  double gains_filter_change_rate_;
  double gains_filter_min_change_rate_;

  double gains_filter_max_change_;  // calculated from change_rate_/timer_rate_;
  double gains_filter_min_change_;  // calculated from change_rate_/timer_rate_;

private:
  Eigen::Vector2d Ib_b;  // body error integral in the body frame
  Eigen::Vector2d Iw_w;  // world error integral in the world_frame
};

MpcController::MpcController(void) {
}

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void MpcController::initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params) {

  ros::NodeHandle nh_(parent_nh, "mpc_controller");

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;

  std::vector<double> UvaluesList;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "MpcController");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // | --------------------- mpc controller --------------------- |

  // load the dynamicall model parameters
  param_loader.load_param("mpc_model/number_of_states", n);
  param_loader.load_param("mpc_model/number_of_inputs", m);
  param_loader.load_param("mpc_model/dt", dt);

  param_loader.load_matrix_static("mpc_model/A", A, n, n);
  param_loader.load_matrix_static("mpc_model/B", B, n, m);

  param_loader.load_param("mpc_parameters/horizon_len", horizon_len);
  param_loader.load_param("mpc_parameters/number_of_variables", n_variables);
  param_loader.load_matrix_static("mpc_parameters/Q", Q, n, n);
  param_loader.load_matrix_static("mpc_parameters/P", P, m, m);
  param_loader.load_param("mpc_parameters/U", UvaluesList);

  // create the U matrix
  int tempIdx  = 0;
  int tempIdx2 = 0;
  U            = Eigen::MatrixXd::Zero(horizon_len * m, n_variables);

  for (unsigned long i = 0; i < UvaluesList.size(); i++) {
    for (int j = 0; j < UvaluesList[i]; j++) {

      U.block(tempIdx, tempIdx2, m, m) = Eigen::MatrixXd::Identity(m, m);
      tempIdx += m;
    }
    tempIdx2 += m;
  }


  // | ------------- height and attitude controller ------------- |

  // height gains
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/kp", kpz);
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/kv", kvz);
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/ka", kaz);

  // attitude gains
  param_loader.load_param("attitude_vertical_feedback/default_gains/horizontal/attitude/kq", kqxy);
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/attitude/kq", kqz);

  // attitude rate gains
  param_loader.load_param("attitude_vertical_feedback/default_gains/horizontal/attitude/kw", kwxy);
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/attitude/kw", kwz);

  // mass estimator
  param_loader.load_param("attitude_vertical_feedback/default_gains/weight_estimator/km", km);
  param_loader.load_param("attitude_vertical_feedback/default_gains/weight_estimator/km_lim", km_lim);

  // physical
  param_loader.load_param("uav_mass", uav_mass_);
  param_loader.load_param("g", g_);

  // gain filtering
  param_loader.load_param("attitude_vertical_feedback/gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("attitude_vertical_feedback/gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("attitude_vertical_feedback/gains_filter/min_change_rate", gains_filter_min_change_rate_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MpcController]: Could not load all parameters!");
    ros::shutdown();
  }

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180) * PI;
  yaw_offset      = (yaw_offset / 180.0) * PI;

  uav_mass_difference = 0;
  Iw_w                = Eigen::Vector2d::Zero(2);
  Ib_b                = Eigen::Vector2d::Zero(2);

  // --------------------------------------------------------------
  // |                 MPC parameters and matrices                |
  // --------------------------------------------------------------

  // prepare the MPC matrice

  // initialize the trajectory variables
  mpc_reference = Eigen::MatrixXd::Zero(n*horizon_len, 1);

  // prepare A_roof matrix
  // A_roof = [A;
  //           A^2;
  //           A^4;
  //           ...;
  //           A^n];

  A_roof = Eigen::MatrixXd::Zero(horizon_len * n, n);

  Eigen::MatrixXd tempMatrix;

  tempMatrix = Eigen::MatrixXd(n, n);  // for acumulating the powers of A

  tempMatrix = A;

  for (int i = 0; i < horizon_len; i++) {

    A_roof.middleRows(i * n, n) = tempMatrix;
    tempMatrix                  = tempMatrix * A;  // compute next power of A
  }

  // B_roof matrix
  //			% n = prediction horizon length
  //			% B_roof = [B1,        0,        0,   0;
  //         				  AB1,       B2,       0,   0;
  //         				  A^2B1,     AB2,      B2,  0;
  //         				  ...;
  //         				  A^(n-2)B1, A^(n-1)B2, ..., 0;
  //         				  A^(n-1)B1, A^(n-2)B2, ..., B2;

  B_roof = Eigen::MatrixXd::Zero(horizon_len * n, horizon_len * m);

  B_roof.block(0, 0, n, m)         = B;
  B_roof.block(1 * n, 0 * m, n, m) = A * B;
  B_roof.block(1 * n, 1 * m, n, m) = B;

  for (int i = 2; i < horizon_len; i++) {  // over rows of submatrices

    for (int j = 2; j < horizon_len; j++) {  // over cols of submatrices

      // replicate the previous line but shift it one block right
      B_roof.block(i * n, j * m, n, m) = B_roof.block((i - 1) * n, (j - 1) * m, n, m);
    }

    // create the first block of the new line
    B_roof.block(i * n, 0, n, m) = A * B_roof.block((i - 1) * n, 0, n, m);

    // create the second block of the new line
    B_roof.block(i * n, m, n, m) = A * B_roof.block((i - 1) * n, m, n, m);
  }

  // create the reduced version of B_roof
  B_roof_reduced = Eigen::MatrixXd::Zero(horizon_len * n, n_variables);
  B_roof_reduced = B_roof * U;

  // Q_roof matrix
  // n = number of system states
  // Q = n*n
  //     diagonal, penalizing control errors
  // Q_roof = [Q,   0,   ...,  0;
  //           0,   Q,   ...,  0;
  //           ..., ..., Q,    0;
  //           0,   ..., ...,  S];

  Q_roof = Eigen::MatrixXd::Zero(horizon_len * n, horizon_len * n);

  for (int i = 0; i < horizon_len; i++) {

    Q_roof.block(i * n, i * n, n, n) = Q;
  }

  // P_roof matrix
  // penalizing control actions
  // P_roof = [P,   0,   ...,  0;
  //           0,   P,   ...,  0;
  //           ..., ..., P,    0;
  //           0,   ..., ...,  P];

  P_roof = Eigen::MatrixXd::Zero(n_variables, n_variables);

  tempIdx = 0;
  for (unsigned long i = 0; i < UvaluesList.size(); i++) {

    P_roof.block(i * m, i * m, m, m) = P * UvaluesList[i];
  }

  // initialize other matrices
  x                = Eigen::MatrixXd::Zero(n, 1);
  X_0              = Eigen::MatrixXd::Zero(horizon_len * n, 1);
  c                = Eigen::MatrixXd::Zero(n_variables * m, 1);
  u_cf             = Eigen::MatrixXd::Zero(n_variables * m, 1);
  u                = Eigen::MatrixXd::Zero(horizon_len * m, 1);
  states           = Eigen::MatrixXd::Zero(horizon_len * n, 1);
  H_inv            = Eigen::MatrixXd::Zero(n_variables, n_variables);
  H                = Eigen::MatrixXd::Zero(n_variables, n_variables);
  outputTrajectory = Eigen::MatrixXd::Zero(horizon_len * n, 1);

  // calculate H
  H = B_roof_reduced.transpose()*Q_roof*B_roof_reduced + P_roof;

  // create the inversion of H matrix - the main matrix of the qudratic form
  H_inv = (0.5*H).inverse();

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_desired_gains.kpz    = kpz;
  drs_desired_gains.kvz    = kvz;
  drs_desired_gains.kaz    = kaz;
  drs_desired_gains.kqxy   = kqxy;
  drs_desired_gains.kqz    = kqz;
  drs_desired_gains.kwxy   = kwxy;
  drs_desired_gains.kwz    = kwz;
  drs_desired_gains.km     = km;
  drs_desired_gains.km_lim = km_lim;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_desired_gains);
  ReconfigureServer::CallbackType f = boost::bind(&MpcController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "MpcController", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &MpcController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[MpcController]: Could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("[MpcController]: initialized");

  is_initialized = true;
}

//}

/* //{ activate() */

bool MpcController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {
    activation_control_command_ = mrs_msgs::AttitudeCommand();
    uav_mass_difference         = 0;
    ROS_WARN("[MpcController]: activated without getting the last tracker's command.");
  } else {
    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;
    ROS_INFO("[MpcController]: activated with a last trackers command.");
  }

  first_iteration = true;

  ROS_INFO("[MpcController]: activated");

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void MpcController::deactivate(void) {

  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[MpcController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr MpcController::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

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

  Rp << 0, 0, reference->position.z;  // fill the desired position
  Rv << 0, 0, reference->velocity.z;
  Ra << 0, 0, reference->acceleration.z;
  Rw << 0, 0, reference->yaw_dot;

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(0, 0, odometry->pose.pose.position.z);
  Eigen::Vector3d Ov(0, 0, odometry->twist.twist.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(odometry->twist.twist.angular.x, odometry->twist.twist.angular.y, odometry->twist.twist.angular.z);

  // --------------------------------------------------------------
  // |                     MPC lateral control                    |
  // --------------------------------------------------------------

  // prepare the full reference vector
  for (int i = 0; i < horizon_len; i++) {

    mpc_reference(i * n, 0)     = reference->position.x;
    mpc_reference(i * n + 1, 0) = 0;
    mpc_reference(i * n + 2, 0) = 0;
    mpc_reference(i * n + 3, 0) = reference->position.y;
    mpc_reference(i * n + 4, 0) = 0;
    mpc_reference(i * n + 5, 0) = 0;
  }

  // set the initial condition
  x << odometry->pose.pose.position.x, odometry->twist.twist.linear.x, 0, odometry->pose.pose.position.y, odometry->twist.twist.linear.y, 0;

  // prepare the linear part of the qudratic function
  X_0 = (A_roof * x - mpc_reference).transpose();

  Eigen::MatrixXd temp(1, n * horizon_len);

  // do clever product of X_0 and Q_roof
  for (int i = 0; i < n * horizon_len; i++) {
    if (i == 0 || i == (n * horizon_len - 1))
      temp(0, i) = X_0(0, i) * Q_roof(i, i);
    else
      temp(0, i) = X_0(0, i - 1) * Q_roof(i, i - 1) + X_0(0, i) * Q_roof(i, i) + X_0(0, i + 1) * Q_roof(i, i + 1);
  }

  c = (temp * B_roof_reduced).transpose();

  // calculate the control actions
  u_cf = H_inv * (c * (-0.5));

  // stretch the control vector to its full length
  u = U * u_cf;

  /* ROS_INFO("[MpcController]: u[0] = %.2f, u[1] = %.2f", u(0), u(1)); */

  // simulate the whole trajectory
  states = A_roof * x + B_roof * u;

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  Eigen::Vector3d Ep = Op - Rp;
  Eigen::Vector3d Ev = Ov - Rv;

  /* calculate dt //{ */

  double dt;

  if (first_iteration) {

    reset();
    last_update = odometry->header.stamp;

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (odometry->header.stamp - last_update).toSec();
    last_update = odometry->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_WARN_STREAM_THROTTLE(1.0, "[MpcController]: last " << last_update << ", current " << odometry->header.stamp);
    ROS_WARN_THROTTLE(1.0, "[MpcController]: the last odometry message came too close! %f", dt);
    if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_output_command;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
    }
  }

  //}

  // --------------------------------------------------------------
  // |                            gains                           |
  // --------------------------------------------------------------
  //
  Eigen::Vector3d Ka;
  Eigen::Array3d  Kp, Kv, Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains);

    Kp << 0, 0, kpz;
    Kv << 0, 0, kvz;
    Ka << 0, 0, kaz;
    Kq << kqxy, kqxy, kqz;
    Kw << kwxy, kwxy, kwz;
  }

  // --------------------------------------------------------------
  // |          caculate the current oritentation angles          |
  // --------------------------------------------------------------

  double         yaw, pitch, roll;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry->pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

  // --------------------------------------------------------------
  // |         mpc acceleration -> controller feed forward        |
  // --------------------------------------------------------------

  // calculate the feed forwared acceleration
  Eigen::Vector3d mpc_feed_forward(asin((-u(1) * cos(pitch) * cos(roll)) / g_), asin((u(0) * cos(pitch) * cos(roll)) / g_), 0);
  
  // --------------------------------------------------------------
  // |                recalculate the hover thrust                |
  // --------------------------------------------------------------

  hover_thrust = sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

  // --------------------------------------------------------------
  // |                  feed forward from tracker                 |
  // --------------------------------------------------------------

  // calculate the feed forwared acceleration
  Eigen::Vector3d feed_forward(asin((-reference->acceleration.y * cos(pitch) * cos(roll)) / g_),
                               asin((reference->acceleration.x * cos(pitch) * cos(roll)) / g_), reference->acceleration.z * (hover_thrust / g_));

  // --------------------------------------------------------------
  // |                 desired orientation matrix                 |
  // --------------------------------------------------------------

  tf::Quaternion desired_orientation =
      tf::createQuaternionFromRPY(mpc_feed_forward[0] + feed_forward[0], mpc_feed_forward[1] + feed_forward[1], reference->yaw);

  Eigen::Vector3d f = -Kp * Ep.array() - Kv * Ev.array() + (uav_mass_ + uav_mass_difference) * (Eigen::Vector3d(0, 0, g_) + Ra).array();

  Rq.coeffs() << desired_orientation.getX(), desired_orientation.getY(), desired_orientation.getZ(), desired_orientation.getW();
  Rd = Rq.matrix();

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
    ROS_WARN_THROTTLE(1.0, "[So3Controller]: Just so you know, the desired thrust force is negative (%f", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    ROS_ERROR("NaN detected in variable \"thrust\", setting it to 0 and returning!!!");
  } else if (thrust > 0.8) {
    thrust = 0.8;
  } else if (thrust < 0.0) {
    thrust = 0.0;
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

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    uav_mass_difference -= km * Ep[2] * dt;

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference)) {
      uav_mass_difference = 0;
      ROS_WARN_THROTTLE(1.0, "[MpcController]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!");
    } else if (uav_mass_difference > km_lim) {
      uav_mass_difference = km_lim;
      uav_mass_saturated  = true;
    } else if (uav_mass_difference < -km_lim) {
      uav_mass_difference = -km_lim;
      uav_mass_saturated  = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[MpcController]: The uav_mass_difference is being saturated to %1.3f!", uav_mass_difference);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->attitude_rate.x   = 1 * t[0];
  output_command->attitude_rate.y   = -1 * t[1];
  output_command->attitude_rate.z   = -1 * t[2];
  output_command->attitude_rate_set = true;

  Eigen::Quaterniond thrust_vec       = Eigen::Quaterniond(Rd);
  output_command->quter_attitude.w    = thrust_vec.w();
  output_command->quter_attitude.x    = thrust_vec.x();
  output_command->quter_attitude.y    = thrust_vec.y();
  output_command->quter_attitude.z    = thrust_vec.z();
  output_command->quater_attitude_set = true;

  output_command->euler_attitude_set = false;

  output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  output_command->thrust = thrust;

  output_command->mass_difference = uav_mass_difference;

  last_output_command = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus::Ptr MpcController::getStatus() {

  if (is_initialized) {

    mrs_msgs::ControllerStatus::Ptr controller_status(new mrs_msgs::ControllerStatus);

    if (is_active) {
      controller_status->active = mrs_msgs::ControllerStatus::ACTIVE;
    } else {
      controller_status->active = mrs_msgs::ControllerStatus::NONACTIVE;
    }

    return controller_status;
  } else {

    return mrs_msgs::ControllerStatus::Ptr();
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void MpcController::dynamicReconfigureCallback(mrs_controllers::mpc_gainsConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_desired_gains);

    drs_desired_gains = config;
  }

  ROS_INFO("[MpcController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void MpcController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.01, event);

  double gain_coeff                = 1;
  mutex_lateral_gains_after_toggle = false;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains, mutex_desired_gains);

    kpz    = calculateGainChange(kpz, drs_desired_gains.kpz, false, "kpz");
    kvz    = calculateGainChange(kvz, drs_desired_gains.kvz, false, "kvz");
    kaz    = calculateGainChange(kaz, drs_desired_gains.kaz, false, "kaz");
    kqxy   = calculateGainChange(kqxy, drs_desired_gains.kqxy, false, "kqxy");
    kqz    = calculateGainChange(kqz, drs_desired_gains.kqz, false, "kqz");
    kwxy   = calculateGainChange(kwxy, drs_desired_gains.kwxy, false, "kwxy");
    kwz    = calculateGainChange(kwz, drs_desired_gains.kwz, false, "kwz");
    km     = calculateGainChange(km, drs_desired_gains.km, false, "km");
    km_lim = calculateGainChange(km_lim, drs_desired_gains.km_lim, false, "km_lim");
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
    ROS_INFO_THROTTLE(1.0, "[MpcController]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* reset() //{ */

bool MpcController::reset(void) {

  Iw_w = Eigen::Vector2d::Zero(2);

  return true;
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

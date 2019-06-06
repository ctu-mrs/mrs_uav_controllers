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

#include <cvx_wrapper.h>

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

  // actual gains (used and already filtered)
  double kpz, kvz, kaz, kiwxy, kibxy;
  double kiwxy_lim, kibxy_lim;
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
  int n;  // number of states

  double dt1, dt2;

  double cvx_x_u = 0;
  double cvx_y_u = 0;
  double cvx_z_u = 0;

  int horizon_length_;

  double max_speed_, max_acceleration_, max_jerk_;

  mrs_controllers::mpc_controller::CvxWrapper *cvx_x;
  mrs_controllers::mpc_controller::CvxWrapper *cvx_y;

  bool cvx_verbose_ = false;
  int  cvx_max_iterations_;

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

  param_loader.load_param("mpc_parameters/max_speed", max_speed_);
  param_loader.load_param("mpc_parameters/max_acceleration", max_acceleration_);
  param_loader.load_param("mpc_parameters/max_jerk", max_jerk_);

  std::vector<double> Q, S;
  param_loader.load_param("mpc_parameters/Q", Q);
  param_loader.load_param("mpc_parameters/S", S);

  param_loader.load_param("cvx_parameters/verbose", cvx_verbose_);
  param_loader.load_param("cvx_parameters/max_iterations", cvx_max_iterations_);

  // | --------------------- integral gains --------------------- |

  param_loader.load_param("integral_gains/kiw", kiwxy);
  param_loader.load_param("integral_gains/kib", kibxy);

  // integrator limits
  param_loader.load_param("integral_gains/kiw_lim", kiwxy_lim);
  param_loader.load_param("integral_gains/kib_lim", kibxy_lim);

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

  uav_mass_difference = 0;
  Iw_w                = Eigen::Vector2d::Zero(2);
  Ib_b                = Eigen::Vector2d::Zero(2);

  // --------------------------------------------------------------
  // |                       prepare cvxgen                       |
  // --------------------------------------------------------------

  cvx_x = new mrs_controllers::mpc_controller::CvxWrapper(cvx_verbose_, cvx_max_iterations_, Q, S, dt1, dt2);
  cvx_y = new mrs_controllers::mpc_controller::CvxWrapper(cvx_verbose_, cvx_max_iterations_, Q, S, dt1, dt2);

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

  Rp << reference->position.x, reference->position.y, reference->position.z;  // fill the desired position
  Rv << reference->velocity.x, reference->velocity.y, reference->velocity.z;
  Rw << 0, 0, reference->yaw_dot;

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z);
  Eigen::Vector3d Ov(odometry->twist.twist.linear.x, odometry->twist.twist.linear.y, odometry->twist.twist.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(odometry->twist.twist.angular.x, odometry->twist.twist.angular.y, odometry->twist.twist.angular.z);

  // --------------------------------------------------------------
  // |                     MPC lateral control                    |
  // --------------------------------------------------------------

  // | ------------------- initial conditions ------------------- |

  Eigen::MatrixXd initial_x = Eigen::MatrixXd::Zero(3, 1);
  initial_x << odometry->pose.pose.position.x, odometry->twist.twist.linear.x, reference->acceleration.x;

  Eigen::MatrixXd initial_y = Eigen::MatrixXd::Zero(3, 1);
  initial_y << odometry->pose.pose.position.y, odometry->twist.twist.linear.y, reference->acceleration.y;

  // | ---------------------- set reference --------------------- |

  Eigen::MatrixXd mpc_reference_x = Eigen::MatrixXd::Zero(horizon_length_ * n, 1);
  Eigen::MatrixXd mpc_reference_y = Eigen::MatrixXd::Zero(horizon_length_ * n, 1);

  // prepare the full reference vector
  for (int i = 0; i < horizon_length_; i++) {

    mpc_reference_x((i * n) + 0, 0) = reference->position.x;
    mpc_reference_x((i * n) + 1, 0) = reference->velocity.x;
    mpc_reference_x((i * n) + 2, 0) = reference->acceleration.x;

    mpc_reference_y((i * n) + 0, 0) = reference->position.y;
    mpc_reference_y((i * n) + 1, 0) = reference->velocity.y;
    mpc_reference_y((i * n) + 2, 0) = reference->acceleration.y;
  }

  // | ------------------------ optimize ------------------------ |

  cvx_x->setLastInput(cvx_x_u);
  cvx_x->loadReference(mpc_reference_x);
  cvx_x->setLimits(max_speed_, max_acceleration_, max_jerk_, dt1, dt2);
  cvx_x->setInitialState(initial_x);
  [[maybe_unused]] int iters_x = cvx_x->solveCvx();
  cvx_x_u                      = cvx_x->getFirstControlInput();

  cvx_y->setLastInput(cvx_y_u);
  cvx_y->loadReference(mpc_reference_y);
  cvx_y->setLimits(max_speed_, max_acceleration_, max_jerk_, dt1, dt2);
  cvx_y->setInitialState(initial_y);
  [[maybe_unused]] int iters_y = cvx_y->solveCvx();
  cvx_y_u                      = cvx_y->getFirstControlInput();

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

  // --------------------------------------------------------------
  // |          caculate the current oritentation angles          |
  // --------------------------------------------------------------

  double         yaw, pitch, roll;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry->pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

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
  // |                recalculate the hover thrust                |
  // --------------------------------------------------------------

  hover_thrust = sqrt((uav_mass_ + uav_mass_difference) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;

  // --------------------------------------------------------------
  // |                 desired orientation matrix                 |
  // --------------------------------------------------------------

  Eigen::Vector2d Ib_w = rotate2d(Ib_b, -yaw);

  Eigen::Vector3d Ip(Ib_w[0] + Iw_w[0], Ib_w[1] + Iw_w[1], 0);

  Ra << reference->acceleration.x + cvx_x_u, reference->acceleration.y + cvx_y_u, reference->acceleration.z;
  /* Ra << cvx_x_u, cvx_y_u, reference->acceleration.z; */

  Eigen::Vector3d f      = -Kp * Ep.array() - Kv * Ev.array() + Ip.array() + (uav_mass_ + uav_mass_difference) * (Eigen::Vector3d(0, 0, g_) + Ra).array();
  Eigen::Vector3d f_norm = f.normalized();

  // | ---------------------- yaw reference --------------------- |

  Rq.coeffs() << 0, 0, sin(reference->yaw / 2.0), cos(reference->yaw / 2.0);

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
  /* double thrust_force = Ft; */
  double thrust = 0;

  if (thrust_force >= 0) {
    thrust = sqrt((thrust_force / 10.0) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
  } else {
    ROS_WARN_THROTTLE(1.0, "[MpcController]: Just so you know, the desired thrust force is negative (%f", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    ROS_ERROR("[MpcController]: NaN detected in variable \"thrust\", setting it to 0 and returning!!!");
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

  // --------------------------------------------------------------
  // |                  integrate the body error                 |
  // --------------------------------------------------------------

  /* body error integrator //{ */

  {
    std::scoped_lock lock(mutex_gains);

    // rotate the control errors to the body
    Eigen::Vector2d Ep_body = rotate2d(Ep.head(2), yaw);

    // integrate the body error

    // antiwindup
    double temp_gain = kibxy;
    if (sqrt(pow(odometry->twist.twist.linear.x, 2) + pow(odometry->twist.twist.linear.y, 2)) > 0.3) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[MpcController]: anti-windup for body integral kicks in");
    }
    Ib_b -= temp_gain * Ep_body * dt;

    // saturate the body
    double body_integral_saturated = false;
    if (!std::isfinite(Ib_b[0])) {
      Ib_b[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[MpcController]: NaN detected in variable \"Ib_b[0]\", setting it to 0!!!");
    } else if (Ib_b[0] > kibxy_lim) {
      Ib_b[0]                 = kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b[0] < -kibxy_lim) {
      Ib_b[0]                 = -kibxy_lim;
      body_integral_saturated = true;
    }

    if (kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[MpcController]: MPC's's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b[1])) {
      Ib_b[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[MpcController]: NaN detected in variable \"Ib_b[1]\", setting it to 0!!!");
    } else if (Ib_b[1] > kibxy_lim) {
      Ib_b[1]                 = kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b[1] < -kibxy_lim) {
      Ib_b[1]                 = -kibxy_lim;
      body_integral_saturated = true;
    }

    if (kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[MpcController]: MPC's's body roll integral is being saturated!");
    }
  }

  //}

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error

    // antiwindup
    double temp_gain = kiwxy;
    if (sqrt(pow(odometry->twist.twist.linear.x, 2) + pow(odometry->twist.twist.linear.y, 2)) > 0.3) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[MpcController]: anti-windup for world integral kicks in");
    }
    Iw_w -= temp_gain * Ep.head(2) * dt;

    // saturate the world
    double world_integral_saturated = false;
    if (!std::isfinite(Iw_w[0])) {
      Iw_w[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[MpcController]: NaN detected in variable \"Iw_w[0]\", setting it to 0!!!");
    } else if (Iw_w[0] > kiwxy_lim) {
      Iw_w[0]                  = kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w[0] < -kiwxy_lim) {
      Iw_w[0]                  = -kiwxy_lim;
      world_integral_saturated = true;
    }

    if (kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[MpcController]: SO3's world X integral is being saturated!");
    }

    // saturate the world
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w[1])) {
      Iw_w[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[MpcController]: NaN detected in variable \"Iw_w[1]\", setting it to 0!!!");
    } else if (Iw_w[1] > kiwxy_lim) {
      Iw_w[1]                  = kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w[1] < -kiwxy_lim) {
      Iw_w[1]                  = -kiwxy_lim;
      world_integral_saturated = true;
    }

    if (kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[MpcController]: SO3's world Y integral is being saturated!");
    }
  }

  //}

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
  // |            report on the values of the integrals           |
  // --------------------------------------------------------------

  ROS_INFO_THROTTLE(5.0, "[MpcController]: world error integral: x %.2f, y %.2f, lim: %.2f", Iw_w[X], Iw_w[Y], kiwxy_lim);
  ROS_INFO_THROTTLE(5.0, "[MpcController]:  body error integral: x %.2f, y %.2f, lim: %.2f", Ib_b[X], Ib_b[Y], kibxy_lim);

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->attitude_rate.x   = t[0];
  output_command->attitude_rate.y   = t[1];
  output_command->attitude_rate.z   = t[2];
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
  Ib_b = Eigen::Vector2d::Zero(2);

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

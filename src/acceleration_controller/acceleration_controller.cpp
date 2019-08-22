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

#include <mrs_controllers/acceleration_controllerConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

#include <mrs_controllers/cvx_wrapper.h>

//}

#define X 0
#define Y 1
#define Z 2

#define OUTPUT_ATTITUDE_RATE 1
#define OUTPUT_ATTITUDE_QUATERNION 2

namespace mrs_controllers
{

namespace acceleration_controller
{

/* //{ class AccelerationController */

class AccelerationController : public mrs_uav_manager::Controller {

public:
  void initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus::Ptr     getStatus();

  void dynamicReconfigureCallback(mrs_controllers::acceleration_controllerConfig &config, uint32_t level);

  double calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name);

  virtual void switchOdometrySource(const nav_msgs::Odometry::ConstPtr &msg);

  Eigen::Vector2d rotate2d(const Eigen::Vector2d vector_in, double angle);

  void resetDisturbanceEstimators(void);

private:
  bool is_initialized = false;
  bool is_active      = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                                 config_mutex_;
  typedef mrs_controllers::acceleration_controllerConfig Config;
  typedef dynamic_reconfigure::Server<Config>            ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                   reconfigure_server_;
  void                                                   drs_callback(mrs_controllers::acceleration_controllerConfig &config, uint32_t level);
  mrs_controllers::acceleration_controllerConfig         drs_desired_gains;

private:
  double                       uav_mass_;
  double                       uav_mass_difference;
  double                       g_;
  mrs_uav_manager::MotorParams motor_params_;
  double                       hover_thrust;

  // actual gains (used and already filtered)
  double km, km_lim;
  double kqxy, kqz;  // attitude gains
  double kwxy, kwz;  // attitude rate gains

  // desired gains (set by DRS)
  std::mutex mutex_gains;
  std::mutex mutex_desired_gains;

  double tilt_angle_saturation_;
  double tilt_angle_failsafe_;
  double thrust_saturation_;

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

  double cvx_z_u = 0;

  int horizon_length_;

  double max_speed_vertical_, max_acceleration_vertical_, max_u_vertical_;

  mrs_controllers::cvx_wrapper::CvxWrapper *cvx_z;

  bool cvx_verbose_ = false;
  int  cvx_max_iterations_;

  std::vector<double> Q_z, S_z;

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
  int        output_mode_;  // 1 = ATTITUDE RATES, 2 = ATTITUDE QUATERNION
  std::mutex mutex_output_mode;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void AccelerationController::initialize(const ros::NodeHandle &parent_nh, mrs_uav_manager::MotorParams motor_params) {

  ros::NodeHandle nh_(parent_nh, "acceleration_controller");

  ros::Time::waitForValid();

  this->motor_params_ = motor_params;

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "AccelerationController");

  param_loader.load_param("enable_profiler", profiler_enabled_);

  // | --------------------- mpc controller --------------------- |

  // load the dynamicall model parameters
  param_loader.load_param("mpc_model/number_of_states", n);
  param_loader.load_param("mpc_model/dt1", dt1);
  param_loader.load_param("mpc_model/dt2", dt2);

  param_loader.load_param("mpc_parameters/horizon_length", horizon_length_);

  param_loader.load_param("mpc_parameters/vertical/max_speed", max_speed_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_acceleration", max_acceleration_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_u", max_u_vertical_);

  param_loader.load_param("mpc_parameters/vertical/Q", Q_z);
  param_loader.load_param("mpc_parameters/vertical/S", S_z);

  param_loader.load_param("cvx_parameters/verbose", cvx_verbose_);
  param_loader.load_param("cvx_parameters/max_iterations", cvx_max_iterations_);

  // | ------------- height and attitude controller ------------- |

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

  // constraints
  param_loader.load_param("attitude_vertical_feedback/tilt_angle_saturation", tilt_angle_saturation_);
  param_loader.load_param("attitude_vertical_feedback/tilt_angle_failsafe", tilt_angle_failsafe_);
  param_loader.load_param("attitude_vertical_feedback/thrust_saturation", thrust_saturation_);

  // gain filtering
  param_loader.load_param("attitude_vertical_feedback/gains_filter/filter_rate", gains_filter_timer_rate_);
  param_loader.load_param("attitude_vertical_feedback/gains_filter/perc_change_rate", gains_filter_change_rate_);
  param_loader.load_param("attitude_vertical_feedback/gains_filter/min_change_rate", gains_filter_min_change_rate_);

  gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
  gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

  // output mode
  param_loader.load_param("output_mode", output_mode_);

  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[AccelerationController]: output mode has to be {1, 2}!");
  }

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AccelerationController]: Could not load all parameters!");
    ros::shutdown();
  }

  // convert to radians
  max_tilt_angle_ = (max_tilt_angle_ / 180) * M_PI;

  uav_mass_difference = 0;

  // --------------------------------------------------------------
  // |                       prepare cvxgen                       |
  // --------------------------------------------------------------

  cvx_z = new mrs_controllers::cvx_wrapper::CvxWrapper(cvx_verbose_, cvx_max_iterations_, Q_z, S_z, dt1, dt2, 0.5, 0.5);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  drs_desired_gains.kqxy   = kqxy;
  drs_desired_gains.kqz    = kqz;
  drs_desired_gains.kwxy   = kwxy;
  drs_desired_gains.kwz    = kwz;
  drs_desired_gains.km     = km;
  drs_desired_gains.km_lim = km_lim;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_desired_gains);
  ReconfigureServer::CallbackType f = boost::bind(&AccelerationController::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "AccelerationController", profiler_enabled_);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &AccelerationController::timerGainsFilter, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AccelerationController]: Could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("[AccelerationController]: initialized");

  is_initialized = true;
}

//}

/* //{ activate() */

bool AccelerationController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    activation_control_command_ = mrs_msgs::AttitudeCommand();
    uav_mass_difference         = 0;

    ROS_WARN("[AccelerationController]: activated without getting the last tracker's command.");

  } else {

    activation_control_command_ = *cmd;
    uav_mass_difference         = cmd->mass_difference;

    ROS_INFO("[AttitudeController]: setting mass difference from the last AttitudeCmd: %.2f kg", uav_mass_difference);

    ROS_INFO("[AccelerationController]: activated with the last tracker's command.");
  }

  first_iteration = true;

  ROS_INFO("[AccelerationController]: activated");

  is_active = true;

  return true;
}

//}

/* //{ deactivate() */

void AccelerationController::deactivate(void) {

  is_active           = false;
  first_iteration     = false;
  uav_mass_difference = 0;

  ROS_INFO("[AccelerationController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr AccelerationController::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                         const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

  if (!is_active) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rc - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference
  Eigen::Vector3d           Rp, Rv, Ra, Rw;
  Eigen::Quaternion<double> Rq;

  Eigen::Matrix3d Rd;

  Rp << reference->position.x, reference->position.y, reference->position.z;
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

  Eigen::MatrixXd initial_z = Eigen::MatrixXd::Zero(3, 1);
  initial_z << odometry->pose.pose.position.z, odometry->twist.twist.linear.z, reference->acceleration.z;

  // | ---------------------- set reference --------------------- |

  Eigen::MatrixXd mpc_reference_z = Eigen::MatrixXd::Zero(horizon_length_ * n, 1);

  // prepare the full reference vector
  if (reference->use_position) {
    for (int i = 0; i < horizon_length_; i++) {
      mpc_reference_z((i * n) + 0, 0) = reference->position.z;
    }
  }

  if (reference->use_velocity) {
    for (int i = 0; i < horizon_length_; i++) {
      mpc_reference_z((i * n) + 1, 0) = reference->velocity.z;
    }
  }

  if (reference->use_acceleration) {
    for (int i = 0; i < horizon_length_; i++) {
      mpc_reference_z((i * n) + 2, 0) = reference->acceleration.z;
    }
  }

  // | ------------------------ optimize ------------------------ |

  // update Q and S based on the reference type
  std::vector<double> temp_Q = Q_z;
  std::vector<double> temp_S = S_z;

  if (!reference->use_position) {
    temp_Q[0] = 0;
    temp_S[0] = 0;
  }

  if (!reference->use_velocity) {
    temp_Q[1] = 0;
    temp_S[1] = 0;
  }

  cvx_z->lock();
  cvx_z->setQ(temp_Q);
  cvx_z->setS(temp_S);
  cvx_z->setParams();
  cvx_z->setLastInput(cvx_z_u);
  cvx_z->loadReference(mpc_reference_z);
  cvx_z->setLimits(max_speed_vertical_, max_acceleration_vertical_, max_u_vertical_, 999.0, dt1, dt2);
  cvx_z->setInitialState(initial_z);
  [[maybe_unused]] int iters_z = cvx_z->solveCvx();
  cvx_z_u                      = cvx_z->getFirstControlInput();
  cvx_z->unlock();

  // --------------------------------------------------------------
  // |                  calculate control errors                  |
  // --------------------------------------------------------------

  Eigen::Vector3d Ep = Op - Rp;

  // --------------------------------------------------------------
  // |                      calculate the dt                      |
  // --------------------------------------------------------------

  double dt;

  if (first_iteration) {

    last_update = odometry->header.stamp;

    first_iteration = false;

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));

  } else {

    dt          = (odometry->header.stamp - last_update).toSec();
    last_update = odometry->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_WARN_STREAM_THROTTLE(1.0, "[AccelerationController]: last " << last_update << ", current " << odometry->header.stamp);
    ROS_WARN_THROTTLE(1.0, "[AccelerationController]: the last odometry message came too close! %f", dt);
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
  Eigen::Array3d  Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains);

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

  Ra << reference->acceleration.x, reference->acceleration.y, reference->acceleration.z + cvx_z_u;

  double total_mass = uav_mass_ + uav_mass_difference;

  Eigen::Vector3d feed_forward = total_mass * (Eigen::Vector3d(0, 0, g_) + Ra);

  Eigen::Vector3d f = feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // "safe while true"
  for (int i = 0; i < 20; i++) {

    // if the downwards part of the force is close to counter-act the gravity acceleration
    if (f[2] < (0.15 * total_mass * g_)) {

      ROS_ERROR("[AccelerationController]: the calculated downwards desired force is negative (%.2f) -> mitigating the flip (iteration #%d).", f[2], i);

      // half the feedbacks
      feed_forward /= 2.0;

      // recalculate the desired force vector
      f = feed_forward;
    } else {

      break;
    }
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in the spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[AccelerationController]: NaN detected in variable \"theta\", returning null");

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (tilt_angle_failsafe_ > 1e-3 && theta > tilt_angle_failsafe_) {

    ROS_ERROR("[AccelerationController]: The produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null",
              (180.0 / M_PI) * theta, (180.0 / M_PI) * tilt_angle_failsafe_);
    ROS_INFO("[AccelerationController]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
    ROS_INFO("[AccelerationController]: feed forward: [%.2f, %.2f, %.2f]", feed_forward[0], feed_forward[1], feed_forward[2]);
    ROS_INFO("[AccelerationController]: position_cmd: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", reference->position.x, reference->position.y,
             reference->position.z, reference->yaw);
    ROS_INFO("[AccelerationController]: odometry: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", odometry->pose.pose.position.x, odometry->pose.pose.position.y,
             odometry->pose.pose.position.z, yaw);

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle
  if (tilt_angle_saturation_ > 1e-3 && theta > tilt_angle_saturation_) {
    ROS_WARN_THROTTLE(1.0, "[AccelerationController]: tilt is being saturated, desired: %f deg, saturated %f deg", (theta / M_PI) * 180.0,
                      (tilt_angle_saturation_ / M_PI) * 180.0);
    theta = tilt_angle_saturation_;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

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
    ROS_WARN_THROTTLE(1.0, "[AccelerationController]: Just so you know, the desired thrust force is negative (%f", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    ROS_ERROR("[AccelerationController]: NaN detected in variable \"thrust\", setting it to 0 and returning!!!");
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

    // antiwindup
    double temp_gain = km;
    if (fabs(odometry->twist.twist.linear.z) > 0.3 &&
        ((Ep[2] < 0 && odometry->twist.twist.linear.z > 0) || (Ep[2] > 0 && odometry->twist.twist.linear.z < 0))) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[AccelerationController]: anti-windup for the mass kicks in");
    }
    uav_mass_difference -= temp_gain * Ep[2] * dt;

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference)) {
      uav_mass_difference = 0;
      ROS_WARN_THROTTLE(1.0, "[AccelerationController]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!");
    } else if (uav_mass_difference > km_lim) {
      uav_mass_difference = km_lim;
      uav_mass_saturated  = true;
    } else if (uav_mass_difference < -km_lim) {
      uav_mass_difference = -km_lim;
      uav_mass_saturated  = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[AccelerationController]: The uav_mass_difference is being saturated to %0.2f!", uav_mass_difference);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  {
    std::scoped_lock lock(mutex_output_mode);

    if (output_mode_ == OUTPUT_ATTITUDE_RATE) {

      // output the desired attitude rate
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

    } else if (output_mode_ == OUTPUT_ATTITUDE_QUATERNION) {

      // output the desired attitude
      Eigen::Quaterniond thrust_vec       = Eigen::Quaterniond(Rd);
      output_command->quter_attitude.w    = thrust_vec.w();
      output_command->quter_attitude.x    = thrust_vec.x();
      output_command->quter_attitude.y    = thrust_vec.y();
      output_command->quter_attitude.z    = thrust_vec.z();
      output_command->quater_attitude_set = true;

      output_command->euler_attitude_set = false;
      output_command->attitude_rate_set  = false;

      output_command->mode_mask = output_command->MODE_QUATER_ATTITUDE;

      ROS_WARN_THROTTLE(1.0, "[AccelerationController]: outputting attitude quaternion");
    }

    output_command->desired_acceleration.x = f[0] / total_mass;
    output_command->desired_acceleration.y = f[1] / total_mass;
    output_command->desired_acceleration.z = f[2] / total_mass;
  }

  output_command->thrust          = thrust;
  output_command->mass_difference = uav_mass_difference;

  last_output_command = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus::Ptr AccelerationController::getStatus() {

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

/* switchOdometrySource() //{ */

void AccelerationController::switchOdometrySource([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg) {
}

//}

/* resetDisturbanceEstimators() //{ */

void AccelerationController::resetDisturbanceEstimators(void) {
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ dynamicReconfigureCallback() */

void AccelerationController::dynamicReconfigureCallback(mrs_controllers::acceleration_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_desired_gains);

    drs_desired_gains = config;
  }

  ROS_INFO("[AccelerationController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGainFilter() //{ */

void AccelerationController::timerGainsFilter(const ros::TimerEvent &event) {

  mrs_lib::Routine profiler_routine = profiler->createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.01, event);

  double gain_coeff                = 1;
  mutex_lateral_gains_after_toggle = false;

  if (mute_lateral_gains) {
    gain_coeff = mute_coefficitent_;
  }

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains, mutex_desired_gains);

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

double AccelerationController::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

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
    ROS_INFO_THROTTLE(1.0, "[AccelerationController]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
  }

  return current_value + change;
}

//}

/* rotate2d() //{ */

Eigen::Vector2d AccelerationController::rotate2d(const Eigen::Vector2d vector_in, double angle) {

  Eigen::Rotation2D<double> rot2(angle);

  return rot2.toRotationMatrix() * vector_in;
}

//}

}  // namespace acceleration_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::acceleration_controller::AccelerationController, mrs_uav_manager::Controller)

#define VERSION "0.0.5.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

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
  void initialize(const ros::NodeHandle &parent_nh, std::string name, std::string name_space, const mrs_uav_manager::MotorParams motor_params,
                  const double uav_mass, const double g, std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &reference);
  const mrs_msgs::ControllerStatus          getStatus();

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool &updated);

  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &msg);

  void resetDisturbanceEstimators(void);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                 mutex_drs_;
  typedef mrs_controllers::acceleration_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>       Drs_t;
  boost::shared_ptr<Drs_t>                               drs_;
  void                                                   callbackDrs(mrs_controllers::acceleration_controllerConfig &config, uint32_t level);
  DrsConfig_t                                            drs_gains_;

  // | ---------- thrust generation and mass estimation --------- |

  double                       _uav_mass_;
  double                       uav_mass_difference_;
  double                       _g_;
  mrs_uav_manager::MotorParams _motor_params_;

  // | ------------------- configurable gains ------------------- |

  // gains that are used and already filtered
  double km_;      // mass integrator gain
  double km_lim_;  // mass integrator limit
  double kqxy_;    // attiude pitch/roll gain
  double kqz_;     // attitude yaw gain
  double kwxy_;    // attitude tilt rate pitch/roll gain
  double kwz_;     // attitude yaw rate gain

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  // | --------------------- gain filtering --------------------- |

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  void filterGains(const bool mute_gains, const double dt);

  // | ----------------------- gain muting ---------------------- |

  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;

  // | ------------ controller limits and saturations ----------- |

  double _tilt_angle_saturation_;
  double _tilt_angle_failsafe_;
  double _thrust_saturation_;
  double _max_tilt_angle_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  int _output_mode_;  // 1 = ATTITUDE RATES, 2 = ATTITUDE QUATERNION

  // | --------------------- MPC controller --------------------- |

  // number of states
  int _n_states_;

  // time steps
  double _dt1_;  // the first time step
  double _dt2_;  // all the other steps

  // the last control input
  double cvx_z_u_ = 0;

  int _horizon_length_;

  // constraints
  double _max_speed_vertical_;
  double _max_acceleration_vertical_;
  double _max_u_vertical_;

  // CVXGen handler
  std::unique_ptr<mrs_controllers::cvx_wrapper::CvxWrapper> cvx_z_;

  // CVXGen params
  bool _cvx_verbose_ = false;
  int  _cvx_max_iterations_;

  // Q and S matrix diagonals
  std::vector<double> _Q_z_, _S_z_;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler;
  bool              _profiler_enabled_ = false;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void AccelerationController::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] std::string name, std::string name_space,
                                        const mrs_uav_manager::MotorParams motor_params, const double uav_mass, const double g,
                                        std::shared_ptr<mrs_uav_manager::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _motor_params_   = motor_params;
  _uav_mass_       = uav_mass;
  _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "AccelerationController");

  param_loader.load_param("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[AccelerationController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.load_param("enable_profiler", _profiler_enabled_);

  // | --------------------- mpc controller --------------------- |

  // load the dynamicall model parameters
  param_loader.load_param("mpc_model/number_of_states", _n_states_);
  param_loader.load_param("mpc_model/dt1", _dt1_);
  param_loader.load_param("mpc_model/dt2", _dt2_);

  param_loader.load_param("mpc_parameters/horizon_length", _horizon_length_);

  param_loader.load_param("mpc_parameters/vertical/max_speed", _max_speed_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_acceleration", _max_acceleration_vertical_);
  param_loader.load_param("mpc_parameters/vertical/max_u", _max_u_vertical_);

  param_loader.load_param("mpc_parameters/vertical/Q", _Q_z_);
  param_loader.load_param("mpc_parameters/vertical/S", _S_z_);

  param_loader.load_param("cvx_parameters/verbose", _cvx_verbose_);
  param_loader.load_param("cvx_parameters/max_iterations", _cvx_max_iterations_);

  // | ------------- height and attitude controller ------------- |

  // attitude gains
  param_loader.load_param("attitude_vertical_feedback/default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/attitude/kq", kqz_);

  // attitude rate gains
  param_loader.load_param("attitude_vertical_feedback/default_gains/horizontal/attitude/kw", kwxy_);
  param_loader.load_param("attitude_vertical_feedback/default_gains/vertical/attitude/kw", kwz_);

  // mass estimator
  param_loader.load_param("attitude_vertical_feedback/default_gains/weight_estimator/km", km_);
  param_loader.load_param("attitude_vertical_feedback/default_gains/weight_estimator/km_lim", km_lim_);

  // constraints
  param_loader.load_param("attitude_vertical_feedback/tilt_angle_saturation", _tilt_angle_saturation_);
  param_loader.load_param("attitude_vertical_feedback/tilt_angle_failsafe", _tilt_angle_failsafe_);
  param_loader.load_param("attitude_vertical_feedback/thrust_saturation", _thrust_saturation_);

  // gain filtering
  param_loader.load_param("attitude_vertical_feedback/gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.load_param("attitude_vertical_feedback/gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // output mode
  param_loader.load_param("output_mode", _output_mode_);

  // gain muting
  param_loader.load_param("gain_mute_coefficient", _gain_mute_coefficient_);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[AccelerationController]: could not load all parameters!");
    ros::shutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(_output_mode_ == OUTPUT_ATTITUDE_RATE || _output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[AccelerationController]: output mode has to be {1, 2}!");
    ros::shutdown();
  }

  // convert to radians
  _max_tilt_angle_ = (_max_tilt_angle_ / 180.0) * M_PI;

  uav_mass_difference_ = 0;

  // | ------------------- prepare the CVXGen ------------------- |

  cvx_z_ = std::make_unique<mrs_controllers::cvx_wrapper::CvxWrapper>(
      mrs_controllers::cvx_wrapper::CvxWrapper(_cvx_verbose_, _cvx_max_iterations_, _Q_z_, _S_z_, _dt1_, _dt2_, 0.5, 0.5));

  // | --------------- dynamic reconfigure server --------------- |

  drs_gains_.kqxy   = kqxy_;
  drs_gains_.kqz    = kqz_;
  drs_gains_.kwxy   = kwxy_;
  drs_gains_.kwz    = kwz_;
  drs_gains_.km     = km_;
  drs_gains_.km_lim = km_lim_;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_gains_);
  Drs_t::CallbackType f = boost::bind(&AccelerationController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------ profiler ------------------------ |

  profiler = mrs_lib::Profiler(nh_, "AccelerationController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[AccelerationController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool AccelerationController::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

  if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[AccelerationController]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *cmd;
    uav_mass_difference_     = cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    ROS_INFO("[AttitudeController]: setting mass difference from the last AttitudeCmd: %.2f kg", uav_mass_difference_);

    ROS_INFO("[AccelerationController]: activated with the last controller's command");
  }

  first_iteration_ = true;
  gains_muted_     = true;

  ROS_INFO("[AccelerationController]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void AccelerationController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[AccelerationController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr AccelerationController::update(const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                         const mrs_msgs::PositionCommand::ConstPtr &reference) {

  mrs_lib::Routine profiler_routine = profiler.createRoutine("update");

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

    ROS_DEBUG("[AccelerationController]: the last odometry message came too close (%.2f s)!", dt);

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
  Eigen::Vector3d Op(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // Oq - UAV attitude quaternion
  Eigen::Quaternion<double> Oq;
  Oq.coeffs() << uav_state->pose.orientation.x, uav_state->pose.orientation.y, uav_state->pose.orientation.z, uav_state->pose.orientation.w;
  Eigen::Matrix3d R = Oq.toRotationMatrix();

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // | --------------- calculate the control error -------------- |

  Eigen::Vector3d Ep = Op - Rp;

  // --------------------------------------------------------------
  // |                    MPC vertical control                    |
  // --------------------------------------------------------------

  // | ------------------- initial conditions ------------------- |

  Eigen::MatrixXd initial_z = Eigen::MatrixXd::Zero(3, 1);
  initial_z << uav_state->pose.position.z, uav_state->velocity.linear.z, reference->acceleration.z;

  // | ---------------------- set reference --------------------- |

  Eigen::MatrixXd mpc_reference_z = Eigen::MatrixXd::Zero(_horizon_length_ * _n_states_, 1);

  // prepare the full reference vector
  if (reference->use_position_vertical) {
    for (int i = 0; i < _horizon_length_; i++) {
      mpc_reference_z((i * _n_states_) + 0, 0) = reference->position.z;
    }
  }

  if (reference->use_velocity_vertical) {
    for (int i = 0; i < _horizon_length_; i++) {
      mpc_reference_z((i * _n_states_) + 1, 0) = reference->velocity.z;
    }
  }

  if (reference->use_acceleration) {
    for (int i = 0; i < _horizon_length_; i++) {
      mpc_reference_z((i * _n_states_) + 2, 0) = reference->acceleration.z;
    }
  }

  // | ------------------------ optimize ------------------------ |

  // update Q and S based on the reference type
  std::vector<double> temp_Q = _Q_z_;
  std::vector<double> temp_S = _S_z_;

  if (!reference->use_position_vertical) {
    temp_Q[0] = 0;
    temp_S[0] = 0;
  }

  if (!reference->use_velocity_vertical) {
    temp_Q[1] = 0;
    temp_S[1] = 0;
  }

  cvx_z_->lock();
  cvx_z_->setQ(temp_Q);
  cvx_z_->setS(temp_S);
  cvx_z_->setParams();
  cvx_z_->setLastInput(cvx_z_u_);
  cvx_z_->loadReference(mpc_reference_z);
  cvx_z_->setLimits(_max_speed_vertical_, _max_acceleration_vertical_, _max_u_vertical_, 999.0, _dt1_, _dt2_);
  cvx_z_->setInitialState(initial_z);
  [[maybe_unused]] int iters_z = cvx_z_->solveCvx();
  cvx_z_u_                     = cvx_z_->getFirstControlInput();
  cvx_z_->unlock();

  // --------------------------------------------------------------
  // |                       lateral control                      |
  // --------------------------------------------------------------

  // | ----------- get the current orientation angles ----------- |

  double         yaw, pitch, roll;
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(uav_state->pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(roll, pitch, yaw);

  // | --------------------- load the gains --------------------- |

  filterGains(reference->disable_position_gains, dt);

  Eigen::Vector3d Ka;
  Eigen::Array3d  Kq, Kw;

  {
    std::scoped_lock lock(mutex_gains_);

    Kq << kqxy_, kqxy_, kqz_;
    Kw << kwxy_, kwxy_, kwz_;
  }

  // | ---------- desired orientation matrix and force ---------- |

  Ra << reference->acceleration.x, reference->acceleration.y, reference->acceleration.z + cvx_z_u_;

  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward = total_mass * (Eigen::Vector3d(0, 0, _g_) + Ra);

  Eigen::Vector3d f = feed_forward;

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[So3Controller]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[AccelerationController]: NaN detected in variable 'theta', returning null");

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (_tilt_angle_failsafe_ > 1e-3 && theta > _tilt_angle_failsafe_) {

    ROS_ERROR("[AccelerationController]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null",
              (180.0 / M_PI) * theta, (180.0 / M_PI) * _tilt_angle_failsafe_);
    ROS_INFO("[AccelerationController]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
    ROS_INFO("[AccelerationController]: feed forward: [%.2f, %.2f, %.2f]", feed_forward[0], feed_forward[1], feed_forward[2]);
    ROS_INFO("[AccelerationController]: position_cmd: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", reference->position.x, reference->position.y,
             reference->position.z, reference->yaw);
    ROS_INFO("[AccelerationController]: odometry: x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", uav_state->pose.position.x, uav_state->pose.position.y,
             uav_state->pose.position.z, yaw);

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle
  if (_tilt_angle_saturation_ > 1e-3 && theta > _tilt_angle_saturation_) {
    ROS_WARN_THROTTLE(1.0, "[AccelerationController]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta / M_PI) * 180.0,
                      (_tilt_angle_saturation_ / M_PI) * 180.0);
    theta = _tilt_angle_saturation_;
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
    ROS_WARN_THROTTLE(1.0, "[AccelerationController]: just so you know, the desired thrust force is negative (%.2f", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    ROS_ERROR("[AccelerationController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");
  } else if (thrust > _thrust_saturation_) {
    thrust = _thrust_saturation_;
    ROS_WARN("[AccelerationController]: saturating thrust to %.2f", _thrust_saturation_);
  } else if (thrust < 0.0) {
    thrust = 0.0;
    ROS_WARN("[AccelerationController]: saturating thrust to %.2f", 0.0);
  }

  Eigen::Vector3d t;
  t = -Kq * Eq.array() - Kw * Ew.array();

  // --------------------------------------------------------------
  // |                       mass estimation                      |
  // --------------------------------------------------------------

  /* mass estimatior //{ */

  {
    std::scoped_lock lock(mutex_gains_);

    // antiwindup
    double temp_gain = km_;
    if (fabs(uav_state->velocity.linear.z) > 0.3 && ((Ep[2] < 0 && uav_state->velocity.linear.z > 0) || (Ep[2] > 0 && uav_state->velocity.linear.z < 0))) {
      temp_gain = 0;
      ROS_INFO_THROTTLE(1.0, "[AccelerationController]: anti-windup for the mass kicks in");
    }
    uav_mass_difference_ -= temp_gain * Ep[2] * dt;

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[AccelerationController]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[AccelerationController]: the UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {
    Eigen::Quaterniond des_quater = Eigen::Quaterniond(Rd);

    // rotate the drone's z axis
    Eigen::Vector3d uav_z_in_world = des_quater * Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d thrust_vector = thrust_force * uav_z_in_world;

    double world_accel_x = (thrust_vector[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass);
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

    ROS_WARN_THROTTLE(1.0, "[So3Controller]: outputting attitude quaternion (this is not normal)");
  }

  output_command->desired_acceleration.x = desired_x_accel;
  output_command->desired_acceleration.y = desired_y_accel;
  output_command->desired_acceleration.z = desired_z_accel;

  output_command->thrust          = thrust;
  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = _uav_mass_ + uav_mass_difference_;

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "AccelerationController";

  last_attitude_cmd_ = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus AccelerationController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void AccelerationController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &msg) {
}

//}

/* resetDisturbanceEstimators() //{ */

void AccelerationController::resetDisturbanceEstimators(void) {
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void AccelerationController::callbackDrs(mrs_controllers::acceleration_controllerConfig &config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_gains_ = config;
  }

  ROS_INFO("[AccelerationController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void AccelerationController::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kqxy_ = calculateGainChange(dt, kqxy_, drs_gains_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kqz_  = calculateGainChange(dt, kqz_, drs_gains_.kqz * gain_coeff, bypass_filter, "kqz", updated);
    kwxy_ = calculateGainChange(dt, kwxy_, drs_gains_.kwxy * gain_coeff, bypass_filter, "kwxy", updated);
    kwz_  = calculateGainChange(dt, kwz_, drs_gains_.kwz * gain_coeff, bypass_filter, "kwz", updated);
    km_   = calculateGainChange(dt, km_, drs_gains_.km * gain_coeff, bypass_filter, "km", updated);

    km_lim_ = calculateGainChange(dt, km_lim_, drs_gains_.km_lim, false, "km_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_gains_;

      new_drs_gains_.kqxy = kqxy_;
      new_drs_gains_.kqz  = kqz_;
      new_drs_gains_.kwxy = kwxy_;
      new_drs_gains_.kwz  = kwz_;
      new_drs_gains_.km   = km_;

      new_drs_gains_.km_lim = km_lim_;

      drs_->updateConfig(new_drs_gains_);
    }
  }
}

//}

/* calculateGainChange() //{ */

double AccelerationController::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate,
                                                   std::string name, bool &updated) {

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
    ROS_INFO_THROTTLE(1.0, "[AccelerationController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace acceleration_controller

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::acceleration_controller::AccelerationController, mrs_uav_manager::Controller)

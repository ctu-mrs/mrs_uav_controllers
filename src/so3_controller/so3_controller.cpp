/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#include <mrs_msgs/ControllerStatus.h>
#include <mrs_mav_manager/Controller.h>

#include <mrs_controllers/so3_gainsConfig.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Utils.h>

//}

#define X 0
#define Y 1
#define Z 2

namespace mrs_controllers
{

  /* //{ class So3Controller */

  class So3Controller : public mrs_mav_manager::Controller {

  public:
    So3Controller(void);

    void initialize(const ros::NodeHandle &parent_nh, mrs_mav_manager::MotorParams motor_params);
    bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd);
    void deactivate(void);

    const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &odometry, const mrs_msgs::PositionCommand::ConstPtr &reference);
    const mrs_msgs::ControllerStatus::Ptr     getStatus();

    void dynamicReconfigureCallback(mrs_controllers::so3_gainsConfig &config, uint32_t level);

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
    typedef mrs_controllers::so3_gainsConfig    Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer>        reconfigure_server_;
    void                                        drs_callback(mrs_controllers::so3_gainsConfig &config, uint32_t level);
    mrs_controllers::so3_gainsConfig            drs_desired_gains;

  private:
    double                       uav_mass_;
    double                       uav_mass_difference;
    double                       g_;
    mrs_mav_manager::MotorParams motor_params_;

    double roll, pitch, yaw;

    double yaw_offset;

    // actual gains (used and already filtered)
    double kpxy, kiwxy, kibxy, kvxy, kaxy;
    double kpz, kvz, kaz;
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

  So3Controller::So3Controller(void) {
  }

  //}

  // --------------------------------------------------------------
  // |                   controller's interface                   |
  // --------------------------------------------------------------

  /* //{ initialize() */

  void So3Controller::initialize(const ros::NodeHandle &parent_nh, mrs_mav_manager::MotorParams motor_params) {

    ros::NodeHandle nh_(parent_nh, "so3_controller");

    ros::Time::waitForValid();

    this->motor_params_ = motor_params;

    // --------------------------------------------------------------
    // |                       load parameters                      |
    // --------------------------------------------------------------

    mrs_lib::ParamLoader param_loader(nh_, "So3Controller");

    param_loader.load_param("enable_profiler", profiler_enabled_);

    // lateral gains
    param_loader.load_param("default_gains/horizontal/kp", kpxy);
    param_loader.load_param("default_gains/horizontal/kv", kvxy);
    param_loader.load_param("default_gains/horizontal/ka", kaxy);

    param_loader.load_param("default_gains/horizontal/kiw", kiwxy);
    param_loader.load_param("default_gains/horizontal/kib", kibxy);

    param_loader.load_param("lateral_mute_coefficitent", mute_coefficitent_);

    // height gains
    param_loader.load_param("default_gains/vertical/kp", kpz);
    param_loader.load_param("default_gains/vertical/kv", kvz);
    param_loader.load_param("default_gains/vertical/ka", kaz);

    // attitude gains
    param_loader.load_param("default_gains/horizontal/attitude/kq", kqxy);
    param_loader.load_param("default_gains/vertical/attitude/kq", kqz);

    // attitude rate gains
    param_loader.load_param("default_gains/horizontal/attitude/kw", kwxy);
    param_loader.load_param("default_gains/vertical/attitude/kw", kwz);

    // mass estimator
    param_loader.load_param("default_gains/weight_estimator/km", km);
    param_loader.load_param("default_gains/weight_estimator/km_lim", km_lim);

    // integrator limits
    param_loader.load_param("default_gains/horizontal/kiw_lim", kiwxy_lim);
    param_loader.load_param("default_gains/horizontal/kib_lim", kibxy_lim);

    // physical
    param_loader.load_param("uav_mass", uav_mass_);
    param_loader.load_param("g", g_);

    // constraints
    param_loader.load_param("max_tilt_angle", max_tilt_angle_);

    // yaw offset for compensating for heading error estimation
    param_loader.load_param("yaw_offset", yaw_offset);

    // gain filtering
    param_loader.load_param("gains_filter/filter_rate", gains_filter_timer_rate_);
    param_loader.load_param("gains_filter/perc_change_rate", gains_filter_change_rate_);
    param_loader.load_param("gains_filter/min_change_rate", gains_filter_min_change_rate_);

    gains_filter_max_change_ = gains_filter_change_rate_ / gains_filter_timer_rate_;
    gains_filter_min_change_ = gains_filter_min_change_rate_ / gains_filter_timer_rate_;

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[So3Controller]: Could not load all parameters!");
      ros::shutdown();
    }

    // convert to radians
    max_tilt_angle_ = (max_tilt_angle_ / 180) * 3.141592;
    yaw_offset      = (yaw_offset / 180.0) * 3.141592;

    uav_mass_difference = 0;
    Iw_w                = Eigen::Vector2d::Zero(2);
    Ib_b                = Eigen::Vector2d::Zero(2);

    // --------------------------------------------------------------
    // |                     dynamic reconfigure                    |
    // --------------------------------------------------------------

    drs_desired_gains.kpxy      = kpxy;
    drs_desired_gains.kvxy      = kvxy;
    drs_desired_gains.kaxy      = kaxy;
    drs_desired_gains.kiwxy     = kiwxy;
    drs_desired_gains.kibxy     = kibxy;
    drs_desired_gains.kpz       = kpz;
    drs_desired_gains.kvz       = kvz;
    drs_desired_gains.kaz       = kaz;
    drs_desired_gains.kqxy      = kqxy;
    drs_desired_gains.kqz       = kqz;
    drs_desired_gains.kwxy      = kwxy;
    drs_desired_gains.kwz       = kwz;
    drs_desired_gains.kiwxy_lim = kiwxy_lim;
    drs_desired_gains.kibxy_lim = kibxy_lim;
    drs_desired_gains.km        = km;
    drs_desired_gains.km_lim    = km_lim;

    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
    reconfigure_server_->updateConfig(drs_desired_gains);
    ReconfigureServer::CallbackType f = boost::bind(&So3Controller::dynamicReconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    // --------------------------------------------------------------
    // |                          profiler                          |
    // --------------------------------------------------------------

    profiler = new mrs_lib::Profiler(nh_, "So3Controller", profiler_enabled_);

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    timer_gain_filter = nh_.createTimer(ros::Rate(gains_filter_timer_rate_), &So3Controller::timerGainsFilter, this);

    // | ----------------------- finish init ---------------------- |

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[So3Controller]: Could not load all parameters!");
      ros::shutdown();
    }

    ROS_INFO("[So3Controller]: initialized");

    is_initialized = true;
  }

  //}

  /* //{ activate() */

  bool So3Controller::activate(const mrs_msgs::AttitudeCommand::ConstPtr &cmd) {

    if (cmd == mrs_msgs::AttitudeCommand::Ptr()) {
      activation_control_command_ = mrs_msgs::AttitudeCommand();
      uav_mass_difference         = 0;
      ROS_WARN("[So3Controller]: activated without getting the last tracker's command.");
    } else {
      activation_control_command_ = *cmd;
      uav_mass_difference         = cmd->mass_difference;
      ROS_INFO("[So3Controller]: activated with a last trackers command.");
    }

    first_iteration = true;

    ROS_INFO("[So3Controller]: activated");

    is_active = true;

    return true;
  }

  //}

  /* //{ deactivate() */

  void So3Controller::deactivate(void) {

    first_iteration     = false;
    uav_mass_difference = 0;

    ROS_INFO("[So3Controller]: deactivated");
  }

  //}

  /* //{ update() */

  const mrs_msgs::AttitudeCommand::ConstPtr So3Controller::update(const nav_msgs::Odometry::ConstPtr &       odometry,
                                                                  const mrs_msgs::PositionCommand::ConstPtr &reference) {

    mrs_lib::Routine profiler_routine = profiler->createRoutine("update");

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

      ROS_WARN_STREAM_THROTTLE(1.0, "[So3Controller]: last " << last_update << ", current " << odometry->header.stamp);
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: the last odometry message came too close! %f", dt);
      if (last_output_command != mrs_msgs::AttitudeCommand::Ptr()) {

        return last_output_command;

      } else {

        return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_control_command_));
      }
    }

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

    if (reference->use_position) {

      Rp << reference->position.x, reference->position.y, reference->position.z;  // fill the desired position

      if (reference->use_euler_attitude) {
        Rq.coeffs() << 0, 0, sin(reference->yaw / 2.0), cos(reference->yaw / 2.0);
      }
    }

    if (reference->use_velocity) {
      Rv << reference->velocity.x, reference->velocity.y, reference->velocity.z;
    } else {
      Rv << 0, 0, 0;
    }

    if (reference->use_acceleration) {
      Ra << reference->acceleration.x, reference->acceleration.y, reference->acceleration.z;
    } else {
      Ra << 0, 0, 0;
    }

    if (reference->use_attitude_rate) {
      Rw << reference->attitude_rate.x, reference->attitude_rate.y, reference->attitude_rate.z;
    } else if (reference->use_euler_attitude) {
      Rw << 0, 0, reference->yaw_dot;
    }

    // Op - position in global frame
    // Ov - velocity in global frame
    Eigen::Vector3d Op(odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z);
    Eigen::Vector3d Ov(odometry->twist.twist.linear.x, odometry->twist.twist.linear.y, odometry->twist.twist.linear.z);

    // Oq - UAV attitude quaternion
    /* Eigen::Quaternion<double> Oq = Eigen::Quaterniond(odometry->pose.pose.orientation.w, odometry->pose.pose.orientation.x,
     * odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z); */
    Eigen::Quaternion<double> Oq;
    Oq.coeffs() << odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w;
    Eigen::Matrix3d R = Oq.toRotationMatrix();

    // Ow - UAV angular rate
    Eigen::Vector3d Ow(odometry->twist.twist.angular.x, odometry->twist.twist.angular.y, odometry->twist.twist.angular.z);

    // --------------------------------------------------------------
    // |                  calculate control errors                  |
    // --------------------------------------------------------------

    Eigen::Vector3d Ep = Op - Rp;
    Eigen::Vector3d Ev = Ov - Rv;

    // --------------------------------------------------------------
    // |                            gains                           |
    // --------------------------------------------------------------
    //
    Eigen::Vector3d Ka;
    Eigen::Array3d  Kp, Kv, Kq, Kw;

    {
      std::scoped_lock lock(mutex_gains);

      Kp << kpxy, kpxy, kpz;
      Kv << kvxy, kvxy, kvz;
      Ka << kaxy, kaxy, kaz;
      Kq << kqxy, kqxy, kqz;
      Kw << kwxy, kwxy, kwz;
    }

    // --------------------------------------------------------------
    // |                 desired orientation matrix                 |
    // --------------------------------------------------------------

    Eigen::Vector2d Ib_w = rotate2d(Ib_b, -yaw);

    Eigen::Vector3d Ip(Ib_w[0] + Iw_w[0], Ib_w[1] + Iw_w[1], 0);

    Eigen::Vector3d f = -Kp * Ep.array() - Kv * Ev.array() + Ip.array() + (uav_mass_ + uav_mass_difference) * (Eigen::Vector3d(0, 0, 9.81) + Ra).array();

    Rd.col(2) = f.normalized();
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

    /* output */
    double thrust_force = f.dot(R.col(2));
    double thrust       = 0;
    if (thrust_force >= 0) {
      thrust = sqrt((f.dot(R.col(2)) / 10.0) * g_) * motor_params_.hover_thrust_a + motor_params_.hover_thrust_b;
    } else {
      ROS_WARN_THROTTLE(1.0, "[So3Controller]: Just so you know, the desired thrust force is negative (%f", thrust_force);
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

    /* world error integrator //{ */

    // --------------------------------------------------------------
    // |                  integrate the world error                 |
    // --------------------------------------------------------------

    {
      std::scoped_lock lock(mutex_gains);

      Eigen::Vector3d integration_switch(1, 1, 0);

      // integrate the world error
      Iw_w -= kiwxy * Ep.head(2) * dt;

      // saturate the world
      double world_integral_saturated = false;
      if (!std::isfinite(Iw_w[0])) {
        Iw_w[0] = 0;
        ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Iw_w[0]\", setting it to 0!!!");
      } else if (Iw_w[0] > kiwxy_lim) {
        Iw_w[0]                  = kiwxy_lim;
        world_integral_saturated = true;
      } else if (Iw_w[0] < -kiwxy_lim) {
        Iw_w[0]                  = -kiwxy_lim;
        world_integral_saturated = true;
      }

      if (kiwxy_lim >= 0 && world_integral_saturated) {
        ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's world X integral is being saturated!");
      }

      // saturate the world
      world_integral_saturated = false;
      if (!std::isfinite(Iw_w[1])) {
        Iw_w[1] = 0;
        ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Iw_w[1]\", setting it to 0!!!");
      } else if (Iw_w[1] > kiwxy_lim) {
        Iw_w[1]                  = kiwxy_lim;
        world_integral_saturated = true;
      } else if (Iw_w[1] < -kiwxy_lim) {
        Iw_w[1]                  = -kiwxy_lim;
        world_integral_saturated = true;
      }

      if (kiwxy_lim >= 0 && world_integral_saturated) {
        ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's world Y integral is being saturated!");
      }
    }

    //}

    /* body error integrator //{ */

    // --------------------------------------------------------------
    // |                  integrate the body error                 |
    // --------------------------------------------------------------

    {
      std::scoped_lock lock(mutex_gains);

      // rotate the control errors to the body
      Eigen::Vector2d Ep_body = rotate2d(Ep.head(2), yaw);

      // integrate the body error
      Ib_b -= kibxy * Ep_body * dt;

      // saturate the body
      double body_integral_saturated = false;
      if (!std::isfinite(Ib_b[0])) {
        Ib_b[0] = 0;
        ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Ib_b[0]\", setting it to 0!!!");
      } else if (Ib_b[0] > kibxy_lim) {
        Ib_b[0]                 = kibxy_lim;
        body_integral_saturated = true;
      } else if (Ib_b[0] < -kibxy_lim) {
        Ib_b[0]                 = -kibxy_lim;
        body_integral_saturated = true;
      }

      if (kibxy_lim > 0 && body_integral_saturated) {
        ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's body pitch integral is being saturated!");
      }

      // saturate the body
      body_integral_saturated = false;
      if (!std::isfinite(Ib_b[1])) {
        Ib_b[1] = 0;
        ROS_ERROR_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"Ib_b[1]\", setting it to 0!!!");
      } else if (Ib_b[1] > kibxy_lim) {
        Ib_b[1]                 = kibxy_lim;
        body_integral_saturated = true;
      } else if (Ib_b[1] < -kibxy_lim) {
        Ib_b[1]                 = -kibxy_lim;
        body_integral_saturated = true;
      }

      if (kibxy_lim > 0 && body_integral_saturated) {
        ROS_WARN_THROTTLE(1.0, "[So3Controller]: SO3's body roll integral is being saturated!");
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
        ROS_WARN_THROTTLE(1.0, "[So3Controller]: NaN detected in variable \"uav_mass_difference\", setting it to 0 and returning!!!");
      } else if (uav_mass_difference > km_lim) {
        uav_mass_difference = km_lim;
        uav_mass_saturated  = true;
      } else if (uav_mass_difference < -km_lim) {
        uav_mass_difference = -km_lim;
        uav_mass_saturated  = true;
      }

      if (uav_mass_saturated) {
        ROS_WARN_THROTTLE(1.0, "[So3Controller]: The uav_mass_difference is being saturated to %1.3f!", uav_mass_difference);
      }
    }

    //}

    // --------------------------------------------------------------
    // |            report on the values of the integrals           |
    // --------------------------------------------------------------

    ROS_INFO_THROTTLE(5.0, "[So3Controller]: world error integral: x %1.2f, y %1.2f, lim: %1.2f", Iw_w[X], Iw_w[Y], kiwxy_lim);
    ROS_INFO_THROTTLE(5.0, "[So3Controller]: body error integral:  x %1.2f, y %1.2f, lim: %1.2f", Ib_b[X], Ib_b[Y], kibxy_lim);

    // --------------------------------------------------------------
    // |                 produce the control output                 |
    // --------------------------------------------------------------

    mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
    output_command->header.stamp = ros::Time::now();

    // output the desired attitude rate
    output_command->attitude_rate.x = 1 * t[0];
    output_command->attitude_rate.y = -1 * t[1];
    output_command->attitude_rate.z = -1 * t[2];

    // output the desired attitude
    /* Eigen::Quaterniond thrust_vec = Eigen::Quaterniond(Rd); */
    /* output_command->quter_attitude.w = thrust_vec.w(); */
    /* output_command->quter_attitude.x = thrust_vec.x(); */
    /* output_command->quter_attitude.y = thrust_vec.y(); */
    /* output_command->quter_attitude.z = thrust_vec.z(); */

    output_command->quter_attitude.w = cos(reference->yaw / 2.0);
    output_command->quter_attitude.x = 0;
    output_command->quter_attitude.y = 0;
    output_command->quter_attitude.z = sin(reference->yaw / 2.0);

    output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;
    /* output_command->mode_mask = output_command->MODE_QUATER_ATTITUDE; */

    output_command->thrust = thrust;

    output_command->mass_difference = uav_mass_difference;

    last_output_command = output_command;

    return output_command;
  }

  //}

  /* //{ getStatus() */

  const mrs_msgs::ControllerStatus::Ptr So3Controller::getStatus() {

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

  void So3Controller::dynamicReconfigureCallback(mrs_controllers::so3_gainsConfig &config, [[maybe_unused]] uint32_t level) {

    {
      std::scoped_lock lock(mutex_desired_gains);

      drs_desired_gains = config;
    }

    ROS_INFO("[So3Controller]: DRS updated gains");
  }

  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* timerGainFilter() //{ */

  void So3Controller::timerGainsFilter(const ros::TimerEvent &event) {

    mrs_lib::Routine profiler_routine = profiler->createRoutine("timerGainsFilter", gains_filter_timer_rate_, 0.01, event);

    double gain_coeff                = 1;
    bool   bypass_filter             = mute_lateral_gains || mutex_lateral_gains_after_toggle;
    mutex_lateral_gains_after_toggle = false;

    if (mute_lateral_gains) {
      gain_coeff = mute_coefficitent_;
    }

    // calculate the difference
    {
      std::scoped_lock lock(mutex_gains, mutex_desired_gains);

      kpxy      = calculateGainChange(kpxy, drs_desired_gains.kpxy * gain_coeff, bypass_filter, "kpxy");
      kvxy      = calculateGainChange(kvxy, drs_desired_gains.kvxy * gain_coeff, bypass_filter, "kvxy");
      kaxy      = calculateGainChange(kaxy, drs_desired_gains.kaxy * gain_coeff, bypass_filter, "kaxy");
      kiwxy     = calculateGainChange(kiwxy, drs_desired_gains.kiwxy * gain_coeff, bypass_filter, "kiwxy");
      kibxy     = calculateGainChange(kibxy, drs_desired_gains.kibxy * gain_coeff, bypass_filter, "kibxy");
      kpz       = calculateGainChange(kpz, drs_desired_gains.kpz, false, "kpz");
      kvz       = calculateGainChange(kvz, drs_desired_gains.kvz, false, "kvz");
      kaz       = calculateGainChange(kaz, drs_desired_gains.kaz, false, "kaz");
      kqxy      = calculateGainChange(kqxy, drs_desired_gains.kqxy, false, "kqxy");
      kqz       = calculateGainChange(kqz, drs_desired_gains.kqz, false, "kqz");
      kwxy      = calculateGainChange(kwxy, drs_desired_gains.kwxy, false, "kwxy");
      kwz       = calculateGainChange(kwz, drs_desired_gains.kwz, false, "kwz");
      km        = calculateGainChange(km, drs_desired_gains.km, false, "km");
      kiwxy_lim = calculateGainChange(kiwxy_lim, drs_desired_gains.kiwxy_lim, false, "kiwxy_lim");
      kibxy_lim = calculateGainChange(kibxy_lim, drs_desired_gains.kibxy_lim, false, "kibxy_lim");
      km_lim    = calculateGainChange(km_lim, drs_desired_gains.km_lim, false, "km_lim");
    }
  }

  //}

  // --------------------------------------------------------------
  // |                       other routines                       |
  // --------------------------------------------------------------

  /* calculateGainChange() //{ */

  double So3Controller::calculateGainChange(const double current_value, const double desired_value, const bool bypass_rate, std::string name) {

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
      ROS_INFO_THROTTLE(1.0, "[So3Controller]: changing gain \"%s\" from %f to %f", name.c_str(), current_value, desired_value);
    }

    return current_value + change;
  }

  //}

  /* reset() //{ */

  bool So3Controller::reset(void) {

    Iw_w = Eigen::Vector2d::Zero(2);

    return true;
  }

  //}

  /* rotate2d() //{ */

  Eigen::Vector2d So3Controller::rotate2d(const Eigen::Vector2d vector_in, double angle) {

    Eigen::Rotation2D<double> rot2(angle);

    return rot2.toRotationMatrix() * vector_in;
  }

  //}

}  // namespace mrs_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::So3Controller, mrs_mav_manager::Controller)

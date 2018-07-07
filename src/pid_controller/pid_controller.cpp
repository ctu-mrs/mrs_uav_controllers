#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <list>

#include <mrs_controllers/debug.h>
#include <mrs_controllers/pid_gainsConfig.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_uav_manager/Controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_controllers/pid_gainsConfig.h>
#include <std_srvs/SetBool.h>

using namespace std;

namespace mrs_controllers
{

  class PidController : public mrs_uav_manager::Controller {

    public:

      PidController(void);

      void Initialize(const ros::NodeHandle &parent_nh);
      bool Activate(void);
      void Deactivate(void);

      const mrs_msgs::AttitudeCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg, const mrs_msgs::PositionCommand::ConstPtr &cmd);
      const mrs_msgs::ControllerStatus::Ptr status();

      /* void InputManage2(const nav_msgs::OdometryConstPtr &odom); */
      /* void InputManage(const geometry_msgs::PoseStampedConstPtr &pose); */

      boost::recursive_mutex                      config_mutex_;
      typedef mrs_controllers::pid_gainsConfig    Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      boost::shared_ptr<ReconfigureServer>        reconfigure_server_;
      void drs_callback(mrs_controllers::pid_gainsConfig &config, uint32_t level);
      mrs_controllers::pid_gainsConfig last_drs_config;

    private:
      tf::Quaternion q_orig, q_rot;
      /* ros::Publisher cmd_publisher; */

      ros::Publisher  debugPub;
      ros::Subscriber input_subscriber;
      ros::Subscriber input_subscriber2;
      ros::Subscriber input_points;
      double          x, y, z, TH;
      double          roll, pitch, yaw, yawD, oldPitch;
      double          elx, ely, elz, lz, lx, ly, SmallKpA;
      double          ez, ex, ey, dez, dex, dey, dz, dx, dy, iez, iex, iey;

      // ofstream myfile,myfile2,myfile3;
      double hz = 200;
      double Kp, KI, Kd;
      double KpA, KIA, KdA;
      double dT        = 1 / hz;
      double setPointZ = 3, setPointX = 2, setPointY = 2;
      mrs_msgs::PositionCommand::ConstPtr lastCommand;

      double lastTime;

    private:
      void SetGainsDRS(mrs_controllers::pid_gainsConfig &config, uint32_t level);
  };

  PidController::PidController(void) {
  }

  double IsReal(double number) {
    if (isnan(number) || number == INFINITY)
      return 0;
    return number;
  }

  void PidController::SetGainsDRS(mrs_controllers::pid_gainsConfig &config, uint32_t level) {

    ROS_INFO("Controller constants were Kp: %3.5f, KI: %3.5f, Kd: %3.5f,KpA: %3.5f, KIA: %3.5f, KdA: %3.5f", Kp, KI, Kd, KpA, KIA, KdA);
    Kp       = config.P;
    Kd       = config.D;
    KI       = config.I;
    KpA      = config.Pa;
    KdA      = config.Da;
    KIA      = config.Ia;
    TH       = config.thrustHover;
    SmallKpA = config.Pags;
    ROS_INFO("Controller constants ARE Kp: %3.5f, KI: %3.5f, Kd: %3.5f,KpA: %3.5f, KIA: %3.5f, KdA: %3.5f", Kp, KI, Kd, KpA, KIA, KdA);
  }

  /* void PidController::InputManage(const geometry_msgs::PoseStampedConstPtr &pose) { */

  /*   x  = pose.pose.position.x; */
  /*   y  = pose.pose.position.y; */
  /*   z  = pose.pose.position.z; */
  /*   dT = lastTime - pose.header.stamp.toSec(); */
  /*   if (dT < 0.001) { */
  /*     dT = 0.001; */
  /*   } */
  /*   lastTime = pose.header.stamp.toSec(); */
  /*   // Odchylka */
  /*   ez = (setPointZ - z); */
  /*   ex = (setPointX - x); */
  /*   ey = (setPointY - y); */
  /*   // Derivace Odchylky */
  /*   dez = (ez - elz) / (dT); */
  /*   elz = ez; */
  /*   dex = (ex - elx) / (dT); */
  /*   elx = ex; */
  /*   dey = (ey - ely) / (dT); */
  /*   ely = ey; */
  /*   // Integrace Odchylky */
  /*   iez += ez; */
  /*   iex -= ex; */
  /*   iey -= ey; */
  /*   // Pid */
  /*   dz = (z - lz) / (dT); */
  /*   lz = z; */
  /*   dx = (x - lx) / (dT); */
  /*   lx = x; */
  /*   dy = (y - ly) / (dT); */
  /*   ly = y; */
  /*   //  myfile << x <<endl; */
  /*   //   myfile2 << ros::Time::now().toSec() <<endl; */
  /*   //   myfile3 << dx <<endl; */
  /*   quaternionMsgToTF(pose.pose.orientation, q_orig); */
  /*   tf::Matrix3x3 m(q_orig); */
  /*   m.getRPY(roll, pitch, yaw); */
  /* } */

  bool PidController::Activate(void) {

    return true;
  }

  void PidController::Deactivate(void) {
  }

  void PidController::Initialize(const ros::NodeHandle &parent_nh) {

    ros::NodeHandle priv_nh(parent_nh, "pid_controller");

    ros::Time::waitForValid();

    double u;

    // SUBSCRIBERS
    /* input_subscriber = priv_nh.subscribe("pose_in", 1, &PidController::InputManage, this, ros::TransportHints().tcpNoDelay()); */
    /* input_subscriber2 = priv_nh.subscribe("odom_in", 1, &PidController::InputManage2, this, ros::TransportHints().tcpNoDelay()); */

    /* cmd_publisher = priv_nh.advertise<mavros_msgs::AttitudeTarget>("action_out", 1); */

    debugPub = priv_nh.advertise<mrs_controllers::debug>("debug", 1);

    priv_nh.param("Kp", Kp, -1.0);
    priv_nh.param("Ki", KI, -1.0);
    priv_nh.param("Kd", Kd, -1.0);
    priv_nh.param("KpA", KpA, -1.0);
    priv_nh.param("KiA", KIA, -1.0);
    priv_nh.param("KdA", KdA, -1.0);
    priv_nh.param("thrust_hover", TH, -1.0);
    priv_nh.param("SmallKpA", SmallKpA, -1.0);

    if (Kp < 0) {
      ROS_ERROR("PidController: Kp is not specified!");
      ros::shutdown();
    }

    if (KI < 0) {
      ROS_ERROR("PidController: KI is not specified!");
      ros::shutdown();
    }

    if (Kd < 0) {
      ROS_ERROR("PidController: Kd is not specified!");
      ros::shutdown();
    }

    if (KpA < 0) {
      ROS_ERROR("PidController: KpA is not specified!");
      ros::shutdown();
    }

    if (KIA < 0) {
      ROS_ERROR("PidController: KIA is not specified!");
      ros::shutdown();
    }

    if (KdA < 0) {
      ROS_ERROR("PidController: KdA is not specified!");
      ros::shutdown();
    }

    if (TH < 0) {
      ROS_ERROR("PidController: thrust_hover is not specified!");
      ros::shutdown();
    }

    if (SmallKpA < 0) {
      ROS_ERROR("PidController: small_kpa is not specified!");
      ros::shutdown();
    }

    double Speedx = 0.0, Speedy = 0.0;
    double MaxAngel     = 15;
    double maxAngleRoll = MaxAngel, maxAnglePitch = MaxAngel, maxAngleYaw = MaxAngel;
    maxAngleRoll  = (maxAngleRoll / 360) * 2 * 3.1459;
    maxAnglePitch = (maxAnglePitch / 360) * 2 * 3.1459;
    maxAngleYaw   = (maxAngleYaw / 360) * 2 * 3.1459;

    ros::Rate r(hz);
    ROS_INFO("Controller running at %3f [Hz]", hz);
    ROS_INFO("Controller constants are Kp: %3.5f, KI: %3.5f, Kd: %3.5f", Kp, KI, Kd);
    ROS_INFO("Controller constants are KpA: %3.5f, KIA: %3.5f, KdA: %3.5f", KpA, KIA, KdA);
    dynamic_reconfigure::Server<mrs_controllers::pid_gainsConfig>               server(config_mutex_);
    dynamic_reconfigure::Server<mrs_controllers::pid_gainsConfig>::CallbackType f;

    f = boost::bind(&PidController::SetGainsDRS, this, _1, _2);
    server.setCallback(f);
    mrs_controllers::pid_gainsConfig conf;
    conf.P          = Kp;
    conf.D          = Kd;
    conf.I          = KI;
    conf.Pa         = KpA;
    conf.Da         = KdA;
    conf.Ia         = KIA;
    conf.thrustHover = TH;
    conf.Pags       = SmallKpA;

    config_mutex_.lock();
    server.updateConfig(conf);
    config_mutex_.unlock();

    mrs_msgs::PositionCommand cmdf;
    cmdf.yaw        = 0;
    cmdf.position.x = x;
    cmdf.position.y = y;
    cmdf.position.z = z;
    ros::Duration(1).sleep();
    mrs_msgs::PositionCommand::ConstPtr cmd_ptr(new mrs_msgs::PositionCommand(cmdf));

    std::list<double> filterFront;
    int               filterSize = 100;
    while (ros::ok()) {

      ros::spinOnce();
      mavros_msgs::AttitudeTarget newCommand;
      mrs_controllers::debug      debugmsg;
      newCommand.header.stamp = ros::Time::now();
      newCommand.type_mask    = 0;

      // body rate
      newCommand.body_rate.x = 0;
      newCommand.body_rate.y = 0;
      newCommand.body_rate.z = 0;

      u = (Kp * ez + KI * iez + Kd * dez) * (1 / (cos(roll) * cos(pitch)));

      if (-KpA * ey + KdA * (dy - Speedy) >= 1) {
        roll = asin(1);
        iey  = 0;
      } else if (-KpA * ey + KdA * (dy - Speedy) <= -1) {
        roll = asin(-1);
        iey  = 0;
      } else {
        if ((y - setPointY) * (y - setPointY) <= 1)
          roll = asin(-KpA * ey * SmallKpA + KdA * (dy - Speedy) + KIA * iey);
        else {
          roll = asin(-KpA * ey + KdA * (dy - Speedy) + KIA * iey);
        }
      }

      if (-KpA * ex + KdA * (dx - Speedx) >= 1) {
        pitch = -asin(1);
        iex   = 0;
      } else if (-KpA * ex + KdA * (dx - Speedx) <= -1) {
        pitch = -asin(-1);
        iex   = 0;
      } else {
        if ((x - setPointX) * (x - setPointX) <= 1) {
          pitch = -asin(-KpA * ex * SmallKpA + KdA * (dx - Speedx) + KIA * iex);
        } else {
          pitch = -asin(-KpA * ex + KdA * (dx - Speedx) + KIA * iex);
        }
      }

      newCommand.thrust = 1 - 1 / (1 + exp(u + TH));
      newCommand.thrust = IsReal(newCommand.thrust);  // INF NAN check
      filterFront.push_front(u);
      if (filterFront.size() > filterSize) {
        filterFront.pop_back();
      }
      double                      sum = 0, out = 0;
      std::list<double>::iterator it = filterFront.begin();
      for (int i = 0; i < filterFront.size(); i++) {
        sum += *it;
        std::advance(it, 1);
        out = sum / filterFront.size();
      }
      debugmsg.regoutThrust = out;

      // 0.95
      // ROS_INFO("U: %04.5f Z: %03.5f Com: %3.5f " , u,z,newCommand.thrust);
      // ROS_INFO("Kp: %f" , Kp);

      if (roll >= maxAngleRoll) {
        roll = maxAngleRoll;
        iey  = 0;
      } else if (roll <= -maxAngleRoll) {
        roll = -maxAngleRoll;
        iey  = 0;
      } else {
      }

      if (pitch >= maxAnglePitch) {
        pitch = maxAnglePitch;
        iex   = 0;
      } else if (pitch <= -maxAnglePitch) {
        pitch = -maxAnglePitch;
        iex   = 0;
      } else {
      }

      oldPitch = pitch;
      pitch    = pitch * cos(yaw) - roll * sin(yaw);
      roll     = roll * cos(yaw) + oldPitch * sin(yaw);
      yaw      = yawD;

      // Nan check & Inf check
      pitch = IsReal(pitch);
      roll  = IsReal(roll);
      yaw   = IsReal(yaw);
      // ROS_INFO("P: %04.5f R: %03.5f Y: %3.5f U: %3.5f " , pitch,roll,yaw,newCommand.thrust);
      q_orig = tf::createQuaternionFromRPY(roll, pitch, yaw);
      // q_orig *= q_rot;  // Calculate the new orientation
      q_orig.normalize();
      quaternionTFToMsg(q_orig, newCommand.orientation);
      // Got it out!
      debugPub.publish(debugmsg);
      /* cmd_publisher.publish(newCommand); */
      r.sleep();
    }
  }

  const mrs_msgs::AttitudeCommand::ConstPtr PidController::update(const nav_msgs::Odometry::ConstPtr &msg, const mrs_msgs::PositionCommand::ConstPtr &cmd) {

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  const mrs_msgs::ControllerStatus::Ptr PidController::status() {
    return mrs_msgs::ControllerStatus::Ptr();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_controllers::PidController, mrs_uav_manager::Controller)  //<reformat_checkpoint>

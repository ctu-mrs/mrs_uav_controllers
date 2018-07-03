// some ros includes
#include <math.h>
#include <list>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <mrs_uav_manager/Tracker.h>
// some std includes
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>   // For Input to the controller
#include <mavros_msgs/AttitudeTarget.h>  //For sending commands to lower controller
#include <mrs_msgs/TrackerPointStamped.h>
#include <nodelet/loader.h>
#include <pidcontroll.h>
#include <pidcontroll/debug.h>
#include <pidcontroll/SettingsConfig.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_datatypes.h>
//#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(PidController, nodelet::Nodelet
)
using namespace std;

tf::Quaternion q_orig, q_rot;
ros::Publisher cmd_publisher;

ros::Publisher debugPub;
ros::Subscriber input_subscriber;
ros::Subscriber input_subscriber2;
ros::Subscriber input_points;
double x, y, z,TH;
double roll, pitch, yaw, yawD, oldPitch;
double elx, ely, elz, lz, lx, ly, SmallKpA;
double ez, ex, ey, dez, dex, dey, dz, dx, dy, iez, iex, iey;
// ofstream myfile,myfile2,myfile3;
double hz = 200;
double Kp, KI, Kd;
double KpA, KIA, KdA;
double dT = 1 / hz;
int aTracker;
std::string tracker_list;
double setPointZ = 3, setPointX = 2, setPointY = 2;
// for trackers
bool trackerReady = false;
//boost::shared_ptr <trackers_manager::Tracker> tracker;
std::vector <boost::shared_ptr<trackers_manager::Tracker>> Trackers;
quadrotor_msgs::PositionCommand::ConstPtr lastCommand;
std::vector <std::string> vectorOfTrackers;
double IsReal(double number) {
    if (isnan(number) || number == INFINITY)
        return 0;
    return number;
}

void NewSettings(pidcontroll::SettingsConfig &config, uint32_t level) {
    if(aTracker!=config.aTracker&&trackerReady){
        ROS_WARN("Changing tracker %s for %s",vectorOfTrackers[aTracker].c_str(),vectorOfTrackers[config.aTracker].c_str());
        (*Trackers[aTracker]).Deactivate();
        (*Trackers[config.aTracker]).Activate(lastCommand);
        aTracker = config.aTracker;
        ROS_INFO("Trackers are switched");
    }else{
        ROS_INFO("Controller constants were Kp: %3.5f, KI: %3.5f, Kd: %3.5f,KpA: %3.5f, KIA: %3.5f, KdA: %3.5f", Kp, KI, Kd, KpA, KIA, KdA);
        Kp = config.P;
        Kd = config.D;
        KI = config.I;
        KpA = config.Pa;
        KdA = config.Da;
        KIA = config.Ia;
        TH = config.trustHover;
        SmallKpA = config.Pags;
        ROS_INFO("Controller constants ARE Kp: %3.5f, KI: %3.5f, Kd: %3.5f,KpA: %3.5f, KIA: %3.5f, KdA: %3.5f", Kp, KI, Kd, KpA, KIA, KdA);
    }

}

double lastTime;


void InputManage2(const nav_msgs::Odometry &odom) {
    if (trackerReady) {

        try {

            nav_msgs::Odometry odo;
            odo.header.stamp = ros::Time::now();
            odo.pose.pose.position = odom.pose.pose.position;
            odo.pose.pose.orientation = odom.pose.pose.orientation;
            odo.twist.twist.linear = odom.twist.twist.linear;
            odo.twist.twist.angular = odom.twist.twist.angular;
            //    quadrotor_msgs::PositionCommand::ConstPtr cmd_ptr(new quadrotor_msgs::PositionCommand(cmdf));
            nav_msgs::Odometry::ConstPtr odo_ptr(new nav_msgs::Odometry(odo));
            quadrotor_msgs::PositionCommand::ConstPtr cmd;
            for (int i = 0; i < Trackers.size(); ++i) {
                if(i == aTracker){
                     cmd = (*Trackers[i]).update(odo_ptr);
                }else{
                    (*Trackers[i]).update(odo_ptr);
                }
            }
                lastCommand = cmd;
            if (quadrotor_msgs::PositionCommand::Ptr() != cmd) {
                yawD = cmd->yaw;
                setPointX = cmd->position.x;
                setPointY = cmd->position.y;
                setPointZ = cmd->position.z;
            } else {
                ROS_DEBUG_THROTTLE(2, "Nothign to command");
            }

        } catch (std::runtime_error &exrun) {
            ROS_INFO("MPC tracke NOT initialized");
            ROS_ERROR("Exeption: %s", exrun.what());
        }


    }
}

  

void InputManage(const geometry_msgs::PoseStamped &pose) {
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    z = pose.pose.position.z;
   dT = lastTime - pose.header.stamp.toSec();
    if (dT < 0.001) {
        dT = 0.001;
    }
    lastTime = pose.header.stamp.toSec();
    // Odchylka
    ez = (setPointZ - z);
    ex = (setPointX - x);
    ey = (setPointY - y);
    // Derivace Odchylky
    dez = (ez - elz) / (dT);
    elz = ez;
    dex = (ex - elx) / (dT);
    elx = ex;
    dey = (ey - ely) / (dT);
    ely = ey;
    // Integrace Odchylky
    iez += ez;
    iex -= ex;
    iey -= ey;
    // Pid
    dz = (z - lz) / (dT);
    lz = z;
    dx = (x - lx) / (dT);
    lx = x;
    dy = (y - ly) / (dT);
    ly = y;
    //  myfile << x <<endl;
    //   myfile2 << ros::Time::now().toSec() <<endl;
    //   myfile3 << dx <<endl;
    quaternionMsgToTF(pose.pose.orientation, q_orig);
    tf::Matrix3x3 m(q_orig);
    m.getRPY(roll, pitch, yaw);
}

using namespace std;
void PidController::onInit() {

    // initialize node and create no handle
    ros::NodeHandle nh_ = nodelet::Nodelet::getPrivateNodeHandle();

    double u;

    // SUBSCRIBERS
    input_subscriber = nh_.subscribe("pose_in", 1, &InputManage, ros::TransportHints().tcpNoDelay());

    input_subscriber2 = nh_.subscribe("odom_in", 1, &InputManage2, ros::TransportHints().tcpNoDelay());


    // PUBLISHERS
    cmd_publisher = nh_.advertise<mavros_msgs::AttitudeTarget>("action_out", 1);

    debugPub = nh_.advertise<pidcontroll::debug>("debug", 1);

    nh_.getParam("Kp", Kp);
    nh_.getParam("KI", KI);
    nh_.getParam("Kd", Kd);
    nh_.getParam("KpA", KpA);
    nh_.getParam("KIA", KIA);
    nh_.getParam("KdA", KdA);
    nh_.getParam("trustHover", TH);
    nh_.getParam("SmallKpA", SmallKpA);
    nh_.getParam("ActiveTracker",aTracker);

    if (nh_.getParam("trackers", tracker_list)) {
        ROS_INFO("Got param: %s", tracker_list.c_str());
    } else {
        ROS_ERROR("Failed to get param 'trackers'");
    }
    std::stringstream stream(tracker_list);
    std::string segment;

    while (std::getline(stream, segment, ',')) {
        vectorOfTrackers.push_back(segment);
    }
    for (int i = 0; i < vectorOfTrackers.size(); ++i) {

        ROS_INFO("Tracker number %d has this name %s.", i, vectorOfTrackers.at(i).c_str());
    }

    ROS_INFO("Starting trakers");
    pluginlib::ClassLoader <trackers_manager::Tracker> trackers_loader("trackers_manager", "trackers_manager::Tracker");

    for (int j = 0; j < vectorOfTrackers.size(); ++j) {
        try {
            Trackers.push_back(trackers_loader.createInstance(vectorOfTrackers[j].c_str()));
        } catch (pluginlib::CreateClassException &ex1) {
            ROS_ERROR("CreateClassException for tracker number %d", j);
            ROS_INFO("Error: %s", ex1.what());
            return;
        } catch (pluginlib::PluginlibException &ex) {
            ROS_ERROR("PluginlibException for tracker number %d", j);
            ROS_INFO("Error: %s", ex.what());
            ROS_ERROR("End of PluginlibException");
            return;
        }
    }
    ROS_INFO("Trackers are loaded");

    //ROS_INFO_STREAM(string_list);
    //ROS_INFO("%c\n", string_list.data[0]);

    double Speedx = 0.0, Speedy = 0.0;
    double MaxAngel = 15;
    double maxAngleRoll = MaxAngel, maxAnglePitch = MaxAngel, maxAngleYaw = MaxAngel;
    maxAngleRoll = (maxAngleRoll / 360) * 2 * 3.1459;
    maxAnglePitch = (maxAnglePitch / 360) * 2 * 3.1459;
    maxAngleYaw = (maxAngleYaw / 360) * 2 * 3.1459;

    // myfile.open("/home/machy/workspace/src/back_stepping_controller/launch/X.txt");
    // myfile2.open("/home/machy/workspace/src/back_stepping_controller/launch/TX.txt");
    // myfile3.open("/home/machy/workspace/src/back_stepping_controller/launch/DX.txt");
    ros::Rate r(hz);
    ROS_INFO("Controller running at %3f [Hz]", hz);
    ROS_INFO("Controller constants are Kp: %3.5f, KI: %3.5f, Kd: %3.5f", Kp, KI, Kd);
    ROS_INFO("Controller constants are KpA: %3.5f, KIA: %3.5f, KdA: %3.5f", KpA, KIA, KdA);
    dynamic_reconfigure::Server <pidcontroll::SettingsConfig> server;
    dynamic_reconfigure::Server<pidcontroll::SettingsConfig>::CallbackType f;

    f = boost::bind(&NewSettings, _1, _2);
    server.setCallback(f);
    pidcontroll::SettingsConfig conf;
         conf.P=Kp;
         conf.D=Kd;
         conf.I=KI;
         conf.Pa=KpA;
         conf.Da=KdA;
         conf.Ia=KIA;
         conf.trustHover=TH;
         conf.Pags=SmallKpA;
 
    server.updateConfig(conf);
    ros::NodeHandle trackersNodes [Trackers.size()];

    for (int k = 0; k < Trackers.size(); ++k) {
        try {
            (*Trackers[k]).Initialize(trackersNodes[k], nh_);
            ROS_INFO("Initializing tracker: %s",vectorOfTrackers[k].c_str());
        } catch (std::runtime_error &exrun) {
            ROS_INFO("Tracker %s has NOT been initialized!", vectorOfTrackers[k].c_str());
            ROS_ERROR("Exeption: %s", exrun.what());
        }
    }

    trackerReady = true;
  quadrotor_msgs::PositionCommand cmdf;
    cmdf.yaw = 0;
    cmdf.position.x = x;
    cmdf.position.y = y;
    cmdf.position.z = z;
    ros::Duration(1).sleep();
    quadrotor_msgs::PositionCommand::ConstPtr cmd_ptr(new quadrotor_msgs::PositionCommand(cmdf));

        try {
            (*Trackers[aTracker]).Activate(cmd_ptr);
            ROS_INFO("Tracke %s has been activated",vectorOfTrackers[aTracker].c_str());
        } catch (std::runtime_error &exrun) {
            ROS_INFO("MPC tracke NOT activated for trucker number %d",0);
            ROS_INFO("Exeption: %s", exrun.what());
        }
  

 
   std::list<double> filterFront;
    int filterSize = 100;
    while (ros::ok()) {

        ros::spinOnce();
        mavros_msgs::AttitudeTarget newCommand;
        pidcontroll::debug debugmsg;
        newCommand.header.stamp = ros::Time::now();
        newCommand.type_mask = 0;

        // body rate
        newCommand.body_rate.x = 0;
        newCommand.body_rate.y = 0;
        newCommand.body_rate.z = 0;

        u = (Kp * ez + KI * iez + Kd * dez) * (1 / (cos(roll) * cos(pitch)));

        if (-KpA * ey + KdA * (dy - Speedy) >= 1) {
            roll = asin(1);
            iey = 0;
        } else if (-KpA * ey + KdA * (dy - Speedy) <= -1) {
            roll = asin(-1);
            iey = 0;
        } else {
            if ((y - setPointY) * (y - setPointY) <= 1)
                roll = asin(-KpA * ey * SmallKpA + KdA * (dy - Speedy) + KIA * iey);
            else {
                roll = asin(-KpA * ey + KdA * (dy - Speedy) + KIA * iey);
            }
        }

        if (-KpA * ex + KdA * (dx - Speedx) >= 1) {
            pitch = -asin(1);
            iex = 0;
        } else if (-KpA * ex + KdA * (dx - Speedx) <= -1) {
            pitch = -asin(-1);
            iex = 0;
        } else {
            if ((x - setPointX) * (x - setPointX) <= 1) {
                pitch = -asin(-KpA * ex * SmallKpA + KdA * (dx - Speedx) + KIA * iex);
            } else {
                pitch = -asin(-KpA * ex + KdA * (dx - Speedx) + KIA * iex);
            }
        }
        
        newCommand.thrust = 1 - 1 / (1 + exp(u+TH));
        newCommand.thrust = IsReal(newCommand.thrust);  // INF NAN check
        filterFront.push_front(u);
        if(filterFront.size()>filterSize){
          filterFront.pop_back(); 
        }
        double sum=0,out=0; 
        std::list<double>::iterator it =filterFront.begin();
        for (int i = 0; i <filterFront.size() ; i++) {
          sum += *it;
          std::advance(it, 1);
          out = sum/filterFront.size();
        }
        debugmsg.regoutTrust =out;
        // 0.95
        // ROS_INFO("U: %04.5f Z: %03.5f Com: %3.5f " , u,z,newCommand.thrust);
        // ROS_INFO("Kp: %f" , Kp);

        if (roll >= maxAngleRoll) {
            roll = maxAngleRoll;
            iey = 0;
        } else if (roll <= -maxAngleRoll) {
            roll = -maxAngleRoll;
            iey = 0;
        } else {
        }

        if (pitch >= maxAnglePitch) {
            pitch = maxAnglePitch;
            iex = 0;
        } else if (pitch <= -maxAnglePitch) {
            pitch = -maxAnglePitch;
            iex = 0;
        } else {
        }

        oldPitch = pitch;
        pitch = pitch * cos(yaw) - roll * sin(yaw);
        roll = roll * cos(yaw) + oldPitch * sin(yaw);
        yaw = yawD;

        // Nan check & Inf check
        pitch = IsReal(pitch);
        roll = IsReal(roll);
        yaw = IsReal(yaw);
        // ROS_INFO("P: %04.5f R: %03.5f Y: %3.5f U: %3.5f " , pitch,roll,yaw,newCommand.thrust);
        q_orig = tf::createQuaternionFromRPY(roll, pitch, yaw);
        // q_orig *= q_rot;  // Calculate the new orientation
        q_orig.normalize();
        quaternionTFToMsg(q_orig, newCommand.orientation);
        // Got it out!
        debugPub.publish(debugmsg);
        cmd_publisher.publish(newCommand);
        r.sleep();

    }
}

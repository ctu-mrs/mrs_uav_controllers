#ifndef MRS_UAV_CONTROLLERS_COMMON_H
#define MRS_UAV_CONTROLLERS_COMMON_H

#include <eigen3/Eigen/Eigen>
#include <mrs_uav_managers/common_handlers.h>
#include <mrs_uav_managers/controller.h>

namespace mrs_uav_controllers
{

namespace common
{

enum CONTROL_OUTPUT
{
  ACTUATORS_CMD,
  CONTROL_GROUP,
  ATTITUDE_RATE,
  ATTITUDE,
  ACCELERATION_HDG_RATE,
  ACCELERATION_HDG,
  VELOCITY_HDG_RATE,
  VELOCITY_HDG,
  POSITION
};

Eigen::Matrix3d so3transform(const Eigen::Vector3d& body_z, const ::Eigen::Vector3d& heading, const bool& preserve_heading);

std::optional<CONTROL_OUTPUT> getLowestOuput(const mrs_uav_managers::ControlOutputModalities_t& outputs);

std::optional<CONTROL_OUTPUT> getHighestOuput(const mrs_uav_managers::ControlOutputModalities_t& outputs);

/* throttle extraction //{ */

std::optional<double> extractThrottle(const mrs_uav_managers::Controller::ControlOutput& control_output);

struct HwApiCmdExtractThrottleVisitor
{
  std::optional<double> operator()(const mrs_msgs::HwApiActuatorCmd& msg) {

    std::optional<double> throttle = {};

    if (msg.motors.size() == 0) {
      return throttle;
    }

    throttle.value() = 0;

    for (size_t i = 0; i < msg.motors.size(); i++) {
      throttle.value() += msg.motors[i];
    };

    throttle.value() /= msg.motors.size();

    return throttle;
  }
  std::optional<double> operator()(const mrs_msgs::HwApiControlGroupCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const mrs_msgs::HwApiAttitudeCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const mrs_msgs::HwApiAttitudeRateCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgRateCmd& msg) {
    return {};
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgCmd& msg) {
    return {};
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgRateCmd& msg) {
    return {};
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgCmd& msg) {
    return {};
  }
  std::optional<double> operator()([[maybe_unused]] const mrs_msgs::HwApiPositionCmd& msg) {
    return {};
  }
};

//}

}  // namespace common

}  // namespace mrs_uav_controllers

#endif  // MRS_UAV_CONTROLLERS_COMMON_H

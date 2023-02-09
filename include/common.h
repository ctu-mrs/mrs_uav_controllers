#ifndef MRS_UAV_CONTROLLERS_COMMON_H
#define MRS_UAV_CONTROLLERS_COMMON_H

#include <eigen3/Eigen/Eigen>
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
  ACCELERATION,
  VELOCITY,
  POSITION
};

Eigen::Matrix3d so3transform(const Eigen::Vector3d& body_z, const ::Eigen::Vector3d& heading, const bool& preserve_heading);

std::optional<CONTROL_OUTPUT> getLowestOuput(const mrs_uav_managers::Controller::ControllerOutputs& outputs);

std::optional<CONTROL_OUTPUT> getHighestOuput(const mrs_uav_managers::Controller::ControllerOutputs& outputs);
}  // namespace common

}  // namespace mrs_uav_controllers

#endif  // MRS_UAV_CONTROLLERS_COMMON_H

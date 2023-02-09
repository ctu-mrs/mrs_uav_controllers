#include <common.h>

namespace mrs_uav_controllers
{

namespace common
{

/* so3transform() //{ */

Eigen::Matrix3d so3transform(const Eigen::Vector3d& body_z, const ::Eigen::Vector3d& heading, const bool& preserve_heading) {

  Eigen::Vector3d body_z_normed = body_z.normalized();

  Eigen::Matrix3d Rd;

  if (preserve_heading) {

    // | ------------------------- body z ------------------------- |
    Rd.col(2) = body_z_normed;

    // | ------------------------- body x ------------------------- |

    // construct the oblique projection
    Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - body_z_normed * body_z_normed.transpose());

    // create a basis of the body-z complement subspace
    Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
    A.col(0)          = projector_body_z_compl.col(0);
    A.col(1)          = projector_body_z_compl.col(1);

    // create the basis of the projection null-space complement
    Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
    B.col(0)          = Eigen::Vector3d(1, 0, 0);
    B.col(1)          = Eigen::Vector3d(0, 1, 0);

    // oblique projector to <range_basis>
    Eigen::MatrixXd Bt_A               = B.transpose() * A;
    Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
    Eigen::MatrixXd oblique_projector  = A * Bt_A_pseudoinverse * B.transpose();

    Rd.col(0) = oblique_projector * heading;
    Rd.col(0).normalize();

    // | ------------------------- body y ------------------------- |

    Rd.col(1) = Rd.col(2).cross(Rd.col(0));
    Rd.col(1).normalize();

  } else {

    Rd.col(2) = body_z_normed;
    Rd.col(1) = Rd.col(2).cross(heading);
    Rd.col(1).normalize();
    Rd.col(0) = Rd.col(1).cross(Rd.col(2));
    Rd.col(0).normalize();
  }

  return Rd;
}

//}

/* getLowestOutput() //{ */

std::optional<CONTROL_OUTPUT> getLowestOuput(const mrs_uav_managers::Controller::ControllerOutputs& outputs) {

  if (outputs.actuators) {
    return ACTUATORS_CMD;
  }

  if (outputs.control_group) {
    return CONTROL_GROUP;
  }

  if (outputs.attitude_rate) {
    return ATTITUDE_RATE;
  }

  if (outputs.attitude) {
    return ATTITUDE;
  }

  if (outputs.acceleration) {
    return ACCELERATION;
  }

  if (outputs.velocity) {
    return VELOCITY;
  }

  if (outputs.position) {
    return POSITION;
  }

  return {};
}

//}

/* getHighestOutput() //{ */

std::optional<CONTROL_OUTPUT> getHighestOuput(const mrs_uav_managers::Controller::ControllerOutputs& outputs) {

  if (outputs.position) {
    return POSITION;
  }

  if (outputs.velocity) {
    return VELOCITY;
  }

  if (outputs.acceleration) {
    return ACCELERATION;
  }

  if (outputs.attitude) {
    return ATTITUDE;
  }

  if (outputs.attitude_rate) {
    return ATTITUDE_RATE;
  }

  if (outputs.control_group) {
    return CONTROL_GROUP;
  }

  if (outputs.actuators) {
    return ACTUATORS_CMD;
  }

  return {};
}

//}

}  // namespace common

}  // namespace mrs_uav_controllers

#include <common.h>

namespace mrs_uav_controllers
{

namespace common
{

/* orientationError() //{ */

Eigen::Vector3d orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& Rd) {

  // orientation error
  Eigen::Matrix3d R_error = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  // vectorize the orientation error
  // clang-format off
    Eigen::Vector3d R_error_vec;
    R_error_vec << (R_error(1, 2) - R_error(2, 1)) / 2.0,
                   (R_error(2, 0) - R_error(0, 2)) / 2.0,
                   (R_error(0, 1) - R_error(1, 0)) / 2.0;
  // clang-format on

  return R_error_vec;
}

//}

/* sanitizeDesiredForce() //{ */

std::optional<Eigen::Vector3d> sanitizeDesiredForce(const Eigen::Vector3d& input, const double& tilt_safety_limit, const double& tilt_saturation,
                                                    const std::string& node_name) {

  // calculate the force in spherical coordinates
  double theta = acos(input[2]);
  double phi   = atan2(input[1], input[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {
    ROS_ERROR("[%s]: sanitizeDesiredForce(): NaN detected in variable 'theta', returning empty command", node_name.c_str());
    return {};
  }

  if (tilt_safety_limit > 1e-3 && std::abs(theta) > tilt_safety_limit) {

    ROS_ERROR("[%s]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", node_name.c_str(), (180.0 / M_PI) * theta,
              (180.0 / M_PI) * tilt_safety_limit);
    ROS_ERROR_STREAM("[" << node_name << "]: f = [" << input.transpose() << "]");

    return {};
  }

  // saturate the angle

  if (tilt_saturation > 1e-3 && std::abs(theta) > tilt_saturation) {
    ROS_WARN_THROTTLE(1.0, "[%s]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", node_name.c_str(), (theta / M_PI) * 180.0,
                      (tilt_saturation / M_PI) * 180.0);
    theta = tilt_saturation;
  }

  // reconstruct the force vector back out of the spherical coordinates
  Eigen::Vector3d output(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

  return {output};
}

//}

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

std::optional<CONTROL_OUTPUT> getLowestOuput(const mrs_uav_managers::ControlOutputModalities_t& outputs) {

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

  if (outputs.acceleration_hdg_rate) {
    return ACCELERATION_HDG_RATE;
  }

  if (outputs.acceleration_hdg) {
    return ACCELERATION_HDG;
  }

  if (outputs.velocity_hdg_rate) {
    return VELOCITY_HDG_RATE;
  }

  if (outputs.velocity_hdg) {
    return VELOCITY_HDG;
  }

  if (outputs.position) {
    return POSITION;
  }

  return {};
}

//}

/* getHighestOutput() //{ */

std::optional<CONTROL_OUTPUT> getHighestOuput(const mrs_uav_managers::ControlOutputModalities_t& outputs) {

  if (outputs.position) {
    return POSITION;
  }

  if (outputs.velocity_hdg) {
    return VELOCITY_HDG;
  }

  if (outputs.velocity_hdg_rate) {
    return VELOCITY_HDG_RATE;
  }

  if (outputs.acceleration_hdg) {
    return ACCELERATION_HDG;
  }

  if (outputs.acceleration_hdg_rate) {
    return ACCELERATION_HDG_RATE;
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

/* extractThrottle() //{ */

std::optional<double> extractThrottle(const mrs_uav_managers::Controller::ControlOutput& control_output) {

  if (!control_output.control_output) {
    return {};
  }

  return std::visit(HwApiCmdExtractThrottleVisitor(), control_output.control_output.value());
}

//}

/* attitudeController() //{ */

std::optional<mrs_msgs::HwApiAttitudeRateCmd> attitudeController(const mrs_msgs::UavState& uav_state, const mrs_msgs::HwApiAttitudeCmd& reference,
                                                                 const Eigen::Vector3d& ff_rate, const Eigen::Vector3d& rate_saturation,
                                                                 const Eigen::Vector3d& gains) {

  Eigen::Matrix3d R  = mrs_lib::AttitudeConverter(uav_state.pose.orientation);
  Eigen::Matrix3d Rd = mrs_lib::AttitudeConverter(reference.orientation);

  // calculate the orientation error
  Eigen::Vector3d E = common::orientationError(R, Rd);

  Eigen::Vector3d rate_feedback = gains.array() * E.array() + ff_rate.array();

  // | ----------- parasitic heading rate compensation ---------- |

  // compensate for the parasitic heading rate created by the desired pitch and roll rate
  Eigen::Vector3d rp_heading_rate_compensation = Eigen::Vector3d(0, 0, 0);

  Eigen::Vector3d q_feedback_yawless = rate_feedback;
  q_feedback_yawless(2)              = 0;  // nullyfy the effect of the original yaw feedback

  double parasitic_heading_rate = 0;

  try {
    parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeadingRate(q_feedback_yawless);
  }
  catch (...) {
    ROS_ERROR("[AttitudeController]: exception caught while calculating the parasitic heading rate!");
  }

  try {
    rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
  }
  catch (...) {
    ROS_ERROR("[AttitudeController]: exception caught while calculating the parasitic heading rate compensation!");
  }

  rate_feedback += rp_heading_rate_compensation;

  // | --------------- saturate the attitude rate --------------- |

  if (rate_feedback[0] > rate_saturation[0]) {
    rate_feedback[0] = rate_saturation[0];
  } else if (rate_feedback[0] < -rate_saturation[0]) {
    rate_feedback[0] = -rate_saturation[0];
  }

  if (rate_feedback[1] > rate_saturation[1]) {
    rate_feedback[1] = rate_saturation[1];
  } else if (rate_feedback[1] < -rate_saturation[1]) {
    rate_feedback[1] = -rate_saturation[1];
  }

  if (rate_feedback[2] > rate_saturation[2]) {
    rate_feedback[2] = rate_saturation[2];
  } else if (rate_feedback[2] < -rate_saturation[2]) {
    rate_feedback[2] = -rate_saturation[2];
  }

  // | ------------ fill in the attitude rate command ----------- |

  mrs_msgs::HwApiAttitudeRateCmd cmd;

  cmd.stamp = ros::Time::now();

  cmd.body_rate.x = rate_feedback[0];
  cmd.body_rate.y = rate_feedback[1];
  cmd.body_rate.z = rate_feedback[2];

  cmd.throttle = reference.throttle;

  return cmd;
}

//}

std::optional<mrs_msgs::HwApiControlGroupCmd> attitudeRateController(const mrs_msgs::UavState& uav_state, const Eigen::Vector3d& des_rate,
                                                                     const double& des_throttle, const Eigen::Vector3d& gains) {

}

}  // namespace common

}  // namespace mrs_uav_controllers

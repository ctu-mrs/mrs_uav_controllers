#ifndef PID_H
#define PID_H

#include <math.h>

namespace mrs_uav_controllers
{

class PIDController {

private:
  // | ----------------------- parameters ----------------------- |

  // gains
  double _kp_ = 0;  // proportional gain
  double _kd_ = 0;  // derivative gain
  double _ki_ = 0;  // integral gain

  // we remember the last control error, to calculate the difference
  double last_error_ = 0;
  double integral_   = 0;

  double saturation = -1;
  double antiwindup = -1;

public:
  PIDController();

  void setParams(const double &kp, const double &kd, const double &ki, const double &saturation, const double &antiwindup);

  void setSaturation(const double saturation = -1);

  void reset(void);

  double update(const double &error, const double &dt);
};

// --------------------------------------------------------------
// |                       implementation                       |
// --------------------------------------------------------------

PIDController::PIDController() {

  this->reset();
}

void PIDController::setParams(const double &kp, const double &kd, const double &ki, const double &saturation, const double &antiwindup) {

  this->_kp_       = kp;
  this->_kd_       = kd;
  this->_ki_       = ki;
  this->saturation = saturation;
  this->antiwindup = antiwindup;
}

void PIDController::setSaturation(const double saturation) {

  this->saturation = saturation;
}

void PIDController::reset(void) {

  this->last_error_ = 0;
  this->integral_   = 0;
}

double PIDController::update(const double &error, const double &dt) {

  // calculate the control error difference
  double difference = (error - last_error_) / dt;
  last_error_       = error;

  double p_component = _kp_ * error;       // proportional feedback
  double d_component = _kd_ * difference;  // derivative feedback
  double i_component = _ki_ * integral_;   // derivative feedback

  double sum = p_component + d_component + i_component;

  if (saturation > 0) {
    if (sum >= saturation) {
      sum = saturation;
    } else if (sum <= -saturation) {
      sum = -saturation;
    }
  }

  if (antiwindup > 0) {
    if (std::abs(sum) < antiwindup) {
      // add to the integral
      integral_ += error * dt;
    }
  }

  // return the summ of the components
  return sum;
}

}  // namespace mrs_uav_controllers

#endif  // PID_H

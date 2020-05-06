# MRS Controllers

[![Build Status](https://travis-ci.com/ctu-mrs/mrs_uav_controllers.svg?branch=master)](https://travis-ci.com/ctu-mrs/mrs_uav_controllers)

* *SO3 controller*
  * non-linear state feedback capable of precise reference tracking and fast manouvres
  * pros: Precise control, fast response, fast convergence
  * cons: Sensitive to measurement noise, requires feasible and smooth reference
  * _Lee, 2010, "Geometric tracking control of a quadrotor UAV on SE(3)"_
* *MPC controller*
  * nonlinear attitude and thrust control + Linear MPC for acceleration feedforward
  * pros: Robust control, immune to measurement noise and reference infeasibilities
  * cons: Slow convergence, only for slow speeds (< 2 m/s), may have large control errors while tracking motion
* NSF controller
  * obsolete nonlinear state feedback, performance-wise similar to the SO(3)
  * should not be used in practice
  * is kept around for educational and testing purposes
* Failsafe controller
  * feed-forward landing routing used to land without the used of a state estimator

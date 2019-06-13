# MRS Controllers

* *SO3 controller*
  * non-linear state feedback capable of precise reference tracking and fast manouvres
  * pros: Precise control, fast response, fast convergence
  * cons: Sensitive to measurement noise, requires feasible reference
* *MPC controller*
  * nonlinear attitude and thrust control + Linear MPC for acceleration feedforward
  * pros: Robust control, reliable to measurement noise and reference infeasibilities
  * cons: Slow convergence, only for slow speeds (< 2 m/s), may have large control errors
* AttitudeController
  * the attitude part of the SO3, used mainly for gain tuning with a gamepad
* NSF controller
  * obsolete nonlinear state feedback, performance-wise similar to SO(3)
  * should not be used
* FailsafeController
  * feed-forward landing routing used to land without the used of a state estimator

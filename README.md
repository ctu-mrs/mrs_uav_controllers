# MRS UAV Controllers [![Build Status](https://travis-ci.com/ctu-mrs/mrs_uav_controllers.svg?branch=master)](https://travis-ci.com/ctu-mrs/mrs_uav_controllers)

![](.fig/thumbnail.jpg)

## Purpose of a controller within the MRS control pipeline

* receiving an all-state reference from [reference trackers](https://github.com/ctu-mrs/mrs_uav_trackers)
* controlling the states of the UAV by outputting a **desired angular rate** (or **desired orientation**) and **desired thrust**

## Available controllers

* "SO(3) controller"
  * geometric state feedback in SE(3) capable of precise reference tracking and fast maneuvers
  * **pros**: precise control, fast response, fast convergence
  * **cons**: sensitive to measurement noise, requires feasible and smooth reference, needs to be tuned
  * originally published in: `Lee, et al., "Geometric tracking control of a quadrotor UAV on SE(3)", CDC 2010`, [link](https://ieeexplore.ieee.org/abstract/document/5717652)
* "MPC controller"
  * SO(3) force tracking + Linear MPC for acceleration feedforward
  * **pros**: robust control, immune to measurement noise and reference infeasibilities
  * **cons**: slow convergence, only for slow speeds (< 2 m/s), may have large control errors while tracking motion
  * briefly described in: `Petrlik, et al., "A Robust UAV System for Operations in a Constrained Environment", RA-L 2020`, [link](https://ieeexplore.ieee.org/abstract/document/8979150)
* "NSF controller"
  * nonlinear state feedback, obsolete 
  * should not be used on a real UAV
  * is kept for testing and educational purposes
  * outputs *desired attitude* only
* "Failsafe controller"
  * feedforward controller for landing without a state estimator
  * relies on the Pixhawk's attitude controller for leveling
  * is triggered in case of emergency
  * gradually decreases thrust while keeping the UAV leveled

## Controller interface

The controllers are compiled as *ROS plugins* ([http://wiki.ros.org/pluginlib](http://wiki.ros.org/pluginlib)) with the [interface](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/include/mrs_uav_managers/controller.h) defined by the [control manager](https://github.com/ctu-mrs/mrs_uav_managers).
A controller from any ROS package can be loaded dynamically by the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) without it being present during [control manager](https://github.com/ctu-mrs/mrs_uav_managers)'s compile time.
Loaded controllers can be switched by the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) in mid-flight, which allows safe testing of new controllers and adds flexibility the [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system).

## Loading controllers to [control manager](https://github.com/ctu-mrs/mrs_uav_managers)

Controllers are defined in `controller.yaml` ([example](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/controllers.yaml)).
Each entry such as
```yaml
So3Controller:
  address: "mrs_uav_controllers/So3Controller"
  namespace: "so3_controller"
  eland_threshold: 1.5 # [m], position error triggering eland
  failsafe_threshold: 2.5 # [m], position error triggering failsafe land
  odometry_innovation_threshold: 1.5 # [m], position odometry innovation threshold
```
creates an instance of a controller, in this case `mrs_uav_controllers/So3Controller` is loaded under the *alias* `So3Controller`.
Multiple instances are allowed and are used to introduce the same controller with various configurations that can be switched in mid-flight.
Once the controller alias is defined within `controllers.yaml`, it needs to be part of the *controllers* list within `control_manager.yaml` ([example](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/control_manager.yaml)) config:
```yaml
# - list of names of dynamically loaded controllers
controllers : [
  "So3Controller",
  "MpcController",
  "FailsafeController",
  "EmergencyController",
]
```
Only the controllers within this list are actually loaded.
Switching to a controller with the alias *So3Controller* is done by calling a service:
```bash
rosservice call /uav1/control_manager/switch_controller So3Controller
```

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2019-07-01)
------------------
* added feedback disablation to mpc controller during takeoff
* added desired acceleration to outputs
* increased failsafe thrust constants
* fixed the mass antiwindup in MPC
* constraints are passed to controllers
* mpc now controls the z axis
* added negative z-force detection and flip mitigation
* fixed body integral in so3, cleaned configs
* upgraded so3's max tilt saturation and failsafe (Naki's accident)
* fixed body rate orientation for new Mavros
* + Mpc controller
* fixed attitude rate reference frame
* Contributors: Tomas Baca

0.0.1 (2019-05-20)
------------------

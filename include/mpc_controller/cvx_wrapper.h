#ifndef CVX_WRAPPER
#define CVX_WRAPPER

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

/* author: Daniel Hert */

namespace mrs_controllers
{

namespace mpc_controller
{

class CvxWrapperX {

public:
  CvxWrapperX(bool verbose, int max_iters, std::vector<double> tempQ, std::vector<double> tempQ_last, double dt, double dt2);

  void   setInitialState(Eigen::MatrixXd &x);
  void   loadReference(Eigen::MatrixXd &reference);
  void   setLimits(double max_speed, double max_acc, double max_acc_slew, double dt1, double dt2);
  void   setDt(double dt, double dt2);
  int    solveCvx();
  void   getStates(Eigen::MatrixXd &future_traj);
  double getFirstControlInput();

private:
  static const int    horizon_len = 26;
  std::vector<double> myQ;
  std::vector<double> myQ_last;
  int                 dim;
  int                 vel_q_persistent;
};

class CvxWrapperY {

public:
  CvxWrapperY(bool verbose, int max_iters, std::vector<double> tempQ, std::vector<double> tempQ_last, double dt, double dt2);

  void   setInitialState(Eigen::MatrixXd &x);
  void   loadReference(Eigen::MatrixXd &reference);
  void   setLimits(double max_speed, double max_acc, double max_acc_slew, double dt1, double dt2);
  void   setDt(double dt, double dt2);
  int    solveCvx();
  void   getStates(Eigen::MatrixXd &future_traj);
  double getFirstControlInput();

private:
  static const int    horizon_len = 26;
  std::vector<double> myQ;
  std::vector<double> myQ_last;
  int                 dim;
  int                 vel_q_persistent;
};

}  // namespace mpc_controller

}  // namespace mrs_controllers

#endif

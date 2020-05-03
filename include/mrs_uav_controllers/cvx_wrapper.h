#ifndef CVX_WRAPPER_CONTROLLER
#define CVX_WRAPPER_CONTROLLER

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mutex>

/* author: Daniel Hert */

namespace mrs_uav_controllers
{

namespace cvx_wrapper
{

class CvxWrapper {

public:
  CvxWrapper(bool verbose, int max_iters, std::vector<double> Q, std::vector<double> Q_last, double dt1, double dt2, double p1, double p2);

  void   setInitialState(Eigen::MatrixXd &x);
  void   loadReference(Eigen::MatrixXd &reference);
  void   setLimits(double max_speed, double max_acc, double max_u, double max_du, double dt1, double dt2);
  void   setDt(double dt, double dt2);
  int    solveCvx();
  void   getStates(Eigen::MatrixXd &future_traj);
  void   setQ(const std::vector<double> new_Q);
  void   setS(const std::vector<double> new_S);
  double getFirstControlInput();
  void   setLastInput(double last_input);
  void   setParams(void);
  void   lock(void);
  void   unlock(void);

private:
  std::vector<double> Q;
  std::vector<double> Q_last;

  static const int horizon_len = 26;

  double dt1, dt2;
  double p1, p2;
  int    dim;
  int    vel_q_persistent;
  bool   verbose;
  int    max_iters;

  static std::mutex mutex_main;
};

}  // namespace cvx_wrapper

}  // namespace mrs_uav_controllers

#endif

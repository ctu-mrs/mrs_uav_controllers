#ifndef CVX_WRAPPER
#define CVX_WRAPPER

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

/* author: Daniel Hert */

namespace mrs_controllers
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
  double getFirstControlInput();
  void   setLastInput(double last_input);
  void   setParams(void);

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
};

}  // namespace cvx_wrapper

}  // namespace mrs_controllers

#endif

#ifndef MPC_CONTROLLER_SOLVER
#define MPC_CONTROLLER_SOLVER

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <mutex>

namespace mrs_mpc_solvers
{

namespace mpc_controller
{

class Solver {

public:
  Solver(std::string name, bool verbose, int max_iters, std::vector<double> Q, std::vector<double> Q_last, double dt1, double dt2, double p1, double p2);

  void   setInitialState(Eigen::MatrixXd &x);
  void   loadReference(Eigen::MatrixXd &reference);
  void   setLimits(double max_speed, double max_acc, double max_u, double max_du, double dt1, double dt2);
  void   setDt(double dt, double dt2);
  int    solveMPC();
  void   getStates(Eigen::MatrixXd &future_traj);
  void   setQ(const std::vector<double> new_Q);
  void   setS(const std::vector<double> new_S);
  double getFirstControlInput();
  void   setLastInput(double last_input);
  void   setParams(void);
  void   lock(void);
  void   unlock(void);

private:

  std::string _name_;

  std::vector<double> Q_;
  std::vector<double> Q_last_;

  static const int _horizon_len_ = 26;

  double dt1_, dt2_;
  double p1_, p2_;
  int    _dim_;
  int    vel_q_persistent_;
  bool   _verbose_;
  int    _max_iters_;

  static std::mutex mutex_main_;
};

}  // namespace mpc_controller

}  // namespace mrs_mpc_solvers

#endif

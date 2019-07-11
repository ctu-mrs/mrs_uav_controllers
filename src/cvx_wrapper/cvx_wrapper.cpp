/* author: Daniel Hert */

#include <eigen3/Eigen/Eigen>
#include <cvx_wrapper.h>

using namespace Eigen;

namespace mrs_controllers
{

namespace cvx_wrapper
{

extern "C" {
#include "cvxgen/solver.h"
}

VarsController      varsController;
ParamsController    paramsController;
WorkspaceController workController;
SettingsController  settingsController;

/* CvxWrapper() //{ */

CvxWrapper::CvxWrapper(bool verbose, int max_iters, std::vector<double> Q, std::vector<double> Q_last, double dt1, double dt2, double p1, double p2) {

  this->Q         = Q;
  this->Q_last    = Q_last;
  this->verbose   = verbose;
  this->max_iters = max_iters;
  this->dt1       = dt1;
  this->dt2       = dt2;
  this->p1        = p1;
  this->p2        = p2;

  paramsController.u_last[0] = 0;

  set_defaults_controller();
  setup_indexing_controller();
  setup_indexed_optvarsController_controller();

  setParams();

  ROS_INFO("Cvx wrapper initiated");
}

//}

/* setParams() //{ */

void CvxWrapper::setParams(void) {

  settingsController.verbose   = this->verbose;
  settingsController.max_iters = this->max_iters;

  for (int i = 0; i < 3; i++) {
    paramsController.Q[i] = Q[i];
  }

  for (int i = 0; i < 3; i++) {
    paramsController.Q_last[i] = Q_last[i];
  }

  paramsController.Af[0] = 1;
  paramsController.Af[1] = 1;
  paramsController.Af[2] = p1;
  paramsController.Af[3] = dt1;
  paramsController.Af[4] = dt1;

  paramsController.Bf[0] = p2;

  paramsController.A[0] = 1;
  paramsController.A[1] = 1;
  paramsController.A[2] = p1;
  paramsController.A[3] = dt2;
  paramsController.A[4] = dt2;

  paramsController.B[0] = p2;
}

//}

/* setLimits() //{ */

void CvxWrapper::setLimits(double max_speed, double max_acc, double max_u, double max_du, double dt1, double dt2) {

  paramsController.x_max_2[0]  = max_speed;
  paramsController.x_max_3[0]  = max_acc;
  paramsController.u_max[0]    = max_u;
  paramsController.du_max_f[0] = max_du * dt1;
  paramsController.du_max[0]   = max_du * dt2;
}

//}

/* setLastInput() //{ */

void CvxWrapper::setLastInput(double last_input) {

  paramsController.u_last[0] = last_input;
}

//}

/* setDt() //{ */

void CvxWrapper::setDt(double dt1, double dt2) {

  paramsController.Af[2] = dt1;
  paramsController.Af[3] = dt1;

  paramsController.A[2] = dt2;
  paramsController.A[3] = dt2;
}

//}

/* setInitialState() //{ */

void CvxWrapper::setInitialState(MatrixXd& x) {

  paramsController.x_0[0] = x(0, 0);
  paramsController.x_0[1] = x(1, 0);
  paramsController.x_0[2] = x(2, 0);
}

//}

/* loadReference() //{ */

void CvxWrapper::loadReference(MatrixXd& reference) {

  for (int i = 0; i < horizon_len; i++) {

    paramsController.x_ss[i + 1][0] = reference((3 * i) + 0, 0);
    paramsController.x_ss[i + 1][1] = reference((3 * i) + 1, 0);
    paramsController.x_ss[i + 1][2] = reference((3 * i) + 2, 0);
  }
}

//}

/* setQ() //{ */

void CvxWrapper::setQ(const std::vector<double> new_Q) {

  for (int i = 0; i < horizon_len; i++) {

    this->Q = new_Q;
  }
}

//}

/* setS() //{ */

void CvxWrapper::setS(const std::vector<double> new_S) {

  for (int i = 0; i < horizon_len; i++) {

    this->Q_last = new_S;
  }
}

//}

/* solveCvx() //{ */

int CvxWrapper::solveCvx() {

  return solve_controller();
}
//}

/* getStates() //{ */

void CvxWrapper::getStates(MatrixXd& future_traj) {

  for (int i = 0; i < horizon_len; i++) {

    future_traj(0 + (i * 3)) = *(varsController.x[i + 1]);
    future_traj(1 + (i * 3)) = *(varsController.x[i + 1] + 1);
    future_traj(2 + (i * 3)) = *(varsController.x[i + 1] + 2);
  }
}

//}

/* getFirstControlInput() //{ */

double CvxWrapper::getFirstControlInput() {

  return *(varsController.u_0);
}

//}

}  // namespace cvx_wrapper

}  // namespace mrs_controllers

/* author: Daniel Hert */

#include <eigen3/Eigen/Eigen>
#include <cvx_wrapper.h>

using namespace Eigen;

namespace mrs_controllers
{

namespace mpc_controller
{

extern "C" {
#include "cvxgen_x/solver.h"
}

VarsControllerX      varsControllerX;
ParamsControllerX    paramsControllerX;
WorkspaceControllerX workControllerX;
SettingsControllerX  settingsControllerX;

/* CvxWrapperX() //{ */

CvxWrapperX::CvxWrapperX(bool verbose, int max_iters, std::vector<double> tempQ, std::vector<double> tempQ_last, double dt1, double dt2) {

  myQ      = std::vector<double>(3);
  myQ_last = std::vector<double>(3);
  set_defaults_controllerX();
  setup_indexing_controllerX();
  setup_indexed_optvarsControllerX_controllerX();

  settingsControllerX.verbose   = verbose;
  settingsControllerX.max_iters = max_iters;
  paramsControllerX.u_last[0]   = 0;

  for (int i = 0; i < 3; i++) {
    paramsControllerX.Q[i] = tempQ[i];
  }

  for (int i = 0; i < 3; i++) {
    paramsControllerX.Q_last[i] = tempQ_last[i];
  }

  paramsControllerX.Af[0] = 1;
  paramsControllerX.Af[1] = 1;
  paramsControllerX.Af[2] = dt1;
  paramsControllerX.Af[3] = dt1;

  paramsControllerX.Bf[0] = 1;

  paramsControllerX.A[0] = 1;
  paramsControllerX.A[1] = 1;
  paramsControllerX.A[2] = dt2;
  paramsControllerX.A[3] = dt2;

  paramsControllerX.B[0] = 1;

  ROS_INFO("Cvx wrapper initiated");
}

//}

/* setLimits() //{ */

void CvxWrapperX::setLimits(double max_speed, double max_acc, double max_acc_slew, double dt1, double dt2) {

  paramsControllerX.x_max_1[0]  = max_speed;
  paramsControllerX.u_max[0]    = max_acc;
  paramsControllerX.du_max_f[0] = max_acc_slew * dt1;
  paramsControllerX.du_max[0]   = max_acc_slew * dt2;
}

//}

/* setDt() //{ */

void CvxWrapperX::setDt(double dt1, double dt2) {

  paramsControllerX.Af[2] = dt1;
  paramsControllerX.Af[3] = dt1;

  paramsControllerX.A[2] = dt2;
  paramsControllerX.A[3] = dt2;
}

//}

/* setInitialState() //{ */

void CvxWrapperX::setInitialState(MatrixXd& x) {

  paramsControllerX.x_0[0] = x(0, 0);
  paramsControllerX.x_0[1] = x(1, 0);
  paramsControllerX.x_0[2] = x(2, 0);
}

//}

/* loadReference() //{ */

void CvxWrapperX::loadReference(MatrixXd& reference) {

  for (int i = 0; i < horizon_len; i++) {

    paramsControllerX.x_ss[i + 1][0] = reference((3 * i) + 0, 0);
    paramsControllerX.x_ss[i + 1][1] = reference((3 * i) + 1, 0);
    paramsControllerX.x_ss[i + 1][2] = reference((3 * i) + 2, 0);
  }
}

//}

/* solveCvx() //{ */

int CvxWrapperX::solveCvx() {

  paramsControllerX.u_last[0] = *(varsControllerX.u_0);

  return solve_controllerX();
}
//}

/* getStates() //{ */

void CvxWrapperX::getStates(MatrixXd& future_traj) {

  for (int i = 0; i < horizon_len; i++) {
    future_traj(0 + (i * 3)) = *(varsControllerX.x[i + 1]);
    future_traj(1 + (i * 3)) = *(varsControllerX.x[i + 1] + 1);
    future_traj(2 + (i * 3)) = *(varsControllerX.x[i + 1] + 2);
  }
}

//}

/* getFirstControlInput() //{ */

double CvxWrapperX::getFirstControlInput() {

  return *(varsControllerX.u_0);
}

//}

}  // namespace mpc_controller

}  // namespace mrs_controllers

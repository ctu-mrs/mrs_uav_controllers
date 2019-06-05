/* author: Daniel Hert */

#include <eigen3/Eigen/Eigen>
#include <cvx_wrapper.h>

using namespace Eigen;

namespace mrs_controllers
{

namespace mpc_controller
{

extern "C" {
#include "cvxgen_y/solver.h"
}

VarsControllerY      varsControllerY;
ParamsControllerY    paramsControllerY;
WorkspaceControllerY workControllerY;
SettingsControllerY  settingsControllerY;

/* CvxWrapperY() //{ */

CvxWrapperY::CvxWrapperY(bool verbose, int max_iters, std::vector<double> tempQ, std::vector<double> tempQ_last, double dt1, double dt2) {

  myQ      = std::vector<double>(3);
  myQ_last = std::vector<double>(3);
  set_defaults_controllerY();
  setup_indexing_controllerY();
  setup_indexed_optvarsControllerY_controllerY();

  settingsControllerY.verbose   = verbose;
  settingsControllerY.max_iters = max_iters;
  paramsControllerY.u_last[0]   = 0;

  for (int i = 0; i < 3; i++) {
    paramsControllerY.Q[i] = tempQ[i];
  }

  for (int i = 0; i < 3; i++) {
    paramsControllerY.Q_last[i] = tempQ_last[i];
  }
  // Af, Bf - first step

  paramsControllerY.Af[0] = 1;
  paramsControllerY.Af[1] = 1;
  paramsControllerY.Af[2] = dt1;
  paramsControllerY.Af[3] = dt1;

  paramsControllerY.Bf[0] = 1;

  paramsControllerY.A[0] = 1;
  paramsControllerY.A[1] = 1;
  paramsControllerY.A[2] = dt2;
  paramsControllerY.A[3] = dt2;

  paramsControllerY.B[0] = 1;

  ROS_INFO("Cvx wrapper initiated");
}

//}

/* setLimits() //{ */

void CvxWrapperY::setLimits(double max_speed, double max_acc, double max_acc_slew, double dt1, double dt2) {

  paramsControllerY.x_max_1[0]  = max_speed;
  paramsControllerY.u_max[0]    = max_acc;
  paramsControllerY.du_max_f[0] = max_acc_slew * dt1;
  paramsControllerY.du_max[0]   = max_acc_slew * dt2;
}

//}

/* setDt() //{ */

void CvxWrapperY::setDt(double dt1, double dt2) {

  paramsControllerY.Af[2] = dt1;
  paramsControllerY.Af[3] = dt1;

  paramsControllerY.A[2] = dt2;
  paramsControllerY.A[3] = dt2;
}

//}

/* setInitialState() //{ */

void CvxWrapperY::setInitialState(MatrixXd& x) {

  paramsControllerY.x_0[0] = x(0, 0);
  paramsControllerY.x_0[1] = x(1, 0);
  paramsControllerY.x_0[2] = x(2, 0);
}

//}

/* loadReference() //{ */

void CvxWrapperY::loadReference(MatrixXd& reference) {

  for (int i = 0; i < horizon_len; i++) {

    paramsControllerY.x_ss[i + 1][0] = reference((3 * i) + 0, 0);
    paramsControllerY.x_ss[i + 1][1] = reference((3 * i) + 1, 0);
    paramsControllerY.x_ss[i + 1][2] = reference((3 * i) + 2, 0);
  }
}

//}

/* solveCvx() //{ */

int CvxWrapperY::solveCvx() {

  paramsControllerY.u_last[0] = *(varsControllerY.u_0);

  return solve_controllerY();
}
//}

/* getStates() //{ */

void CvxWrapperY::getStates(MatrixXd& future_traj) {

  for (int i = 0; i < horizon_len; i++) {

    future_traj(0 + (i * 3)) = *(varsControllerY.x[i + 1]);
    future_traj(1 + (i * 3)) = *(varsControllerY.x[i + 1] + 1);
    future_traj(2 + (i * 3)) = *(varsControllerY.x[i + 1] + 2);
  }
}

//}

/* getFirstControlInput() //{ */

double CvxWrapperY::getFirstControlInput() {

  return *(varsControllerY.u_0);
}

//}

}  // namespace mpc_controller

}  // namespace mrs_controllers

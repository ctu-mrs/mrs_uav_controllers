/* author: Daniel Hert */

extern "C" {
#include "cvxgen/solver.h"
}
#include <eigen3/Eigen/Eigen>
#include <cvx_wrapper.h>

using namespace Eigen;

Vars      vars;
Params    params;
Workspace work;
Settings  settings;

/* CvxWrapper() //{ */

CvxWrapper::CvxWrapper(bool verbose, int max_iters, std::vector<double> tempQ, std::vector<double> tempQ_last, double dt, double dt2) {

  myQ      = std::vector<double>(3);
  myQ_last = std::vector<double>(3);
  set_defaults();
  setup_indexing();
  setup_indexed_params();

  settings.verbose   = verbose;
  settings.max_iters = max_iters;
  params.u_last[0]   = 0;

  for (int i = 0; i < 3; i++) {
    params.Q[i] = tempQ[i];
  }

  for (int i = 0; i < 3; i++) {
    params.Q_last[i] = tempQ_last[i];
  }
  // Af, Bf - first step

  params.A[0] = 1;
  params.A[1] = 1;
  params.A[2] = dt2;
  params.A[3] = dt2;

  params.B[0] = 0;

  params.Af[0] = 1;
  params.Af[1] = 1;
  params.Af[2] = dt;
  params.Af[3] = dt;

  params.Bf[0] = dt;

  ROS_INFO("Cvx wrapper initiated");
}

//}

/* setLimits() //{ */

void CvxWrapper::setLimits(double max_speed, double max_acc, double max_acc_slew) {
  params.x_max_1[0] = max_speed;
  params.u_max[0]   = max_acc;
  params.du_max[0]  = max_acc_slew;
}

//}

/* setDt() //{ */

void CvxWrapper::setDt(double dt, double dt2) {
  params.A[2]  = dt2;
  params.A[3]  = dt2;
  params.Af[2] = dt;
  params.Af[3] = dt;
}

//}

/* setInitialState() //{ */

void CvxWrapper::setInitialState(MatrixXd& x) {
  params.x_0[0] = x(0, 0);
  params.x_0[1] = x(1, 0);
  params.x_0[2] = x(2, 0);
}

//}

/* loadReference() //{ */

void CvxWrapper::loadReference(MatrixXd& reference) {
  for (int i = 0; i < horizon_len; i++) {
    *params.x_ss[i + 1] = reference(i, 0);
    // Tohle je mozna blbe
    *(params.x_ss[i + 1] + 1) = reference(i, 1);
    *(params.x_ss[i + 1] + 2) = reference(i, 2);
  }
}

//}

/* solveCvx //{ */

int CvxWrapper::solveCvx() {
  return solve();
  params.u_last[0] = *(vars.u_0);
}
//}

/* getStates() //{ */

void CvxWrapper::getStates(MatrixXd& future_traj) {
  for (int i = 0; i < horizon_len; i++) {
    future_traj(0 + (i * 3)) = *(vars.x[i + 1]);
    future_traj(1 + (i * 3)) = *(vars.x[i + 1] + 1);
    future_traj(2 + (i * 3)) = *(vars.x[i + 1] + 2);
  }
}

//}

/* getFirstControlInput() //{ */

double CvxWrapper::getFirstControlInput() {
  return *(vars.u_0);
}

//}

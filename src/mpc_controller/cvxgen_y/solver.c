/* Produced by CVXGEN, 2019-06-05 08:31:52 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap_controllerY(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 234; i++)
    gap += workControllerY.z[i]*workControllerY.s[i];
  return gap;
}
void set_defaults_controllerY(void) {
  settingsControllerY.resid_tol = 1e-6;
  settingsControllerY.eps = 1e-4;
  settingsControllerY.max_iters = 25;
  settingsControllerY.refine_steps = 1;
  settingsControllerY.s_init = 1;
  settingsControllerY.z_init = 1;
  settingsControllerY.debug = 0;
  settingsControllerY.verbose = 1;
  settingsControllerY.verbose_refinement = 0;
  settingsControllerY.better_start = 1;
  settingsControllerY.kkt_reg = 1e-7;
}
void setup_pointers_controllerY(void) {
  workControllerY.y = workControllerY.x + 182;
  workControllerY.s = workControllerY.x + 260;
  workControllerY.z = workControllerY.x + 494;
  varsControllerY.u_0 = workControllerY.x + 78;
  varsControllerY.u_1 = workControllerY.x + 79;
  varsControllerY.u_2 = workControllerY.x + 80;
  varsControllerY.u_3 = workControllerY.x + 81;
  varsControllerY.u_4 = workControllerY.x + 82;
  varsControllerY.u_5 = workControllerY.x + 83;
  varsControllerY.u_6 = workControllerY.x + 84;
  varsControllerY.u_7 = workControllerY.x + 85;
  varsControllerY.u_8 = workControllerY.x + 86;
  varsControllerY.u_9 = workControllerY.x + 87;
  varsControllerY.u_10 = workControllerY.x + 88;
  varsControllerY.u_11 = workControllerY.x + 89;
  varsControllerY.u_12 = workControllerY.x + 90;
  varsControllerY.u_13 = workControllerY.x + 91;
  varsControllerY.u_14 = workControllerY.x + 92;
  varsControllerY.u_15 = workControllerY.x + 93;
  varsControllerY.u_16 = workControllerY.x + 94;
  varsControllerY.u_17 = workControllerY.x + 95;
  varsControllerY.u_18 = workControllerY.x + 96;
  varsControllerY.u_19 = workControllerY.x + 97;
  varsControllerY.u_20 = workControllerY.x + 98;
  varsControllerY.u_21 = workControllerY.x + 99;
  varsControllerY.u_22 = workControllerY.x + 100;
  varsControllerY.u_23 = workControllerY.x + 101;
  varsControllerY.u_24 = workControllerY.x + 102;
  varsControllerY.u_25 = workControllerY.x + 103;
  varsControllerY.x_1 = workControllerY.x + 104;
  varsControllerY.x_2 = workControllerY.x + 107;
  varsControllerY.x_3 = workControllerY.x + 110;
  varsControllerY.x_4 = workControllerY.x + 113;
  varsControllerY.x_5 = workControllerY.x + 116;
  varsControllerY.x_6 = workControllerY.x + 119;
  varsControllerY.x_7 = workControllerY.x + 122;
  varsControllerY.x_8 = workControllerY.x + 125;
  varsControllerY.x_9 = workControllerY.x + 128;
  varsControllerY.x_10 = workControllerY.x + 131;
  varsControllerY.x_11 = workControllerY.x + 134;
  varsControllerY.x_12 = workControllerY.x + 137;
  varsControllerY.x_13 = workControllerY.x + 140;
  varsControllerY.x_14 = workControllerY.x + 143;
  varsControllerY.x_15 = workControllerY.x + 146;
  varsControllerY.x_16 = workControllerY.x + 149;
  varsControllerY.x_17 = workControllerY.x + 152;
  varsControllerY.x_18 = workControllerY.x + 155;
  varsControllerY.x_19 = workControllerY.x + 158;
  varsControllerY.x_20 = workControllerY.x + 161;
  varsControllerY.x_21 = workControllerY.x + 164;
  varsControllerY.x_22 = workControllerY.x + 167;
  varsControllerY.x_23 = workControllerY.x + 170;
  varsControllerY.x_24 = workControllerY.x + 173;
  varsControllerY.x_25 = workControllerY.x + 176;
  varsControllerY.x_26 = workControllerY.x + 179;
}
void setup_indexed_paramsControllerY_controllerY(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  paramsControllerY.x_ss[1] = paramsControllerY.x_ss_1;
  paramsControllerY.x_ss[2] = paramsControllerY.x_ss_2;
  paramsControllerY.x_ss[3] = paramsControllerY.x_ss_3;
  paramsControllerY.x_ss[4] = paramsControllerY.x_ss_4;
  paramsControllerY.x_ss[5] = paramsControllerY.x_ss_5;
  paramsControllerY.x_ss[6] = paramsControllerY.x_ss_6;
  paramsControllerY.x_ss[7] = paramsControllerY.x_ss_7;
  paramsControllerY.x_ss[8] = paramsControllerY.x_ss_8;
  paramsControllerY.x_ss[9] = paramsControllerY.x_ss_9;
  paramsControllerY.x_ss[10] = paramsControllerY.x_ss_10;
  paramsControllerY.x_ss[11] = paramsControllerY.x_ss_11;
  paramsControllerY.x_ss[12] = paramsControllerY.x_ss_12;
  paramsControllerY.x_ss[13] = paramsControllerY.x_ss_13;
  paramsControllerY.x_ss[14] = paramsControllerY.x_ss_14;
  paramsControllerY.x_ss[15] = paramsControllerY.x_ss_15;
  paramsControllerY.x_ss[16] = paramsControllerY.x_ss_16;
  paramsControllerY.x_ss[17] = paramsControllerY.x_ss_17;
  paramsControllerY.x_ss[18] = paramsControllerY.x_ss_18;
  paramsControllerY.x_ss[19] = paramsControllerY.x_ss_19;
  paramsControllerY.x_ss[20] = paramsControllerY.x_ss_20;
  paramsControllerY.x_ss[21] = paramsControllerY.x_ss_21;
  paramsControllerY.x_ss[22] = paramsControllerY.x_ss_22;
  paramsControllerY.x_ss[23] = paramsControllerY.x_ss_23;
  paramsControllerY.x_ss[24] = paramsControllerY.x_ss_24;
  paramsControllerY.x_ss[25] = paramsControllerY.x_ss_25;
  paramsControllerY.x_ss[26] = paramsControllerY.x_ss_26;
  paramsControllerY.x[0] = paramsControllerY.x_0;
}
void setup_indexed_optvarsControllerY_controllerY(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  varsControllerY.x[1] = varsControllerY.x_1;
  varsControllerY.x[2] = varsControllerY.x_2;
  varsControllerY.x[3] = varsControllerY.x_3;
  varsControllerY.x[4] = varsControllerY.x_4;
  varsControllerY.x[5] = varsControllerY.x_5;
  varsControllerY.x[6] = varsControllerY.x_6;
  varsControllerY.x[7] = varsControllerY.x_7;
  varsControllerY.x[8] = varsControllerY.x_8;
  varsControllerY.x[9] = varsControllerY.x_9;
  varsControllerY.x[10] = varsControllerY.x_10;
  varsControllerY.x[11] = varsControllerY.x_11;
  varsControllerY.x[12] = varsControllerY.x_12;
  varsControllerY.x[13] = varsControllerY.x_13;
  varsControllerY.x[14] = varsControllerY.x_14;
  varsControllerY.x[15] = varsControllerY.x_15;
  varsControllerY.x[16] = varsControllerY.x_16;
  varsControllerY.x[17] = varsControllerY.x_17;
  varsControllerY.x[18] = varsControllerY.x_18;
  varsControllerY.x[19] = varsControllerY.x_19;
  varsControllerY.x[20] = varsControllerY.x_20;
  varsControllerY.x[21] = varsControllerY.x_21;
  varsControllerY.x[22] = varsControllerY.x_22;
  varsControllerY.x[23] = varsControllerY.x_23;
  varsControllerY.x[24] = varsControllerY.x_24;
  varsControllerY.x[25] = varsControllerY.x_25;
  varsControllerY.x[26] = varsControllerY.x_26;
  varsControllerY.u[0] = varsControllerY.u_0;
  varsControllerY.u[1] = varsControllerY.u_1;
  varsControllerY.u[2] = varsControllerY.u_2;
  varsControllerY.u[3] = varsControllerY.u_3;
  varsControllerY.u[4] = varsControllerY.u_4;
  varsControllerY.u[5] = varsControllerY.u_5;
  varsControllerY.u[6] = varsControllerY.u_6;
  varsControllerY.u[7] = varsControllerY.u_7;
  varsControllerY.u[8] = varsControllerY.u_8;
  varsControllerY.u[9] = varsControllerY.u_9;
  varsControllerY.u[10] = varsControllerY.u_10;
  varsControllerY.u[11] = varsControllerY.u_11;
  varsControllerY.u[12] = varsControllerY.u_12;
  varsControllerY.u[13] = varsControllerY.u_13;
  varsControllerY.u[14] = varsControllerY.u_14;
  varsControllerY.u[15] = varsControllerY.u_15;
  varsControllerY.u[16] = varsControllerY.u_16;
  varsControllerY.u[17] = varsControllerY.u_17;
  varsControllerY.u[18] = varsControllerY.u_18;
  varsControllerY.u[19] = varsControllerY.u_19;
  varsControllerY.u[20] = varsControllerY.u_20;
  varsControllerY.u[21] = varsControllerY.u_21;
  varsControllerY.u[22] = varsControllerY.u_22;
  varsControllerY.u[23] = varsControllerY.u_23;
  varsControllerY.u[24] = varsControllerY.u_24;
  varsControllerY.u[25] = varsControllerY.u_25;
}
void setup_indexing_controllerY(void) {
  setup_pointers_controllerY();
  setup_indexed_paramsControllerY_controllerY();
  setup_indexed_optvarsControllerY_controllerY();
}
void set_start_controllerY(void) {
  int i;
  for (i = 0; i < 182; i++)
    workControllerY.x[i] = 0;
  for (i = 0; i < 78; i++)
    workControllerY.y[i] = 0;
  for (i = 0; i < 234; i++)
    workControllerY.s[i] = (workControllerY.h[i] > 0) ? workControllerY.h[i] : settingsControllerY.s_init;
  for (i = 0; i < 234; i++)
    workControllerY.z[i] = settingsControllerY.z_init;
}
double eval_objv_controllerY(void) {
  int i;
  double objv;
  /* Borrow space in workControllerY.rhs. */
  multbyP_controllerY(workControllerY.rhs, workControllerY.x);
  objv = 0;
  for (i = 0; i < 182; i++)
    objv += workControllerY.x[i]*workControllerY.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 182; i++)
    objv += workControllerY.q[i]*workControllerY.x[i];
  objv += workControllerY.quad_600568381440[0]+workControllerY.quad_898851794944[0]+workControllerY.quad_88433618944[0]+workControllerY.quad_240204779520[0]+workControllerY.quad_635618762752[0]+workControllerY.quad_732753989632[0]+workControllerY.quad_427523055616[0]+workControllerY.quad_976046530560[0]+workControllerY.quad_688550678528[0]+workControllerY.quad_304816418816[0]+workControllerY.quad_819339411456[0]+workControllerY.quad_101800079360[0]+workControllerY.quad_976903761920[0]+workControllerY.quad_141299838976[0]+workControllerY.quad_343404097536[0]+workControllerY.quad_815806124032[0]+workControllerY.quad_997002137600[0]+workControllerY.quad_141630619648[0]+workControllerY.quad_835263414272[0]+workControllerY.quad_962643562496[0]+workControllerY.quad_928463360000[0]+workControllerY.quad_380101586944[0]+workControllerY.quad_150659375104[0]+workControllerY.quad_392524365824[0]+workControllerY.quad_653656117248[0]+workControllerY.quad_758104227840[0];
  return objv;
}
void fillrhs_aff_controllerY(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = workControllerY.rhs;
  r2 = workControllerY.rhs + 182;
  r3 = workControllerY.rhs + 416;
  r4 = workControllerY.rhs + 650;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT_controllerY(r1, workControllerY.y);
  multbymGT_controllerY(workControllerY.buffer, workControllerY.z);
  for (i = 0; i < 182; i++)
    r1[i] += workControllerY.buffer[i];
  multbyP_controllerY(workControllerY.buffer, workControllerY.x);
  for (i = 0; i < 182; i++)
    r1[i] -= workControllerY.buffer[i] + workControllerY.q[i];
  /* r2 = -z. */
  for (i = 0; i < 234; i++)
    r2[i] = -workControllerY.z[i];
  /* r3 = -Gx - s + h. */
  multbymG_controllerY(r3, workControllerY.x);
  for (i = 0; i < 234; i++)
    r3[i] += -workControllerY.s[i] + workControllerY.h[i];
  /* r4 = -Ax + b. */
  multbymA_controllerY(r4, workControllerY.x);
  for (i = 0; i < 78; i++)
    r4[i] += workControllerY.b[i];
}
void fillrhs_cc_controllerY(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = workControllerY.rhs + 182;
  ds_aff = workControllerY.lhs_aff + 182;
  dz_aff = workControllerY.lhs_aff + 416;
  mu = 0;
  for (i = 0; i < 234; i++)
    mu += workControllerY.s[i]*workControllerY.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 234; i++)
    if (ds_aff[i] < minval*workControllerY.s[i])
      minval = ds_aff[i]/workControllerY.s[i];
  for (i = 0; i < 234; i++)
    if (dz_aff[i] < minval*workControllerY.z[i])
      minval = dz_aff[i]/workControllerY.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 234; i++)
    sigma += (workControllerY.s[i] + alpha*ds_aff[i])*
      (workControllerY.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.004273504273504274;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 182; i++)
    workControllerY.rhs[i] = 0;
  for (i = 416; i < 728; i++)
    workControllerY.rhs[i] = 0;
  for (i = 0; i < 234; i++)
    r2[i] = workControllerY.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine_controllerY(double *target, double *var) {
  int i, j;
  double *residual = workControllerY.buffer;
  double norm2;
  double *new_var = workControllerY.buffer2;
  for (j = 0; j < settingsControllerY.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply_controllerY(residual, var);
    for (i = 0; i < 728; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settingsControllerY.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve_controllerY(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 728; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settingsControllerY.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply_controllerY(residual, var);
    for (i = 0; i < 728; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double calc_ineq_resid_squared_controllerY(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG_controllerY(workControllerY.buffer, workControllerY.x);
  /* Add -s + h. */
  for (i = 0; i < 234; i++)
    workControllerY.buffer[i] += -workControllerY.s[i] + workControllerY.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 234; i++)
    norm2_squared += workControllerY.buffer[i]*workControllerY.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared_controllerY(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA_controllerY(workControllerY.buffer, workControllerY.x);
  /* Add +b. */
  for (i = 0; i < 78; i++)
    workControllerY.buffer[i] += workControllerY.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 78; i++)
    norm2_squared += workControllerY.buffer[i]*workControllerY.buffer[i];
  return norm2_squared;
}
void better_start_controllerY(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  workControllerY.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 234; i++)
    workControllerY.s_inv_z[i] = 1;
  fill_KKT_controllerY();
  ldl_factor_controllerY();
  fillrhs_start_controllerY();
  /* Borrow workControllerY.lhs_aff for the solution. */
  ldl_solve_controllerY(workControllerY.rhs, workControllerY.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = workControllerY.lhs_aff;
  s = workControllerY.lhs_aff + 182;
  z = workControllerY.lhs_aff + 416;
  y = workControllerY.lhs_aff + 650;
  /* Just set x and y as is. */
  for (i = 0; i < 182; i++)
    workControllerY.x[i] = x[i];
  for (i = 0; i < 78; i++)
    workControllerY.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 234; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 234; i++)
      workControllerY.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 234; i++)
      workControllerY.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 234; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 234; i++)
      workControllerY.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 234; i++)
      workControllerY.z[i] = z[i] + alpha;
  }
}
void fillrhs_start_controllerY(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = workControllerY.rhs;
  r2 = workControllerY.rhs + 182;
  r3 = workControllerY.rhs + 416;
  r4 = workControllerY.rhs + 650;
  for (i = 0; i < 182; i++)
    r1[i] = -workControllerY.q[i];
  for (i = 0; i < 234; i++)
    r2[i] = 0;
  for (i = 0; i < 234; i++)
    r3[i] = workControllerY.h[i];
  for (i = 0; i < 78; i++)
    r4[i] = workControllerY.b[i];
}
long solve_controllerY(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  workControllerY.converged = 0;
  setup_pointers_controllerY();
  pre_ops_controllerY();
#ifndef ZERO_LIBRARY_MODE
  if (settingsControllerY.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq_controllerY();
  fillh_controllerY();
  fillb_controllerY();
  if (settingsControllerY.better_start)
    better_start_controllerY();
  else
    set_start_controllerY();
  for (iter = 0; iter < settingsControllerY.max_iters; iter++) {
    for (i = 0; i < 234; i++) {
      workControllerY.s_inv[i] = 1.0 / workControllerY.s[i];
      workControllerY.s_inv_z[i] = workControllerY.s_inv[i]*workControllerY.z[i];
    }
    workControllerY.block_33[0] = 0;
    fill_KKT_controllerY();
    ldl_factor_controllerY();
    /* Affine scaling directions. */
    fillrhs_aff_controllerY();
    ldl_solve_controllerY(workControllerY.rhs, workControllerY.lhs_aff);
    refine_controllerY(workControllerY.rhs, workControllerY.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc_controllerY();
    ldl_solve_controllerY(workControllerY.rhs, workControllerY.lhs_cc);
    refine_controllerY(workControllerY.rhs, workControllerY.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 728; i++)
      workControllerY.lhs_aff[i] += workControllerY.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = workControllerY.lhs_aff;
    ds = workControllerY.lhs_aff + 182;
    dz = workControllerY.lhs_aff + 416;
    dy = workControllerY.lhs_aff + 650;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 234; i++)
      if (ds[i] < minval*workControllerY.s[i])
        minval = ds[i]/workControllerY.s[i];
    for (i = 0; i < 234; i++)
      if (dz[i] < minval*workControllerY.z[i])
        minval = dz[i]/workControllerY.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 182; i++)
      workControllerY.x[i] += alpha*dx[i];
    for (i = 0; i < 234; i++)
      workControllerY.s[i] += alpha*ds[i];
    for (i = 0; i < 234; i++)
      workControllerY.z[i] += alpha*dz[i];
    for (i = 0; i < 78; i++)
      workControllerY.y[i] += alpha*dy[i];
    workControllerY.gap = eval_gap_controllerY();
    workControllerY.eq_resid_squared = calc_eq_resid_squared_controllerY();
    workControllerY.ineq_resid_squared = calc_ineq_resid_squared_controllerY();
#ifndef ZERO_LIBRARY_MODE
    if (settingsControllerY.verbose) {
      workControllerY.optval = eval_objv_controllerY();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, workControllerY.optval, workControllerY.gap, sqrt(workControllerY.eq_resid_squared),
          sqrt(workControllerY.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (workControllerY.gap < settingsControllerY.eps)
        && (workControllerY.eq_resid_squared <= settingsControllerY.resid_tol*settingsControllerY.resid_tol)
        && (workControllerY.ineq_resid_squared <= settingsControllerY.resid_tol*settingsControllerY.resid_tol)
       ) {
      workControllerY.converged = 1;
      workControllerY.optval = eval_objv_controllerY();
      return iter+1;
    }
  }
  return iter;
}

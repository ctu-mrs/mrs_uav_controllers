/* Produced by CVXGEN, 2019-06-05 08:31:52 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap_controllerX(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 234; i++)
    gap += workControllerX.z[i]*workControllerX.s[i];
  return gap;
}
void set_defaults_controllerX(void) {
  settingsControllerX.resid_tol = 1e-6;
  settingsControllerX.eps = 1e-4;
  settingsControllerX.max_iters = 25;
  settingsControllerX.refine_steps = 1;
  settingsControllerX.s_init = 1;
  settingsControllerX.z_init = 1;
  settingsControllerX.debug = 0;
  settingsControllerX.verbose = 1;
  settingsControllerX.verbose_refinement = 0;
  settingsControllerX.better_start = 1;
  settingsControllerX.kkt_reg = 1e-7;
}
void setup_pointers_controllerX(void) {
  workControllerX.y = workControllerX.x + 182;
  workControllerX.s = workControllerX.x + 260;
  workControllerX.z = workControllerX.x + 494;
  varsControllerX.u_0 = workControllerX.x + 78;
  varsControllerX.u_1 = workControllerX.x + 79;
  varsControllerX.u_2 = workControllerX.x + 80;
  varsControllerX.u_3 = workControllerX.x + 81;
  varsControllerX.u_4 = workControllerX.x + 82;
  varsControllerX.u_5 = workControllerX.x + 83;
  varsControllerX.u_6 = workControllerX.x + 84;
  varsControllerX.u_7 = workControllerX.x + 85;
  varsControllerX.u_8 = workControllerX.x + 86;
  varsControllerX.u_9 = workControllerX.x + 87;
  varsControllerX.u_10 = workControllerX.x + 88;
  varsControllerX.u_11 = workControllerX.x + 89;
  varsControllerX.u_12 = workControllerX.x + 90;
  varsControllerX.u_13 = workControllerX.x + 91;
  varsControllerX.u_14 = workControllerX.x + 92;
  varsControllerX.u_15 = workControllerX.x + 93;
  varsControllerX.u_16 = workControllerX.x + 94;
  varsControllerX.u_17 = workControllerX.x + 95;
  varsControllerX.u_18 = workControllerX.x + 96;
  varsControllerX.u_19 = workControllerX.x + 97;
  varsControllerX.u_20 = workControllerX.x + 98;
  varsControllerX.u_21 = workControllerX.x + 99;
  varsControllerX.u_22 = workControllerX.x + 100;
  varsControllerX.u_23 = workControllerX.x + 101;
  varsControllerX.u_24 = workControllerX.x + 102;
  varsControllerX.u_25 = workControllerX.x + 103;
  varsControllerX.x_1 = workControllerX.x + 104;
  varsControllerX.x_2 = workControllerX.x + 107;
  varsControllerX.x_3 = workControllerX.x + 110;
  varsControllerX.x_4 = workControllerX.x + 113;
  varsControllerX.x_5 = workControllerX.x + 116;
  varsControllerX.x_6 = workControllerX.x + 119;
  varsControllerX.x_7 = workControllerX.x + 122;
  varsControllerX.x_8 = workControllerX.x + 125;
  varsControllerX.x_9 = workControllerX.x + 128;
  varsControllerX.x_10 = workControllerX.x + 131;
  varsControllerX.x_11 = workControllerX.x + 134;
  varsControllerX.x_12 = workControllerX.x + 137;
  varsControllerX.x_13 = workControllerX.x + 140;
  varsControllerX.x_14 = workControllerX.x + 143;
  varsControllerX.x_15 = workControllerX.x + 146;
  varsControllerX.x_16 = workControllerX.x + 149;
  varsControllerX.x_17 = workControllerX.x + 152;
  varsControllerX.x_18 = workControllerX.x + 155;
  varsControllerX.x_19 = workControllerX.x + 158;
  varsControllerX.x_20 = workControllerX.x + 161;
  varsControllerX.x_21 = workControllerX.x + 164;
  varsControllerX.x_22 = workControllerX.x + 167;
  varsControllerX.x_23 = workControllerX.x + 170;
  varsControllerX.x_24 = workControllerX.x + 173;
  varsControllerX.x_25 = workControllerX.x + 176;
  varsControllerX.x_26 = workControllerX.x + 179;
}
void setup_indexed_paramsControllerX_controllerX(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  paramsControllerX.x_ss[1] = paramsControllerX.x_ss_1;
  paramsControllerX.x_ss[2] = paramsControllerX.x_ss_2;
  paramsControllerX.x_ss[3] = paramsControllerX.x_ss_3;
  paramsControllerX.x_ss[4] = paramsControllerX.x_ss_4;
  paramsControllerX.x_ss[5] = paramsControllerX.x_ss_5;
  paramsControllerX.x_ss[6] = paramsControllerX.x_ss_6;
  paramsControllerX.x_ss[7] = paramsControllerX.x_ss_7;
  paramsControllerX.x_ss[8] = paramsControllerX.x_ss_8;
  paramsControllerX.x_ss[9] = paramsControllerX.x_ss_9;
  paramsControllerX.x_ss[10] = paramsControllerX.x_ss_10;
  paramsControllerX.x_ss[11] = paramsControllerX.x_ss_11;
  paramsControllerX.x_ss[12] = paramsControllerX.x_ss_12;
  paramsControllerX.x_ss[13] = paramsControllerX.x_ss_13;
  paramsControllerX.x_ss[14] = paramsControllerX.x_ss_14;
  paramsControllerX.x_ss[15] = paramsControllerX.x_ss_15;
  paramsControllerX.x_ss[16] = paramsControllerX.x_ss_16;
  paramsControllerX.x_ss[17] = paramsControllerX.x_ss_17;
  paramsControllerX.x_ss[18] = paramsControllerX.x_ss_18;
  paramsControllerX.x_ss[19] = paramsControllerX.x_ss_19;
  paramsControllerX.x_ss[20] = paramsControllerX.x_ss_20;
  paramsControllerX.x_ss[21] = paramsControllerX.x_ss_21;
  paramsControllerX.x_ss[22] = paramsControllerX.x_ss_22;
  paramsControllerX.x_ss[23] = paramsControllerX.x_ss_23;
  paramsControllerX.x_ss[24] = paramsControllerX.x_ss_24;
  paramsControllerX.x_ss[25] = paramsControllerX.x_ss_25;
  paramsControllerX.x_ss[26] = paramsControllerX.x_ss_26;
  paramsControllerX.x[0] = paramsControllerX.x_0;
}
void setup_indexed_optvarsControllerX_controllerX(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  varsControllerX.x[1] = varsControllerX.x_1;
  varsControllerX.x[2] = varsControllerX.x_2;
  varsControllerX.x[3] = varsControllerX.x_3;
  varsControllerX.x[4] = varsControllerX.x_4;
  varsControllerX.x[5] = varsControllerX.x_5;
  varsControllerX.x[6] = varsControllerX.x_6;
  varsControllerX.x[7] = varsControllerX.x_7;
  varsControllerX.x[8] = varsControllerX.x_8;
  varsControllerX.x[9] = varsControllerX.x_9;
  varsControllerX.x[10] = varsControllerX.x_10;
  varsControllerX.x[11] = varsControllerX.x_11;
  varsControllerX.x[12] = varsControllerX.x_12;
  varsControllerX.x[13] = varsControllerX.x_13;
  varsControllerX.x[14] = varsControllerX.x_14;
  varsControllerX.x[15] = varsControllerX.x_15;
  varsControllerX.x[16] = varsControllerX.x_16;
  varsControllerX.x[17] = varsControllerX.x_17;
  varsControllerX.x[18] = varsControllerX.x_18;
  varsControllerX.x[19] = varsControllerX.x_19;
  varsControllerX.x[20] = varsControllerX.x_20;
  varsControllerX.x[21] = varsControllerX.x_21;
  varsControllerX.x[22] = varsControllerX.x_22;
  varsControllerX.x[23] = varsControllerX.x_23;
  varsControllerX.x[24] = varsControllerX.x_24;
  varsControllerX.x[25] = varsControllerX.x_25;
  varsControllerX.x[26] = varsControllerX.x_26;
  varsControllerX.u[0] = varsControllerX.u_0;
  varsControllerX.u[1] = varsControllerX.u_1;
  varsControllerX.u[2] = varsControllerX.u_2;
  varsControllerX.u[3] = varsControllerX.u_3;
  varsControllerX.u[4] = varsControllerX.u_4;
  varsControllerX.u[5] = varsControllerX.u_5;
  varsControllerX.u[6] = varsControllerX.u_6;
  varsControllerX.u[7] = varsControllerX.u_7;
  varsControllerX.u[8] = varsControllerX.u_8;
  varsControllerX.u[9] = varsControllerX.u_9;
  varsControllerX.u[10] = varsControllerX.u_10;
  varsControllerX.u[11] = varsControllerX.u_11;
  varsControllerX.u[12] = varsControllerX.u_12;
  varsControllerX.u[13] = varsControllerX.u_13;
  varsControllerX.u[14] = varsControllerX.u_14;
  varsControllerX.u[15] = varsControllerX.u_15;
  varsControllerX.u[16] = varsControllerX.u_16;
  varsControllerX.u[17] = varsControllerX.u_17;
  varsControllerX.u[18] = varsControllerX.u_18;
  varsControllerX.u[19] = varsControllerX.u_19;
  varsControllerX.u[20] = varsControllerX.u_20;
  varsControllerX.u[21] = varsControllerX.u_21;
  varsControllerX.u[22] = varsControllerX.u_22;
  varsControllerX.u[23] = varsControllerX.u_23;
  varsControllerX.u[24] = varsControllerX.u_24;
  varsControllerX.u[25] = varsControllerX.u_25;
}
void setup_indexing_controllerX(void) {
  setup_pointers_controllerX();
  setup_indexed_paramsControllerX_controllerX();
  setup_indexed_optvarsControllerX_controllerX();
}
void set_start_controllerX(void) {
  int i;
  for (i = 0; i < 182; i++)
    workControllerX.x[i] = 0;
  for (i = 0; i < 78; i++)
    workControllerX.y[i] = 0;
  for (i = 0; i < 234; i++)
    workControllerX.s[i] = (workControllerX.h[i] > 0) ? workControllerX.h[i] : settingsControllerX.s_init;
  for (i = 0; i < 234; i++)
    workControllerX.z[i] = settingsControllerX.z_init;
}
double eval_objv_controllerX(void) {
  int i;
  double objv;
  /* Borrow space in workControllerX.rhs. */
  multbyP_controllerX(workControllerX.rhs, workControllerX.x);
  objv = 0;
  for (i = 0; i < 182; i++)
    objv += workControllerX.x[i]*workControllerX.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 182; i++)
    objv += workControllerX.q[i]*workControllerX.x[i];
  objv += workControllerX.quad_600568381440[0]+workControllerX.quad_898851794944[0]+workControllerX.quad_88433618944[0]+workControllerX.quad_240204779520[0]+workControllerX.quad_635618762752[0]+workControllerX.quad_732753989632[0]+workControllerX.quad_427523055616[0]+workControllerX.quad_976046530560[0]+workControllerX.quad_688550678528[0]+workControllerX.quad_304816418816[0]+workControllerX.quad_819339411456[0]+workControllerX.quad_101800079360[0]+workControllerX.quad_976903761920[0]+workControllerX.quad_141299838976[0]+workControllerX.quad_343404097536[0]+workControllerX.quad_815806124032[0]+workControllerX.quad_997002137600[0]+workControllerX.quad_141630619648[0]+workControllerX.quad_835263414272[0]+workControllerX.quad_962643562496[0]+workControllerX.quad_928463360000[0]+workControllerX.quad_380101586944[0]+workControllerX.quad_150659375104[0]+workControllerX.quad_392524365824[0]+workControllerX.quad_653656117248[0]+workControllerX.quad_758104227840[0];
  return objv;
}
void fillrhs_aff_controllerX(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = workControllerX.rhs;
  r2 = workControllerX.rhs + 182;
  r3 = workControllerX.rhs + 416;
  r4 = workControllerX.rhs + 650;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT_controllerX(r1, workControllerX.y);
  multbymGT_controllerX(workControllerX.buffer, workControllerX.z);
  for (i = 0; i < 182; i++)
    r1[i] += workControllerX.buffer[i];
  multbyP_controllerX(workControllerX.buffer, workControllerX.x);
  for (i = 0; i < 182; i++)
    r1[i] -= workControllerX.buffer[i] + workControllerX.q[i];
  /* r2 = -z. */
  for (i = 0; i < 234; i++)
    r2[i] = -workControllerX.z[i];
  /* r3 = -Gx - s + h. */
  multbymG_controllerX(r3, workControllerX.x);
  for (i = 0; i < 234; i++)
    r3[i] += -workControllerX.s[i] + workControllerX.h[i];
  /* r4 = -Ax + b. */
  multbymA_controllerX(r4, workControllerX.x);
  for (i = 0; i < 78; i++)
    r4[i] += workControllerX.b[i];
}
void fillrhs_cc_controllerX(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = workControllerX.rhs + 182;
  ds_aff = workControllerX.lhs_aff + 182;
  dz_aff = workControllerX.lhs_aff + 416;
  mu = 0;
  for (i = 0; i < 234; i++)
    mu += workControllerX.s[i]*workControllerX.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 234; i++)
    if (ds_aff[i] < minval*workControllerX.s[i])
      minval = ds_aff[i]/workControllerX.s[i];
  for (i = 0; i < 234; i++)
    if (dz_aff[i] < minval*workControllerX.z[i])
      minval = dz_aff[i]/workControllerX.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 234; i++)
    sigma += (workControllerX.s[i] + alpha*ds_aff[i])*
      (workControllerX.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.004273504273504274;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 182; i++)
    workControllerX.rhs[i] = 0;
  for (i = 416; i < 728; i++)
    workControllerX.rhs[i] = 0;
  for (i = 0; i < 234; i++)
    r2[i] = workControllerX.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine_controllerX(double *target, double *var) {
  int i, j;
  double *residual = workControllerX.buffer;
  double norm2;
  double *new_var = workControllerX.buffer2;
  for (j = 0; j < settingsControllerX.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply_controllerX(residual, var);
    for (i = 0; i < 728; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settingsControllerX.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve_controllerX(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 728; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settingsControllerX.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply_controllerX(residual, var);
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
double calc_ineq_resid_squared_controllerX(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG_controllerX(workControllerX.buffer, workControllerX.x);
  /* Add -s + h. */
  for (i = 0; i < 234; i++)
    workControllerX.buffer[i] += -workControllerX.s[i] + workControllerX.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 234; i++)
    norm2_squared += workControllerX.buffer[i]*workControllerX.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared_controllerX(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA_controllerX(workControllerX.buffer, workControllerX.x);
  /* Add +b. */
  for (i = 0; i < 78; i++)
    workControllerX.buffer[i] += workControllerX.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 78; i++)
    norm2_squared += workControllerX.buffer[i]*workControllerX.buffer[i];
  return norm2_squared;
}
void better_start_controllerX(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  workControllerX.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 234; i++)
    workControllerX.s_inv_z[i] = 1;
  fill_KKT_controllerX();
  ldl_factor_controllerX();
  fillrhs_start_controllerX();
  /* Borrow workControllerX.lhs_aff for the solution. */
  ldl_solve_controllerX(workControllerX.rhs, workControllerX.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = workControllerX.lhs_aff;
  s = workControllerX.lhs_aff + 182;
  z = workControllerX.lhs_aff + 416;
  y = workControllerX.lhs_aff + 650;
  /* Just set x and y as is. */
  for (i = 0; i < 182; i++)
    workControllerX.x[i] = x[i];
  for (i = 0; i < 78; i++)
    workControllerX.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 234; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 234; i++)
      workControllerX.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 234; i++)
      workControllerX.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 234; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 234; i++)
      workControllerX.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 234; i++)
      workControllerX.z[i] = z[i] + alpha;
  }
}
void fillrhs_start_controllerX(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = workControllerX.rhs;
  r2 = workControllerX.rhs + 182;
  r3 = workControllerX.rhs + 416;
  r4 = workControllerX.rhs + 650;
  for (i = 0; i < 182; i++)
    r1[i] = -workControllerX.q[i];
  for (i = 0; i < 234; i++)
    r2[i] = 0;
  for (i = 0; i < 234; i++)
    r3[i] = workControllerX.h[i];
  for (i = 0; i < 78; i++)
    r4[i] = workControllerX.b[i];
}
long solve_controllerX(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  workControllerX.converged = 0;
  setup_pointers_controllerX();
  pre_ops_controllerX();
#ifndef ZERO_LIBRARY_MODE
  if (settingsControllerX.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq_controllerX();
  fillh_controllerX();
  fillb_controllerX();
  if (settingsControllerX.better_start)
    better_start_controllerX();
  else
    set_start_controllerX();
  for (iter = 0; iter < settingsControllerX.max_iters; iter++) {
    for (i = 0; i < 234; i++) {
      workControllerX.s_inv[i] = 1.0 / workControllerX.s[i];
      workControllerX.s_inv_z[i] = workControllerX.s_inv[i]*workControllerX.z[i];
    }
    workControllerX.block_33[0] = 0;
    fill_KKT_controllerX();
    ldl_factor_controllerX();
    /* Affine scaling directions. */
    fillrhs_aff_controllerX();
    ldl_solve_controllerX(workControllerX.rhs, workControllerX.lhs_aff);
    refine_controllerX(workControllerX.rhs, workControllerX.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc_controllerX();
    ldl_solve_controllerX(workControllerX.rhs, workControllerX.lhs_cc);
    refine_controllerX(workControllerX.rhs, workControllerX.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 728; i++)
      workControllerX.lhs_aff[i] += workControllerX.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = workControllerX.lhs_aff;
    ds = workControllerX.lhs_aff + 182;
    dz = workControllerX.lhs_aff + 416;
    dy = workControllerX.lhs_aff + 650;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 234; i++)
      if (ds[i] < minval*workControllerX.s[i])
        minval = ds[i]/workControllerX.s[i];
    for (i = 0; i < 234; i++)
      if (dz[i] < minval*workControllerX.z[i])
        minval = dz[i]/workControllerX.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 182; i++)
      workControllerX.x[i] += alpha*dx[i];
    for (i = 0; i < 234; i++)
      workControllerX.s[i] += alpha*ds[i];
    for (i = 0; i < 234; i++)
      workControllerX.z[i] += alpha*dz[i];
    for (i = 0; i < 78; i++)
      workControllerX.y[i] += alpha*dy[i];
    workControllerX.gap = eval_gap_controllerX();
    workControllerX.eq_resid_squared = calc_eq_resid_squared_controllerX();
    workControllerX.ineq_resid_squared = calc_ineq_resid_squared_controllerX();
#ifndef ZERO_LIBRARY_MODE
    if (settingsControllerX.verbose) {
      workControllerX.optval = eval_objv_controllerX();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, workControllerX.optval, workControllerX.gap, sqrt(workControllerX.eq_resid_squared),
          sqrt(workControllerX.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (workControllerX.gap < settingsControllerX.eps)
        && (workControllerX.eq_resid_squared <= settingsControllerX.resid_tol*settingsControllerX.resid_tol)
        && (workControllerX.ineq_resid_squared <= settingsControllerX.resid_tol*settingsControllerX.resid_tol)
       ) {
      workControllerX.converged = 1;
      workControllerX.optval = eval_objv_controllerX();
      return iter+1;
    }
  }
  return iter;
}

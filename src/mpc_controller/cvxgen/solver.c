/* Produced by CVXGEN, 2019-06-04 10:54:51 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap_controller(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 234; i++)
    gap += workController.z[i]*workController.s[i];
  return gap;
}
void set_defaults_controller(void) {
  settingsController.resid_tol = 1e-6;
  settingsController.eps = 1e-4;
  settingsController.max_iters = 25;
  settingsController.refine_steps = 1;
  settingsController.s_init = 1;
  settingsController.z_init = 1;
  settingsController.debug = 0;
  settingsController.verbose = 1;
  settingsController.verbose_refinement = 0;
  settingsController.better_start = 1;
  settingsController.kkt_reg = 1e-7;
}
void setup_pointers_controller(void) {
  workController.y = workController.x + 182;
  workController.s = workController.x + 260;
  workController.z = workController.x + 494;
  varsController.u_0 = workController.x + 78;
  varsController.u_1 = workController.x + 79;
  varsController.u_2 = workController.x + 80;
  varsController.u_3 = workController.x + 81;
  varsController.u_4 = workController.x + 82;
  varsController.u_5 = workController.x + 83;
  varsController.u_6 = workController.x + 84;
  varsController.u_7 = workController.x + 85;
  varsController.u_8 = workController.x + 86;
  varsController.u_9 = workController.x + 87;
  varsController.u_10 = workController.x + 88;
  varsController.u_11 = workController.x + 89;
  varsController.u_12 = workController.x + 90;
  varsController.u_13 = workController.x + 91;
  varsController.u_14 = workController.x + 92;
  varsController.u_15 = workController.x + 93;
  varsController.u_16 = workController.x + 94;
  varsController.u_17 = workController.x + 95;
  varsController.u_18 = workController.x + 96;
  varsController.u_19 = workController.x + 97;
  varsController.u_20 = workController.x + 98;
  varsController.u_21 = workController.x + 99;
  varsController.u_22 = workController.x + 100;
  varsController.u_23 = workController.x + 101;
  varsController.u_24 = workController.x + 102;
  varsController.u_25 = workController.x + 103;
  varsController.x_1 = workController.x + 104;
  varsController.x_2 = workController.x + 107;
  varsController.x_3 = workController.x + 110;
  varsController.x_4 = workController.x + 113;
  varsController.x_5 = workController.x + 116;
  varsController.x_6 = workController.x + 119;
  varsController.x_7 = workController.x + 122;
  varsController.x_8 = workController.x + 125;
  varsController.x_9 = workController.x + 128;
  varsController.x_10 = workController.x + 131;
  varsController.x_11 = workController.x + 134;
  varsController.x_12 = workController.x + 137;
  varsController.x_13 = workController.x + 140;
  varsController.x_14 = workController.x + 143;
  varsController.x_15 = workController.x + 146;
  varsController.x_16 = workController.x + 149;
  varsController.x_17 = workController.x + 152;
  varsController.x_18 = workController.x + 155;
  varsController.x_19 = workController.x + 158;
  varsController.x_20 = workController.x + 161;
  varsController.x_21 = workController.x + 164;
  varsController.x_22 = workController.x + 167;
  varsController.x_23 = workController.x + 170;
  varsController.x_24 = workController.x + 173;
  varsController.x_25 = workController.x + 176;
  varsController.x_26 = workController.x + 179;
}
void setup_indexed_paramsController_controller(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  paramsController.x_ss[1] = paramsController.x_ss_1;
  paramsController.x_ss[2] = paramsController.x_ss_2;
  paramsController.x_ss[3] = paramsController.x_ss_3;
  paramsController.x_ss[4] = paramsController.x_ss_4;
  paramsController.x_ss[5] = paramsController.x_ss_5;
  paramsController.x_ss[6] = paramsController.x_ss_6;
  paramsController.x_ss[7] = paramsController.x_ss_7;
  paramsController.x_ss[8] = paramsController.x_ss_8;
  paramsController.x_ss[9] = paramsController.x_ss_9;
  paramsController.x_ss[10] = paramsController.x_ss_10;
  paramsController.x_ss[11] = paramsController.x_ss_11;
  paramsController.x_ss[12] = paramsController.x_ss_12;
  paramsController.x_ss[13] = paramsController.x_ss_13;
  paramsController.x_ss[14] = paramsController.x_ss_14;
  paramsController.x_ss[15] = paramsController.x_ss_15;
  paramsController.x_ss[16] = paramsController.x_ss_16;
  paramsController.x_ss[17] = paramsController.x_ss_17;
  paramsController.x_ss[18] = paramsController.x_ss_18;
  paramsController.x_ss[19] = paramsController.x_ss_19;
  paramsController.x_ss[20] = paramsController.x_ss_20;
  paramsController.x_ss[21] = paramsController.x_ss_21;
  paramsController.x_ss[22] = paramsController.x_ss_22;
  paramsController.x_ss[23] = paramsController.x_ss_23;
  paramsController.x_ss[24] = paramsController.x_ss_24;
  paramsController.x_ss[25] = paramsController.x_ss_25;
  paramsController.x_ss[26] = paramsController.x_ss_26;
  paramsController.x[0] = paramsController.x_0;
}
void setup_indexed_optvarsController_controller(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  varsController.x[1] = varsController.x_1;
  varsController.x[2] = varsController.x_2;
  varsController.x[3] = varsController.x_3;
  varsController.x[4] = varsController.x_4;
  varsController.x[5] = varsController.x_5;
  varsController.x[6] = varsController.x_6;
  varsController.x[7] = varsController.x_7;
  varsController.x[8] = varsController.x_8;
  varsController.x[9] = varsController.x_9;
  varsController.x[10] = varsController.x_10;
  varsController.x[11] = varsController.x_11;
  varsController.x[12] = varsController.x_12;
  varsController.x[13] = varsController.x_13;
  varsController.x[14] = varsController.x_14;
  varsController.x[15] = varsController.x_15;
  varsController.x[16] = varsController.x_16;
  varsController.x[17] = varsController.x_17;
  varsController.x[18] = varsController.x_18;
  varsController.x[19] = varsController.x_19;
  varsController.x[20] = varsController.x_20;
  varsController.x[21] = varsController.x_21;
  varsController.x[22] = varsController.x_22;
  varsController.x[23] = varsController.x_23;
  varsController.x[24] = varsController.x_24;
  varsController.x[25] = varsController.x_25;
  varsController.x[26] = varsController.x_26;
  varsController.u[0] = varsController.u_0;
  varsController.u[1] = varsController.u_1;
  varsController.u[2] = varsController.u_2;
  varsController.u[3] = varsController.u_3;
  varsController.u[4] = varsController.u_4;
  varsController.u[5] = varsController.u_5;
  varsController.u[6] = varsController.u_6;
  varsController.u[7] = varsController.u_7;
  varsController.u[8] = varsController.u_8;
  varsController.u[9] = varsController.u_9;
  varsController.u[10] = varsController.u_10;
  varsController.u[11] = varsController.u_11;
  varsController.u[12] = varsController.u_12;
  varsController.u[13] = varsController.u_13;
  varsController.u[14] = varsController.u_14;
  varsController.u[15] = varsController.u_15;
  varsController.u[16] = varsController.u_16;
  varsController.u[17] = varsController.u_17;
  varsController.u[18] = varsController.u_18;
  varsController.u[19] = varsController.u_19;
  varsController.u[20] = varsController.u_20;
  varsController.u[21] = varsController.u_21;
  varsController.u[22] = varsController.u_22;
  varsController.u[23] = varsController.u_23;
  varsController.u[24] = varsController.u_24;
  varsController.u[25] = varsController.u_25;
}
void setup_indexing_controller(void) {
  setup_pointers_controller();
  setup_indexed_paramsController_controller();
  setup_indexed_optvarsController_controller();
}
void set_start_controller(void) {
  int i;
  for (i = 0; i < 182; i++)
    workController.x[i] = 0;
  for (i = 0; i < 78; i++)
    workController.y[i] = 0;
  for (i = 0; i < 234; i++)
    workController.s[i] = (workController.h[i] > 0) ? workController.h[i] : settingsController.s_init;
  for (i = 0; i < 234; i++)
    workController.z[i] = settingsController.z_init;
}
double eval_objv_controller(void) {
  int i;
  double objv;
  /* Borrow space in workController.rhs. */
  multbyP_controller(workController.rhs, workController.x);
  objv = 0;
  for (i = 0; i < 182; i++)
    objv += workController.x[i]*workController.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 182; i++)
    objv += workController.q[i]*workController.x[i];
  objv += workController.quad_600568381440[0]+workController.quad_898851794944[0]+workController.quad_88433618944[0]+workController.quad_240204779520[0]+workController.quad_635618762752[0]+workController.quad_732753989632[0]+workController.quad_427523055616[0]+workController.quad_976046530560[0]+workController.quad_688550678528[0]+workController.quad_304816418816[0]+workController.quad_819339411456[0]+workController.quad_101800079360[0]+workController.quad_976903761920[0]+workController.quad_141299838976[0]+workController.quad_343404097536[0]+workController.quad_815806124032[0]+workController.quad_997002137600[0]+workController.quad_141630619648[0]+workController.quad_835263414272[0]+workController.quad_962643562496[0]+workController.quad_928463360000[0]+workController.quad_380101586944[0]+workController.quad_150659375104[0]+workController.quad_392524365824[0]+workController.quad_653656117248[0]+workController.quad_758104227840[0];
  return objv;
}
void fillrhs_aff_controller(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = workController.rhs;
  r2 = workController.rhs + 182;
  r3 = workController.rhs + 416;
  r4 = workController.rhs + 650;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT_controller(r1, workController.y);
  multbymGT_controller(workController.buffer, workController.z);
  for (i = 0; i < 182; i++)
    r1[i] += workController.buffer[i];
  multbyP_controller(workController.buffer, workController.x);
  for (i = 0; i < 182; i++)
    r1[i] -= workController.buffer[i] + workController.q[i];
  /* r2 = -z. */
  for (i = 0; i < 234; i++)
    r2[i] = -workController.z[i];
  /* r3 = -Gx - s + h. */
  multbymG_controller(r3, workController.x);
  for (i = 0; i < 234; i++)
    r3[i] += -workController.s[i] + workController.h[i];
  /* r4 = -Ax + b. */
  multbymA_controller(r4, workController.x);
  for (i = 0; i < 78; i++)
    r4[i] += workController.b[i];
}
void fillrhs_cc_controller(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = workController.rhs + 182;
  ds_aff = workController.lhs_aff + 182;
  dz_aff = workController.lhs_aff + 416;
  mu = 0;
  for (i = 0; i < 234; i++)
    mu += workController.s[i]*workController.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 234; i++)
    if (ds_aff[i] < minval*workController.s[i])
      minval = ds_aff[i]/workController.s[i];
  for (i = 0; i < 234; i++)
    if (dz_aff[i] < minval*workController.z[i])
      minval = dz_aff[i]/workController.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 234; i++)
    sigma += (workController.s[i] + alpha*ds_aff[i])*
      (workController.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.004273504273504274;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 182; i++)
    workController.rhs[i] = 0;
  for (i = 416; i < 728; i++)
    workController.rhs[i] = 0;
  for (i = 0; i < 234; i++)
    r2[i] = workController.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine_controller(double *target, double *var) {
  int i, j;
  double *residual = workController.buffer;
  double norm2;
  double *new_var = workController.buffer2;
  for (j = 0; j < settingsController.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply_controller(residual, var);
    for (i = 0; i < 728; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settingsController.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve_controller(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 728; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settingsController.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply_controller(residual, var);
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
double calc_ineq_resid_squared_controller(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG_controller(workController.buffer, workController.x);
  /* Add -s + h. */
  for (i = 0; i < 234; i++)
    workController.buffer[i] += -workController.s[i] + workController.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 234; i++)
    norm2_squared += workController.buffer[i]*workController.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared_controller(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA_controller(workController.buffer, workController.x);
  /* Add +b. */
  for (i = 0; i < 78; i++)
    workController.buffer[i] += workController.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 78; i++)
    norm2_squared += workController.buffer[i]*workController.buffer[i];
  return norm2_squared;
}
void better_start_controller(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  workController.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 234; i++)
    workController.s_inv_z[i] = 1;
  fill_KKT_controller();
  ldl_factor_controller();
  fillrhs_start_controller();
  /* Borrow workController.lhs_aff for the solution. */
  ldl_solve_controller(workController.rhs, workController.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = workController.lhs_aff;
  s = workController.lhs_aff + 182;
  z = workController.lhs_aff + 416;
  y = workController.lhs_aff + 650;
  /* Just set x and y as is. */
  for (i = 0; i < 182; i++)
    workController.x[i] = x[i];
  for (i = 0; i < 78; i++)
    workController.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 234; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 234; i++)
      workController.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 234; i++)
      workController.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 234; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 234; i++)
      workController.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 234; i++)
      workController.z[i] = z[i] + alpha;
  }
}
void fillrhs_start_controller(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = workController.rhs;
  r2 = workController.rhs + 182;
  r3 = workController.rhs + 416;
  r4 = workController.rhs + 650;
  for (i = 0; i < 182; i++)
    r1[i] = -workController.q[i];
  for (i = 0; i < 234; i++)
    r2[i] = 0;
  for (i = 0; i < 234; i++)
    r3[i] = workController.h[i];
  for (i = 0; i < 78; i++)
    r4[i] = workController.b[i];
}
long solve_controller(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  workController.converged = 0;
  setup_pointers_controller();
  pre_ops_controller();
#ifndef ZERO_LIBRARY_MODE
  if (settingsController.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq_controller();
  fillh_controller();
  fillb_controller();
  if (settingsController.better_start)
    better_start_controller();
  else
    set_start_controller();
  for (iter = 0; iter < settingsController.max_iters; iter++) {
    for (i = 0; i < 234; i++) {
      workController.s_inv[i] = 1.0 / workController.s[i];
      workController.s_inv_z[i] = workController.s_inv[i]*workController.z[i];
    }
    workController.block_33[0] = 0;
    fill_KKT_controller();
    ldl_factor_controller();
    /* Affine scaling directions. */
    fillrhs_aff_controller();
    ldl_solve_controller(workController.rhs, workController.lhs_aff);
    refine_controller(workController.rhs, workController.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc_controller();
    ldl_solve_controller(workController.rhs, workController.lhs_cc);
    refine_controller(workController.rhs, workController.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 728; i++)
      workController.lhs_aff[i] += workController.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = workController.lhs_aff;
    ds = workController.lhs_aff + 182;
    dz = workController.lhs_aff + 416;
    dy = workController.lhs_aff + 650;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 234; i++)
      if (ds[i] < minval*workController.s[i])
        minval = ds[i]/workController.s[i];
    for (i = 0; i < 234; i++)
      if (dz[i] < minval*workController.z[i])
        minval = dz[i]/workController.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 182; i++)
      workController.x[i] += alpha*dx[i];
    for (i = 0; i < 234; i++)
      workController.s[i] += alpha*ds[i];
    for (i = 0; i < 234; i++)
      workController.z[i] += alpha*dz[i];
    for (i = 0; i < 78; i++)
      workController.y[i] += alpha*dy[i];
    workController.gap = eval_gap_controller();
    workController.eq_resid_squared = calc_eq_resid_squared_controller();
    workController.ineq_resid_squared = calc_ineq_resid_squared_controller();
#ifndef ZERO_LIBRARY_MODE
    if (settingsController.verbose) {
      workController.optval = eval_objv_controller();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, workController.optval, workController.gap, sqrt(workController.eq_resid_squared),
          sqrt(workController.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (workController.gap < settingsController.eps)
        && (workController.eq_resid_squared <= settingsController.resid_tol*settingsController.resid_tol)
        && (workController.ineq_resid_squared <= settingsController.resid_tol*settingsController.resid_tol)
       ) {
      workController.converged = 1;
      workController.optval = eval_objv_controller();
      return iter+1;
    }
  }
  return iter;
}

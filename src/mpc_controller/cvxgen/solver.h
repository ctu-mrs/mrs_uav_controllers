/* Produced by CVXGEN, 2019-06-04 10:54:53 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables varsController, paramsController, workController and settingsController. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrixd_controller(#A, A, m, n, 1)
#endif
typedef struct ParamsController_t {
  double x_ss_1[3];
  double Q[3];
  double x_ss_2[3];
  double x_ss_3[3];
  double x_ss_4[3];
  double x_ss_5[3];
  double x_ss_6[3];
  double x_ss_7[3];
  double x_ss_8[3];
  double x_ss_9[3];
  double x_ss_10[3];
  double x_ss_11[3];
  double x_ss_12[3];
  double x_ss_13[3];
  double x_ss_14[3];
  double x_ss_15[3];
  double x_ss_16[3];
  double x_ss_17[3];
  double x_ss_18[3];
  double x_ss_19[3];
  double x_ss_20[3];
  double x_ss_21[3];
  double x_ss_22[3];
  double x_ss_23[3];
  double x_ss_24[3];
  double x_ss_25[3];
  double x_ss_26[3];
  double Q_last[3];
  double Af[4];
  double x_0[3];
  double Bf[1];
  double A[4];
  double B[1];
  double u_max[1];
  double u_last[1];
  double du_max[1];
  double x_max_1[1];
  double *x_ss[27];
  double *x[1];
} ParamsController;
typedef struct VarsController_t {
  double *x_1; /* 3 rows. */
  double *x_2; /* 3 rows. */
  double *x_3; /* 3 rows. */
  double *x_4; /* 3 rows. */
  double *x_5; /* 3 rows. */
  double *x_6; /* 3 rows. */
  double *x_7; /* 3 rows. */
  double *x_8; /* 3 rows. */
  double *x_9; /* 3 rows. */
  double *x_10; /* 3 rows. */
  double *x_11; /* 3 rows. */
  double *x_12; /* 3 rows. */
  double *x_13; /* 3 rows. */
  double *x_14; /* 3 rows. */
  double *x_15; /* 3 rows. */
  double *x_16; /* 3 rows. */
  double *x_17; /* 3 rows. */
  double *x_18; /* 3 rows. */
  double *x_19; /* 3 rows. */
  double *x_20; /* 3 rows. */
  double *x_21; /* 3 rows. */
  double *x_22; /* 3 rows. */
  double *x_23; /* 3 rows. */
  double *x_24; /* 3 rows. */
  double *x_25; /* 3 rows. */
  double *x_26; /* 3 rows. */
  double *t_01; /* 1 rows. */
  double *u_0; /* 1 rows. */
  double *t_02; /* 1 rows. */
  double *u_1; /* 1 rows. */
  double *t_03; /* 1 rows. */
  double *u_2; /* 1 rows. */
  double *t_04; /* 1 rows. */
  double *u_3; /* 1 rows. */
  double *t_05; /* 1 rows. */
  double *u_4; /* 1 rows. */
  double *t_06; /* 1 rows. */
  double *u_5; /* 1 rows. */
  double *t_07; /* 1 rows. */
  double *u_6; /* 1 rows. */
  double *t_08; /* 1 rows. */
  double *u_7; /* 1 rows. */
  double *t_09; /* 1 rows. */
  double *u_8; /* 1 rows. */
  double *t_10; /* 1 rows. */
  double *u_9; /* 1 rows. */
  double *t_11; /* 1 rows. */
  double *u_10; /* 1 rows. */
  double *t_12; /* 1 rows. */
  double *u_11; /* 1 rows. */
  double *t_13; /* 1 rows. */
  double *u_12; /* 1 rows. */
  double *t_14; /* 1 rows. */
  double *u_13; /* 1 rows. */
  double *t_15; /* 1 rows. */
  double *u_14; /* 1 rows. */
  double *t_16; /* 1 rows. */
  double *u_15; /* 1 rows. */
  double *t_17; /* 1 rows. */
  double *u_16; /* 1 rows. */
  double *t_18; /* 1 rows. */
  double *u_17; /* 1 rows. */
  double *t_19; /* 1 rows. */
  double *u_18; /* 1 rows. */
  double *t_20; /* 1 rows. */
  double *u_19; /* 1 rows. */
  double *t_21; /* 1 rows. */
  double *u_20; /* 1 rows. */
  double *t_22; /* 1 rows. */
  double *u_21; /* 1 rows. */
  double *t_23; /* 1 rows. */
  double *u_22; /* 1 rows. */
  double *t_24; /* 1 rows. */
  double *u_23; /* 1 rows. */
  double *t_25; /* 1 rows. */
  double *u_24; /* 1 rows. */
  double *t_26; /* 1 rows. */
  double *u_25; /* 1 rows. */
  double *t_27; /* 1 rows. */
  double *t_28; /* 1 rows. */
  double *t_29; /* 1 rows. */
  double *t_30; /* 1 rows. */
  double *t_31; /* 1 rows. */
  double *t_32; /* 1 rows. */
  double *t_33; /* 1 rows. */
  double *t_34; /* 1 rows. */
  double *t_35; /* 1 rows. */
  double *t_36; /* 1 rows. */
  double *t_37; /* 1 rows. */
  double *t_38; /* 1 rows. */
  double *t_39; /* 1 rows. */
  double *t_40; /* 1 rows. */
  double *t_41; /* 1 rows. */
  double *t_42; /* 1 rows. */
  double *t_43; /* 1 rows. */
  double *t_44; /* 1 rows. */
  double *t_45; /* 1 rows. */
  double *t_46; /* 1 rows. */
  double *t_47; /* 1 rows. */
  double *t_48; /* 1 rows. */
  double *t_49; /* 1 rows. */
  double *t_50; /* 1 rows. */
  double *t_51; /* 1 rows. */
  double *t_52; /* 1 rows. */
  double *t_53; /* 1 rows. */
  double *t_54; /* 1 rows. */
  double *t_55; /* 1 rows. */
  double *t_56; /* 1 rows. */
  double *t_57; /* 1 rows. */
  double *t_58; /* 1 rows. */
  double *t_59; /* 1 rows. */
  double *t_60; /* 1 rows. */
  double *t_61; /* 1 rows. */
  double *t_62; /* 1 rows. */
  double *t_63; /* 1 rows. */
  double *t_64; /* 1 rows. */
  double *t_65; /* 1 rows. */
  double *t_66; /* 1 rows. */
  double *t_67; /* 1 rows. */
  double *t_68; /* 1 rows. */
  double *t_69; /* 1 rows. */
  double *t_70; /* 1 rows. */
  double *t_71; /* 1 rows. */
  double *t_72; /* 1 rows. */
  double *t_73; /* 1 rows. */
  double *t_74; /* 1 rows. */
  double *t_75; /* 1 rows. */
  double *t_76; /* 1 rows. */
  double *t_77; /* 1 rows. */
  double *t_78; /* 1 rows. */
  double *x[27];
  double *u[26];
} VarsController;
typedef struct WorkspaceController_t {
  double h[234];
  double s_inv[234];
  double s_inv_z[234];
  double b[78];
  double q[182];
  double rhs[728];
  double x[728];
  double *s;
  double *z;
  double *y;
  double lhs_aff[728];
  double lhs_cc[728];
  double buffer[728];
  double buffer2[728];
  double KKT[1424];
  double L[1195];
  double d[728];
  double v[728];
  double d_inv[728];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_600568381440[1];
  double quad_898851794944[1];
  double quad_88433618944[1];
  double quad_240204779520[1];
  double quad_635618762752[1];
  double quad_732753989632[1];
  double quad_427523055616[1];
  double quad_976046530560[1];
  double quad_688550678528[1];
  double quad_304816418816[1];
  double quad_819339411456[1];
  double quad_101800079360[1];
  double quad_976903761920[1];
  double quad_141299838976[1];
  double quad_343404097536[1];
  double quad_815806124032[1];
  double quad_997002137600[1];
  double quad_141630619648[1];
  double quad_835263414272[1];
  double quad_962643562496[1];
  double quad_928463360000[1];
  double quad_380101586944[1];
  double quad_150659375104[1];
  double quad_392524365824[1];
  double quad_653656117248[1];
  double quad_758104227840[1];
  int converged;
} WorkspaceController;
typedef struct SettingsController_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} SettingsController;
extern VarsController varsController;
extern ParamsController paramsController;
extern WorkspaceController workController;
extern SettingsController settingsController;
/* Function definitions in ldl.c: */
void ldl_solve_controller(double *target, double *var);
void ldl_factor_controller(void);
double check_factorization_controller(void);
void matrix_multiply_controller(double *result, double *source);
double check_residual_controller(double *target, double *multiplicand);
void fill_KKT_controller(void);

/* Function definitions in matrix_support.c: */
void multbymA_controller(double *lhs, double *rhs);
void multbymAT_controller(double *lhs, double *rhs);
void multbymG_controller(double *lhs, double *rhs);
void multbymGT_controller(double *lhs, double *rhs);
void multbyP_controller(double *lhs, double *rhs);
void fillq_controller(void);
void fillh_controller(void);
void fillb_controller(void);
void pre_ops_controller(void);

/* Function definitions in solver.c: */
double eval_gap_controller(void);
void set_defaults_controller(void);
void setup_pointers_controller(void);
void setup_indexed_paramsController_controller(void);
void setup_indexed_optvarsController_controller(void);
void setup_indexing_controller(void);
void set_start_controller(void);
double eval_objv_controller(void);
void fillrhs_aff_controller(void);
void fillrhs_cc_controller(void);
void refine_controller(double *target, double *var);
double calc_ineq_resid_squared_controller(void);
double calc_eq_resid_squared_controller(void);
void better_start_controller(void);
void fillrhs_start_controller(void);
long solve_controller(void);

/* Function definitions in testsolver.c: */
int main_controller(int argc, char **argv);
void load_default_data_controller(void);

/* Function definitions in util.c: */
void tic_controller(void);
float toc_controller(void);
float tocq_controller(void);
void printmatrixd_controller(char *name, double *A, int m, int n, int sparse);
double unif_controller(double lower, double upper);
float ran1d_controller(long*idum, int reset);
float randn_internal_controller(long *idum, int reset);
double randn_controller(void);
void reset_rand_controller(void);

#endif

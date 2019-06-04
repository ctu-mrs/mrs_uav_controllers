% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1 - x_ss_1, Q) + quad_form(x_2 - x_ss_2, Q) + quad_form(x_3 - x_ss_3, Q) + quad_form(x_4 - x_ss_4, Q) + quad_form(x_5 - x_ss_5, Q) + quad_form(x_6 - x_ss_6, Q) + quad_form(x_7 - x_ss_7, Q) + quad_form(x_8 - x_ss_8, Q) + quad_form(x_9 - x_ss_9, Q) + quad_form(x_10 - x_ss_10, Q) + quad_form(x_11 - x_ss_11, Q) + quad_form(x_12 - x_ss_12, Q) + quad_form(x_13 - x_ss_13, Q) + quad_form(x_14 - x_ss_14, Q) + quad_form(x_15 - x_ss_15, Q) + quad_form(x_16 - x_ss_16, Q) + quad_form(x_17 - x_ss_17, Q) + quad_form(x_18 - x_ss_18, Q) + quad_form(x_19 - x_ss_19, Q) + quad_form(x_20 - x_ss_20, Q) + quad_form(x_21 - x_ss_21, Q) + quad_form(x_22 - x_ss_22, Q) + quad_form(x_23 - x_ss_23, Q) + quad_form(x_24 - x_ss_24, Q) + quad_form(x_25 - x_ss_25, Q) + quad_form(x_26 - x_ss_26, Q_last))
%   subject to
%     x_1 == Af*x_0 + Bf*u_0
%     x_2 == A*x_1 + B*u_1
%     x_3 == A*x_2 + B*u_2
%     x_4 == A*x_3 + B*u_3
%     x_5 == A*x_4 + B*u_4
%     x_6 == A*x_5 + B*u_5
%     x_7 == A*x_6 + B*u_6
%     x_8 == A*x_7 + B*u_7
%     x_9 == A*x_8 + B*u_8
%     x_10 == A*x_9 + B*u_9
%     x_11 == A*x_10 + B*u_10
%     x_12 == A*x_11 + B*u_11
%     x_13 == A*x_12 + B*u_12
%     x_14 == A*x_13 + B*u_13
%     x_15 == A*x_14 + B*u_14
%     x_16 == A*x_15 + B*u_15
%     x_17 == A*x_16 + B*u_16
%     x_18 == A*x_17 + B*u_17
%     x_19 == A*x_18 + B*u_18
%     x_20 == A*x_19 + B*u_19
%     x_21 == A*x_20 + B*u_20
%     x_22 == A*x_21 + B*u_21
%     x_23 == A*x_22 + B*u_22
%     x_24 == A*x_23 + B*u_23
%     x_25 == A*x_24 + B*u_24
%     x_26 == A*x_25 + B*u_25
%     abs(u_0) <= u_max
%     abs(u_1) <= u_max
%     abs(u_2) <= u_max
%     abs(u_3) <= u_max
%     abs(u_4) <= u_max
%     abs(u_5) <= u_max
%     abs(u_6) <= u_max
%     abs(u_7) <= u_max
%     abs(u_8) <= u_max
%     abs(u_9) <= u_max
%     abs(u_10) <= u_max
%     abs(u_11) <= u_max
%     abs(u_12) <= u_max
%     abs(u_13) <= u_max
%     abs(u_14) <= u_max
%     abs(u_15) <= u_max
%     abs(u_16) <= u_max
%     abs(u_17) <= u_max
%     abs(u_18) <= u_max
%     abs(u_19) <= u_max
%     abs(u_20) <= u_max
%     abs(u_21) <= u_max
%     abs(u_22) <= u_max
%     abs(u_23) <= u_max
%     abs(u_24) <= u_max
%     abs(u_25) <= u_max
%     abs(u_0 - u_last) <= du_max
%     abs(u_1 - u_0) <= du_max
%     abs(u_2 - u_1) <= du_max
%     abs(u_3 - u_2) <= du_max
%     abs(u_4 - u_3) <= du_max
%     abs(u_5 - u_4) <= du_max
%     abs(u_6 - u_5) <= du_max
%     abs(u_7 - u_6) <= du_max
%     abs(u_8 - u_7) <= du_max
%     abs(u_9 - u_8) <= du_max
%     abs(u_10 - u_9) <= du_max
%     abs(u_11 - u_10) <= du_max
%     abs(u_12 - u_11) <= du_max
%     abs(u_13 - u_12) <= du_max
%     abs(u_14 - u_13) <= du_max
%     abs(u_15 - u_14) <= du_max
%     abs(u_16 - u_15) <= du_max
%     abs(u_17 - u_16) <= du_max
%     abs(u_18 - u_17) <= du_max
%     abs(u_19 - u_18) <= du_max
%     abs(u_20 - u_19) <= du_max
%     abs(u_21 - u_20) <= du_max
%     abs(u_22 - u_21) <= du_max
%     abs(u_23 - u_22) <= du_max
%     abs(u_24 - u_23) <= du_max
%     abs(u_25 - u_24) <= du_max
%     abs(x_1(1)) <= x_max_1
%     abs(x_2(1)) <= x_max_1
%     abs(x_3(1)) <= x_max_1
%     abs(x_4(1)) <= x_max_1
%     abs(x_5(1)) <= x_max_1
%     abs(x_6(1)) <= x_max_1
%     abs(x_7(1)) <= x_max_1
%     abs(x_8(1)) <= x_max_1
%     abs(x_9(1)) <= x_max_1
%     abs(x_10(1)) <= x_max_1
%     abs(x_11(1)) <= x_max_1
%     abs(x_12(1)) <= x_max_1
%     abs(x_13(1)) <= x_max_1
%     abs(x_14(1)) <= x_max_1
%     abs(x_15(1)) <= x_max_1
%     abs(x_16(1)) <= x_max_1
%     abs(x_17(1)) <= x_max_1
%     abs(x_18(1)) <= x_max_1
%     abs(x_19(1)) <= x_max_1
%     abs(x_20(1)) <= x_max_1
%     abs(x_21(1)) <= x_max_1
%     abs(x_22(1)) <= x_max_1
%     abs(x_23(1)) <= x_max_1
%     abs(x_24(1)) <= x_max_1
%     abs(x_25(1)) <= x_max_1
%     abs(x_26(1)) <= x_max_1
%
% with variables
%      u_0   1 x 1
%      u_1   1 x 1
%      u_2   1 x 1
%      u_3   1 x 1
%      u_4   1 x 1
%      u_5   1 x 1
%      u_6   1 x 1
%      u_7   1 x 1
%      u_8   1 x 1
%      u_9   1 x 1
%     u_10   1 x 1
%     u_11   1 x 1
%     u_12   1 x 1
%     u_13   1 x 1
%     u_14   1 x 1
%     u_15   1 x 1
%     u_16   1 x 1
%     u_17   1 x 1
%     u_18   1 x 1
%     u_19   1 x 1
%     u_20   1 x 1
%     u_21   1 x 1
%     u_22   1 x 1
%     u_23   1 x 1
%     u_24   1 x 1
%     u_25   1 x 1
%      x_1   3 x 1
%      x_2   3 x 1
%      x_3   3 x 1
%      x_4   3 x 1
%      x_5   3 x 1
%      x_6   3 x 1
%      x_7   3 x 1
%      x_8   3 x 1
%      x_9   3 x 1
%     x_10   3 x 1
%     x_11   3 x 1
%     x_12   3 x 1
%     x_13   3 x 1
%     x_14   3 x 1
%     x_15   3 x 1
%     x_16   3 x 1
%     x_17   3 x 1
%     x_18   3 x 1
%     x_19   3 x 1
%     x_20   3 x 1
%     x_21   3 x 1
%     x_22   3 x 1
%     x_23   3 x 1
%     x_24   3 x 1
%     x_25   3 x 1
%     x_26   3 x 1
%
% and parameters
%        A   3 x 3
%       Af   3 x 3
%        B   3 x 1
%       Bf   3 x 1
%        Q   3 x 3    positive, PSD, diagonal
%   Q_last   3 x 3    positive, PSD, diagonal
%   du_max   1 x 1
%   u_last   1 x 1
%    u_max   1 x 1
%      x_0   3 x 1
%  x_max_1   1 x 1
%   x_ss_1   3 x 1
%   x_ss_2   3 x 1
%   x_ss_3   3 x 1
%   x_ss_4   3 x 1
%   x_ss_5   3 x 1
%   x_ss_6   3 x 1
%   x_ss_7   3 x 1
%   x_ss_8   3 x 1
%   x_ss_9   3 x 1
%  x_ss_10   3 x 1
%  x_ss_11   3 x 1
%  x_ss_12   3 x 1
%  x_ss_13   3 x 1
%  x_ss_14   3 x 1
%  x_ss_15   3 x 1
%  x_ss_16   3 x 1
%  x_ss_17   3 x 1
%  x_ss_18   3 x 1
%  x_ss_19   3 x 1
%  x_ss_20   3 x 1
%  x_ss_21   3 x 1
%  x_ss_22   3 x 1
%  x_ss_23   3 x 1
%  x_ss_24   3 x 1
%  x_ss_25   3 x 1
%  x_ss_26   3 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_ss_26, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2019-06-04 10:54:42 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.

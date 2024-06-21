%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;
Ts=0.005;

%% Controller tuning
% z_dot=y-r
A_ext = [sys.A [0;0]; sys.C 0]
B_ext = [sys.B; 0]
% sys_ext = ss(A_ext, B_ext, sys.C, sys.D)

Qu = 6e-6;
Qx = [10 0 0; 0 0.1 0; 0 0 1];
% [K,S,e] = lqrd(A_ext, B_ext, Qx, Qu, 0, Ts) 
[K,S,e] = lqr(A_ext, B_ext, Qx, Qu, 0)


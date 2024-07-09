%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;
Ts=0.005;

%% System discretization and extension
sys_dis = c2d(sys,Ts)
A_tilde = [sys_dis.A [0;0]; sys_dis.C*sys_dis.A 1];
B_tilde = [sys_dis.B; sys_dis.C*sys_dis.B];
C_tilde = [0 1];

%% Controller tuning
% Qu = 0.1;
% Qx = [0.01 0 0; 0 7 0; 0 0 180];
Qu = 0.01;
Qx = [0.01 0 0; 0 280 0; 0 0 10];
[K,S,e] = dlqr(A_tilde, B_tilde, Qx, Qu, 0)

%% Stability margins
G_speed = minreal(tf(G_pos)*s);
speed_loop = feedback(G_speed, K(1));
P1 = speed_loop/s;
P2 = feedback(P1,K(2));
z = tf('z', Ts);
C = d2c(K(3)/(1-z^-1));
W = feedback(C*P2,1);
bandwidth(W) % Closed loop bandwidth is 30rad/s
margin(C*P2) % phase margin is 70deg

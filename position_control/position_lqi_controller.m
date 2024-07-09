%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;
Ts=0.005;

%% Controller tuning
% z_dot=y-r
A_ext = [sys.A [0;0]; -sys.C 0];
B_ext = [sys.B; 0];

%v1 tuning
%Qu = 0.1;
%Qx = [1 0 0; 0 50 0; 0 0 0.1];
%[K,S,e] = lqr(A_ext, B_ext, Qx, Qu, 0)
%v2 tuning
Qu = 0.001;
Qx = [0.01 0 0; 0 12 0; 0 0 300];
[K,S,e] = lqr(A_ext, B_ext, Qx, Qu, 0)

%% Speed loop
s=tf('s')
G_speed=minreal(tf(sys)*s)

stepinfo(G_speed)

%% Analysis
speed_loop=feedback(G_speed,K(1))
P=speed_loop/s;
C=K(2)-K(3)/s;

W=C*P/(1+C*P);
W_ff=(C*P-K(2)*P)/(1+C*P);
bode(W)
hold on
bode(W_ff)
legend("W","W_{ff}")
figure()
bode(C*P)

% Speed loop step
speed_loop_info=stepinfo(speed_loop)

% Position loop step
closed_loop_info=stepinfo(W)
closed_loop_ff_info=stepinfo(W_ff)



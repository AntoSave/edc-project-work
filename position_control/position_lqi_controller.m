%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;
Ts=0.005;

%% Controller tuning
% z_dot=y-r
A_ext = [sys.A [0;0]; -sys.C 0];
B_ext = [sys.B; 0];

% Controllore originale
% Qu = 0.01;
% Qx = [0.1 0 0; 0 100 0; 0 0 0.01];
% Controllore nuovo NON CANCELLARE
Qu = 25;
Qx = [1 0 0; 0 300 0; 0 0 40];
% Controllore
% Qu = 0.1;
% Qx = [0.01 0 0; 0 7 0; 0 0 180];

% Controllore DEFINITIVO (forse?)
% Qu = 0.1;
% Qx = [0.01 0 0; 0 10 0; 0 0 340];
% Qu = 0.1;
% Qx = [0.01 0 0; 0 12 0; 0 0 200];
% [K,S,e] = lqrd(A_ext, B_ext, Qx, Qu, 0, Ts) 
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



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

%% Frequency analysis
% Speed inner loop
speed_loop=feedback(G_speed,K(1))
bandwidth(speed_loop) %Bandwidth of the speed loop is 66 rad/s
speed_loop_info=stepinfo(speed_loop) % Speed loop step info

% Position loop without ff
P=speed_loop/s;
C=K(2)-K(3)/s;
W=C*P/(1+C*P);
bandwidth(W)
closed_loop_info=stepinfo(W)

% Position loop with ff
W_ff=(C*P-K(2)*P)/(1+C*P);
bandwidth(W_ff)
closed_loop_ff_info=stepinfo(W_ff)

% Plots
figure
hold on
bode(W)
bode(W_ff)
legend("W","W_{ff}")

figure
bode(C*P)

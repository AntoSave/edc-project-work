%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;
Ts=0.005;

%% Controller tuning
% z_dot=y-r
A_ext = [sys.A [0;0]; -sys.C 0];
B_ext = [sys.B; 0];
% sys_ext = ss(A_ext, B_ext, sys.C, sys.D)

% Controllore originale
% Qu = 0.01;
% Qx = [0.1 0 0; 0 100 0; 0 0 0.01];
% Controllore nuovo NON CANCELLARE
Qu = 13;
Qx = [1 0 0; 0 600 0; 0 0 35];
% Controllore con la Bryson's rule
% Qu = 1/(12)^2;
% Qx = [1/(80)^2 0 0; 0 1/(10)^2 0; 0 0 1/(10)^2];
% [K,S,e] = lqrd(A_ext, B_ext, Qx, Qu, 0, Ts) 
[K,S,e] = lqr(A_ext, B_ext, Qx, Qu, 0)

%% Take the tf with only speed output
s=tf('s')
G_speed=minreal(tf(sys)*s)

stepinfo(G_speed)

%% Margin
speed_loop=feedback(G_speed,K(1))
P=speed_loop/s
C=K(2)-K(3)/s

bode(C*P)
figure()
bode(G_speed*K(1))

stepinfo(speed_loop)
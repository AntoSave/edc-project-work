clear; clc;
s = tf('s');
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;

%% Motor Model
% Motor parameters
L = 2.3e-3;
R = 1.7;
K = 1.339;

G = 1/(s*L+R);
G = zpk(G);
bandwidth(G) % Open loop bandwidth is 737 rad/s
G0 = evalfr(G,0); % Static gain
G_T = -1/G.P{1}; % Time constant

%% Controller tuning
omega_3_d = 20; % We want a closed loop bandwidth of 20rad/s
Ki = omega_3_d/G0;
Kp = G_T*Ki;
Ti = Kp/Ki;

C = Kp + Ki/s;

%% Stability margins
W = feedback(C*G,1);
W_omega_3 = bandwidth(W) % Closed loop bandwidth of the system is approx 20 rad/s
[Gm, Pm] = margin(G*C) % Phase margin is 90rad/s
max_delay = (Pm*pi/180)/W_omega_3 % Max delay is 0.078s

%% Closed loop performances
stepinfo(W) % Rise time is 0.1s, settling time is 0.19s, no overshoot

figure
bode(W)
figure
step(W)

%% Choosing the settling time
Ts = 0.005;
f_s = 1/Ts;
omega_s = 2*pi*f_s;
omega_s > 2*W_omega_3 % Nyquist condition
omega_s > 8*W_omega_3 % Tusin discretization
W_omega_3*Ts/2 < Pm*pi/180 % Time delay introduced by discretization doesn't drain the phase margin



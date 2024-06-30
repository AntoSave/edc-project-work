clear; clc;
s = tf('s');
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;

%% Motor Model
% Motor parameters
L = 2.3e-3;
R = 3.4;
K = 1.32;

G = 1/(s*L+R);

%% Controller tuning
% rltool
C = 0.45966*(s+1478)/s;

% Sampling time
bode(feedback(C*G,1)) % Closed loop bandwidth of the system is 200 rad/s
W_omega3 = 200;
omega_s_min = 5*W_omega3;
Ts_max = 2*pi/omega_s_min;

Ts = 0.005;

%% Integral Algorithm
z = tf('z');
Cz=c2d(C,Ts,'tustin')
Kp=0.4597
Ki=679.3775
Ti=1/(Ki/Kp)
pippo=Kp+Ki*Ts*0.5*(z+1)/(z-1)

%% Integral Algorithm
pid = pidstd(C);
Kp = pid.Kp;
Ti = pid.Ti;
ki = Kp*Ts/Ti;
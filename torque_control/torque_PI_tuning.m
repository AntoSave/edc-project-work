clear; clc;
s = tf('s');
%% Motor Model
% Motor parameters
L = 2.3e-3;
R = 3.4;
Kt = 1.32;

G = 1/(s*L+R);

%% Controller tuning
rltool

% Sampling time
bode(feedback(C*G,1)) % Closed loop bandwidth of the system is 200 rad/s
W_omega3 = 200;
omega_s_min = 5*W_omega3;
Ts_max = 2*pi/omega_s_min;

Ts = 0.005;


%% De-noising
Tf = Ts/2;
bode(1/(1+s*Tf))
omega_3f = 1/Tf;
f_cut = omega_3f/(2*pi);
Cf = 1/(f_cut*11000) - 1e-9

% RC Filter
R_f = 1.7e3;
C_base = 1e-9;
G_f = 1/(1+s*R_f*(C_base+300e-9))
bode(G_f)

R_f = 1.7e3;
C_base = 1e-9;
G_f = 1/(1+s*R_f*(C_base+10e-6))
bode(G_f)

hold on
step(G)
step(G*G_f)

%% Fourier analisys
y = current.Data
x = current.Time
[y2,x2] = resample(y,x);
plot(x, y)
hold on
plot(x2,y2)

x3 = x2(x2 > 3);
y3 = y2(x2 > 3);
plot(x3,y3)

fourier = fft(x3); 
m = abs(fourier);
f = (0:length(fourier)-1)*300/length(fourier); 
plot(f,m)

clear; clc;
%% Run the code on the motor
mdl = "pololu_37D_pos_second_order_identification_sim";
open(mdl)
Ts = 0.005;
overline_u = 12;
% Then click on 'Monitor & Tune'

%% Step response analisys
% Remove idle time
y = speed.Data(speed.Time >= 2.0 & speed.Time <= 5.0);
x = speed.Time(speed.Time >= 2.0 & speed.Time <= 5.0) - 2;
stairs(x,y)

% Interpolate signal
% x_interp = interp(x,10);
% y_interp = interp(y,10);
% hold on
% stairs(x_interp, y_interp)

% Sgolay
% y_filt = sgolayfilt(y_interp,2,51);
% hold on
% stairs(x_interp,y_filt)

% Ital filter
% [y_filt,d] = lowpass(y_interp,1.0,(1/0.005)*10)
% hold on
% stairs(x_interp,y_filt)
% y_filt2 = filtfilt(d,y_interp);
% stairs(x_interp,y_filt2)

% Get overline_y
overline_y = mean(speed.Data(speed.Time >= 2.5 & speed.Time <= 5.0));
mu = overline_y/overline_u;

% Fourier analisys
% y_ft = fft(x_interp);
% Ts = 0.05;
% fs = 1/Ts * 10;
% n = length(y_ft);          % number of samples
% f = (0:n-1)*(fs/n);     % frequency range
% power = abs(y_ft).^2/n;    % power of the DFT
% plot(f,power)
% xlabel('Frequency')
% ylabel('Power')

%% Euler approximation
% Get S1 (the area above the curve)
S1 = cumsum((overline_y-y)*Ts);
S1 = mean(S1(end-400:end));

% Get T+tau
t_plus_tau = S1/overline_y;
y_cut = speed.Data(speed.Time >= 2.0 & speed.Time <= 2.0 + t_plus_tau);

% Get S2 (the area under the curve)
S2 = sum((y_cut)*Ts);

% Get T, tau, mu
T = S2*exp(1)/overline_y;
tau = t_plus_tau - T;

% Plot the resulting model
s = tf('s');
G_no_delay = mu/(1+T*s);
G = G_no_delay * exp(-tau*s);

stairs(x,y/overline_u);
hold on
step(G);
step(G_no_delay);

% Extract position model
G_pos = G_no_delay/s;
G_pos = ss(G_pos);
T = obsv(G_pos.A, G_pos.C);
A = T*G_pos.A*T^-1;
B = T*G_pos.B;
C = G_pos.C*T^-1;
D = 0;
G_pos = ss(A,B,C,D);
G_pos = xperm(G_pos, [2 1]);
save("pololu_37D_pos_second_order", "G_pos");


%% Trapezoid approximation
% Get S1 (the area above the curve)
S1 = cumtrapz(x,overline_y-y);
S1 = mean(S1(end-400:end));

% Get T+tau
t_plus_tau = S1/overline_y;
y_cut = speed.Data(speed.Time >= 2.0 & speed.Time <= 2.0 + t_plus_tau);

% Get S2 (the area under the curve)
S2 = trapz((y_cut)*Ts);

% Get T, tau, mu
T = S2*exp(1)/overline_y;
tau = t_plus_tau - T;

% Plot the resulting model
s = tf('s');
G_no_delay = mu/(1+T*s);
G = G_no_delay * exp(-tau*s);

stairs(x,y/overline_u);
hold on
step(G);
step(G_no_delay);


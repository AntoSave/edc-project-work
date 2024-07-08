clear; clc;
%% Run the code on the motor
mdl = "pololu_37D_pos_second_order_identification_sim";
open(mdl)
% Now click on 'Monitor & Tune' and wait for the simulation to finish

%% Prepare data
time = speed_mean.Time;
speed = speed_mean.Data;

Ts = 0.004; % This is not the same as control sampling time!
overline_u = 6; % We give a reference of 6V
step_time = 2.0;
transient_end_time = 0.55;
response_end_time = 1.0;
epsilon = Ts/20;

%% Step response analisys
% Remove idle time

y = speed(time >= step_time & time <= step_time + response_end_time + epsilon);
x = time(time >= step_time & time <= step_time + response_end_time + epsilon) - step_time;

filter = generate_fir_filt();
y_filt_full = filtfilt(filter.Numerator, 1, speed(time >= step_time));
y_filt_full = interp(y_filt_full,4);
x_filt_full = [0:0.001:size(y_filt_full,1)*0.001-0.001].';
y_filt = y_filt_full(x_filt_full<=x(end));
x_filt = x_filt_full(x_filt_full<=x(end));
%filter = generate_iir_filt();
%y_filt = filtfilt(filter.sosMatrix, filter.ScaleValues, y);

figure
hold on
stairs(x,y)
plot(x_filt,y_filt)
xlim([0 response_end_time])

% Get overline_y
overline_y = mean(y_filt(x_filt >= transient_end_time & x_filt <= response_end_time));
mu = overline_y/overline_u;
plot([0; response_end_time],[overline_y; overline_y])


%% Area method with Euler approximation
% % Get S1 (the area above the curve)
% S1 = sum((overline_y-y)*Ts);
% 
% % Get T+tau
% t_plus_tau = S1/overline_y;
% y_cut = y(x <= t_plus_tau + epsilon);
% plot([t_plus_tau; t_plus_tau], [0; overline_y])
% plot(x(x<=t_plus_tau + epsilon), y_cut)
% % Get S2 (the area under the curve)
% S2 = sum((y_cut)*Ts);
% 
% % Get T, tau, mu
% T = S2*exp(1)/overline_y;
% tau = max(0,t_plus_tau - T);

%% Area method with trapezoid approximation
% Get S1 (the area above the curve)
S1 = trapz(x_filt,overline_y-y_filt);

% Get T+tau
t_plus_tau = S1/overline_y;
y_cut = y_filt(x_filt <= t_plus_tau + epsilon);
x_cut = x_filt(x_filt <= t_plus_tau + epsilon);
plot([t_plus_tau; t_plus_tau], [0; overline_y])
plot(x_cut, y_cut)

% Get S2 (the area under the curve)
S2 = trapz(x_cut,y_cut);

% Get T, tau, mu
T = S2*exp(1)/overline_y;
tau = t_plus_tau - T;

%% Plot the resulting model
s = tf('s');
G_no_delay = mu/(1+T*s);
G = G_no_delay * exp(-tau*s);

figure
hold on
plot(x_filt_full,y_filt_full/overline_u);
step(G);
step(G_no_delay);
xlim([0,1])
legend('Measured speed', 'Model with delay', 'Model without delay')


%% Extract position model from speed model
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

%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;

%% Controllability and Osservability
WR = [sys.B sys.A*sys.B]
rank(WR)

W0 = [sys.C; sys.C*sys.A]
rank(W0)
% Entrambe le matrici hanno rango pieno, quindi il sistema è sia
% raggiungibile che controllabile
%% Take the tf with only speed output
s=tf('s')
G=tf(sys)
G_speed=minreal(G*s)

G_speed0 = evalfr(G_speed,0) % Gain of tf
G_speed_norm = G_speed/G_speed0;

bode(G_speed_norm)

figure()
step(G_speed)
hold on
step(G_speed_norm)

open_loop_info = stepinfo(G_speed_norm)
%% Wrong K_v
syms R_a K_P delta_k_V L_a K_I K_m I_m F_m
A = [-(R_a+K_P)/L_a delta_k_V/L_a K_I/L_a;
    K_m/I_m -F_m/I_m 0;
    -1 0 0];
charpoly(A)

%limit
l = (F_m*K_P + I_m*K_I + F_m*R_a) / K_m;

%% Noise disturbance
s=tf('s');
L_a = 2.3e-3;
R_a = 1.7;
K_P=0.0460;
K_I=34;

C = K_P + K_I/s;
P = 1/(s*L_a+R_a);
H = 1/(1+s/100);
W_d = P/(1+C*P*H);
bode(W_d)
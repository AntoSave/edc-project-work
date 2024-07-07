s=tf('s');


%% Electrical part
L = 2.3e-3;
Ra = 1.7;
K = 1.339;
G_e = 1/(s*L+R);

%% Mechanical part
K = 1.32;
G_m = zpk([],[-0.6847],40.9537,'DisplayFormat','Time Constants');
G_m0 = evalfr(G_m,0);
tau = -1/G_m.P{1};
F_m = K/G_m0;
I_m = tau*F_m;

%% Measurement model
r1 = 0.0034; %Current measurement variance
r2 = 0.0043; %Speed measurement variance
R = [r1 0; 0 r2];
Q = [0.05; 0]; %Process covariance matrix


A=[-Ra/L -K/L; K -F_m/I_m];
B=[1/L; 0];
C=[1 0; 0 1];

discrete_sys=ss(A,B,C,0,0.001);
Ad=discrete_sys.A;
Bd=discrete_sys.B;
Cd=discrete_sys.C;

%% Model 2

A=[-Ra/L];
B=[1/L -K/L];
C=[1];
sys = ss(A,B,C,0);
sysd=c2d(sys,0.005);
[Ad,Bd,Cd,Dd]=ssdata(sysd)
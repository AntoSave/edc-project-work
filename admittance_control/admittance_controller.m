% Incremental controller
k = [6.3096 302.9280 26.1428];
%% Admittance filter damped
Md = 0.005;
KD = 0.02;
KP = 0.02;

A=tf([1],[Md,KD,KP]);
A_discr=c2d(A,0.005,'Tustin');

%% Admittance filter springy
Md = 0.002;
KD = 0.003;
KP = 0.01;

A=tf([1],[Md,KD,KP]);
A_discr=c2d(A,0.005,'Tustin');
Md = 0.01;
KD = 0.03;
KP = 0.4;

A=tf([1],[Md,KD,KP]);
A_discr=c2d(A,0.005,'Tustin');
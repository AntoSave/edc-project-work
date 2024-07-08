Md = 0.005;
KD = 0.02;
KP = 0.02;

A=tf([1],[Md,KD,KP]);
A_discr=c2d(A,0.005,'Tustin');
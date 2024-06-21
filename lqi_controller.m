%% 
sys = ss(A,B,C,D);

Qu = 6e-6;
Qx = [10 0 0;0 0.1 0;0 0 1];
[K,S,e] = lqi(sys,Qx,Qu,0)
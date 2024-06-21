%% Import model
load("pololu_37D_pos_second_order", "G_pos");
sys = G_pos;

%% Controller tuning
Qu = 6e-6;
Qx = [10 0 0;0 0.1 0;0 0 1];
[K,S,e] = lqi(sys,Qx,Qu,0)
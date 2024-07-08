%% Torque step
load('torque_step')
figure
plot(data)
xlim([0,3.5])
ylabel('Torque [Nm]')
title('Torque controller step response')
legend('r','y')
%% Torque stairs
load('torque_stair')
figure
plot(data)
xlim([0,10])
ylabel('Torque [Nm]')
title('Torque controller stair of steps response')
legend('r','y')


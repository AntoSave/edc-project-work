%% Position step
load('position_step')
figure
hold on
plot(data{1}.Values)
plot(data{2}.Values)
xlim([0,1.5])
ylabel('Position [deg]')
title('Position controller step response')
legend('r','y')
%% Position stairs
load('position_stair')
figure
hold on
plot(data{1}.Values)
plot(data{2}.Values)
xlim([0,12])
ylabel('Position [deg]')
title('Position controller stair of steps response')
legend('r','y')
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

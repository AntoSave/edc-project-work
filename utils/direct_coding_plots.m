%% Position control
data = readFromCOM('COM3',true)
time = data(:,1);
u = data(:,2);
pos = data(:,3);
reference = data(:,4);
figure
hold on
plot(time*0.001,reference)
plot(time*0.001,pos)
xlabel('Time [s]')
ylabel('Position [rad]')
legend('r','y')
xlim([500*0.001,time(end)*0.001])

%% Torque control
data = readFromCOM('COM3',true)
time = data(:,1);
u = data(:,2);
torque = data(:,3);
reference = data(:,4);
figure
hold on
plot(time,reference)
plot(time,torque)
xlabel('Time [sample]')
ylabel('Torque [Nm]')
legend('r','y')
xlim([500,time(end)])
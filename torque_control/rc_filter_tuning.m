%% Load the data from file
load("rc_filter_data.mat");

%% Plot the time responses with varying C
subplot(2,1,1)
title('Time response (full view)')
xlabel('Time [s]')
ylabel('Current [A]')
hold on
plot(current_no_filt)
plot(current_100nF)
plot(current_300nF)
plot(current_400nF)
plot(current_1muF)
plot(current_10muF)
xlim([0 5])
legend('No filter', '100nF', '300nF', '400nF', '1muF', '10muF')
subplot(2,1,2)
title('Time response (detail view)')
xlabel('Time [s]')
ylabel('Current [A]')
hold on
plot(current_no_filt)
plot(current_100nF)
plot(current_300nF)
plot(current_400nF)
plot(current_1muF)
plot(current_10muF)
xlim([1.99 2.50])
legend('No filter', '100nF', '300nF', '400nF', '1muF', '10muF')

%% Bode plot of the RC filters
Rf = 1.7e3;
s = tf('s');
H_100nF = 1/(1+s*Rf*(100e-9+1e-9));
H_300nF = 1/(1+s*Rf*(300e-9+1e-9));
H_400nF = 1/(1+s*Rf*(400e-9+1e-9));
H_1muF = 1/(1+s*Rf*(1e-6+1e-9));
H_10muF = 1/(1+s*Rf*(10e-6+1e-9));

figure
hold on
bode(H_100nF)
bode(H_300nF)
bode(H_400nF)
bode(H_1muF)
bode(H_10muF)
legend('100nF', '300nF', '400nF', '1muF', '10muF')

%% Spectral analisys
current_no_filt_tt = timeseries2timetable(current_no_filt);
current_100nF_tt = timeseries2timetable(current_100nF);
current_300nF_tt = timeseries2timetable(current_300nF);
current_400nF_tt = timeseries2timetable(current_400nF);
current_1muF_tt = timeseries2timetable(current_1muF);
current_10muF_tt = timeseries2timetable(current_10muF);

%% Variance and range - Motor off
timeLimits = [0 1.99]; % seconds
current_no_filt_ROI = squeeze(current_no_filt.Data(current_no_filt.Time>=timeLimits(1) & current_no_filt.Time<=timeLimits(2)));
current_100nF_ROI = squeeze(current_100nF.Data(current_100nF.Time>=timeLimits(1) & current_100nF.Time<=timeLimits(2)));
current_300nF_ROI = squeeze(current_300nF.Data(current_300nF.Time>=timeLimits(1) & current_300nF.Time<=timeLimits(2)));
current_400nF_ROI = squeeze(current_400nF.Data(current_400nF.Time>=timeLimits(1) & current_400nF.Time<=timeLimits(2)));
current_1muF_ROI = squeeze(current_1muF.Data(current_1muF.Time>=timeLimits(1) & current_1muF.Time<=timeLimits(2)));
current_10muF_ROI = squeeze(current_10muF.Data(current_10muF.Time>=timeLimits(1) & current_10muF.Time<=timeLimits(2)));

current_no_filt_ROI_std = std(current_no_filt_ROI)
current_100nF_ROI_std = std(current_100nF_ROI)
current_300nF_ROI_std = std(current_300nF_ROI)
current_400nF_ROI_std = std(current_400nF_ROI)
current_1muF_ROI_std = std(current_1muF_ROI)
current_10muF_ROI_std = std(current_10muF_ROI)

current_no_filt_ROI_range = max(current_no_filt_ROI)-min(current_no_filt_ROI)
current_100nF_ROI_range = max(current_100nF_ROI)-min(current_100nF_ROI)
current_300nF_ROI_range = max(current_300nF_ROI)-min(current_300nF_ROI)
current_400nF_ROI_range = max(current_400nF_ROI)-min(current_400nF_ROI)
current_1muF_ROI_range = max(current_1muF_ROI)-min(current_1muF_ROI)
current_10muF_ROI_range = max(current_10muF_ROI)-min(current_10muF_ROI)


%% Variance and range - Motor on
timeLimits = [2.5 5.0]; % seconds
current_no_filt_ROI = squeeze(current_no_filt.Data(current_no_filt.Time>=timeLimits(1) & current_no_filt.Time<=timeLimits(2)));
current_100nF_ROI = squeeze(current_100nF.Data(current_100nF.Time>=timeLimits(1) & current_100nF.Time<=timeLimits(2)));
current_300nF_ROI = squeeze(current_300nF.Data(current_300nF.Time>=timeLimits(1) & current_300nF.Time<=timeLimits(2)));
current_400nF_ROI = squeeze(current_400nF.Data(current_400nF.Time>=timeLimits(1) & current_400nF.Time<=timeLimits(2)));
current_1muF_ROI = squeeze(current_1muF.Data(current_1muF.Time>=timeLimits(1) & current_1muF.Time<=timeLimits(2)));
current_10muF_ROI = squeeze(current_10muF.Data(current_10muF.Time>=timeLimits(1) & current_10muF.Time<=timeLimits(2)));

current_no_filt_ROI_std = std(current_no_filt_ROI)
current_100nF_ROI_std = std(current_100nF_ROI)
current_300nF_ROI_std = std(current_300nF_ROI)
current_400nF_ROI_std = std(current_400nF_ROI)
current_1muF_ROI_std = std(current_1muF_ROI)
current_10muF_ROI_std = std(current_10muF_ROI)

current_no_filt_ROI_range = max(current_no_filt_ROI)-min(current_no_filt_ROI)
current_100nF_ROI_range = max(current_100nF_ROI)-min(current_100nF_ROI)
current_300nF_ROI_range = max(current_300nF_ROI)-min(current_300nF_ROI)
current_400nF_ROI_range = max(current_400nF_ROI)-min(current_400nF_ROI)
current_1muF_ROI_range = max(current_1muF_ROI)-min(current_1muF_ROI)
current_10muF_ROI_range = max(current_10muF_ROI)-min(current_10muF_ROI)
% Calculating mean and std dev. of stationary IMU data
% Author: Robin Yohannan

% Get bias in magnetometer data
clear;
fig1 = figure;
mag1 = readtable('stat_2020-02-07-17-04-48_mag.csv', 'HeaderLines', 1);
mag2 = readtable('stat_2020-02-07-17-06-57_mag.csv', 'HeaderLines', 1);
mag3 = readtable('stat_2020-02-07-17-08-22_mag.csv', 'HeaderLines', 1);
mag4 = readtable('stat_2020-02-07-17-09-44_mag.csv', 'HeaderLines', 1);
mag1 = mag1{:,[5, 6, 7]};
mag2 = mag2{:,[5, 6, 7]};
mag3 = mag3{:,[5, 6, 7]};
mag4 = mag4{:,[5, 6, 7]};
last = min([length(mag1) length(mag2) length(mag3) length(mag4)]);
mag_mean = (mag1(1:last,:) + mag2(1:last,:) + mag3(1:last,:)  + mag4(1:last,:))/4.0;
plot((1:size(mag_mean)), mag_mean);
lgd = legend([{'mag x'},{'mag y'},{'mag z'}]);
lgd.FontSize = 20;
grid on;
mmean = mean(mag_mean)
mstd = std(mag_mean)

fig2 = figure;
imu1 = readtable('stat_2020-02-07-17-04-48_imu.csv', 'HeaderLines', 1);
imu2 = readtable('stat_2020-02-07-17-06-57_imu.csv', 'HeaderLines', 1);
imu3 = readtable('stat_2020-02-07-17-08-22_imu.csv', 'HeaderLines', 1);
imu4 = readtable('stat_2020-02-07-17-09-44_imu.csv', 'HeaderLines', 1);
g1 = imu1{:,[18, 19, 20]};
g2 = imu2{:,[18, 19, 20]};
g3 = imu3{:,[18, 19, 20]};
g4 = imu4{:,[18, 19, 20]};
last = min([length(g1) length(g2) length(g3) length(g4)]);
gyro_mean = (g1(1:last,:) + g2(1:last,:) + g3(1:last,:)  + g4(1:last,:))/4.0;
subplot(1, 3, 1);
plot((1:size(gyro_mean)), gyro_mean(:,1), 'Color', [0 0.4470 0.7410], 'DisplayName', 'gyro x');
lgd = legend;
lgd.FontSize = 20;
grid on;
subplot(1, 3, 2);
plot((1:size(gyro_mean)), gyro_mean(:,2), 'Color',[0.8500 0.3250 0.0980], 'DisplayName', 'gyro y');
lgd = legend;
lgd.FontSize = 20;
grid on;
subplot(1, 3, 3);
plot((1:size(gyro_mean)), gyro_mean(:,3), 'Color',[0.9290 0.6940 0.1250], 'DisplayName', 'gyro z');
lgd = legend;
lgd.FontSize = 20;
grid on;
gmean = mean(gyro_mean)
gstd = std(gyro_mean)

fig3 = figure;
a1 = imu1{:,[30, 31, 32]};
a2 = imu2{:,[30, 31, 32]};
a3 = imu3{:,[30, 31, 32]};
a4 = imu4{:,[30, 31, 32]};
last = min([length(a1) length(a2) length(a3) length(a4)]);
acc_mean = (a1(1:last,:) + a2(1:last,:) + a3(1:last,:)  + a4(1:last,:))/4.0;
subplot(1, 3, 1);
plot((1:size(acc_mean)), acc_mean(:,1), 'Color', [0 0.4470 0.7410], 'DisplayName', 'acc x');
lgd = legend;
lgd.FontSize = 20;
grid on;
subplot(1, 3, 2);
plot((1:size(acc_mean)), acc_mean(:,2), 'Color',[0.8500 0.3250 0.0980], 'DisplayName', 'acc y');
lgd = legend;
lgd.FontSize = 20;
grid on;
subplot(1, 3, 3);
plot((1:size(acc_mean)), acc_mean(:,3), 'Color',[0.9290 0.6940 0.1250], 'DisplayName', 'acc z');
lgd = legend;
lgd.FontSize = 20;
grid on;
amean = mean(acc_mean)
astd = std(acc_mean)
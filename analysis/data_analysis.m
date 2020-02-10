% EECE 5554 Section 2
% Lab 2 Analysis
% Author: Robin Yohannan
clear;
close all;
load('stat_data.mat');
load('traj_data.mat');
% GPS data
traj_gps_out(1, :) = [];
gps_secs = traj_gps_out.secs;
gps_nsecs = traj_gps_out.nsecs;
easting = traj_gps_out.utm_east;
northing = traj_gps_out.utm_north;

% IMU data
traj_imu_out(1, :) = [];
imu_secs = traj_imu_out.secs;
imu_nsecs = traj_imu_out.nsecs;
% Quaternion to Euler Angle Conversion
q_x = traj_imu_out.ori_x;
q_y = traj_imu_out.ori_y;
q_z = traj_imu_out.ori_z;
q_w = traj_imu_out.ori_w;
roll = atan2(2*(q_w.*q_x + q_y.*q_z), 1 - 2*(q_x.*q_x + q_y.*q_y));
sin_pitch = 2*(q_w.*q_y - q_z.*q_x);
complex_pitch = asin(sin_pitch).*(abs(sin_pitch) < 1.0);
out_of_range = (sign(sin_pitch)*(pi/2)).*(abs(sin_pitch) >= 1.0);
pitch = complex_pitch + out_of_range;
yaw = atan2(2*(q_w.*q_z + q_x.*q_y), 1 - 2*(q_y.*q_y + q_z.*q_z));
% Gyro and Accel Data
ang_x = traj_imu_out.ang_x;
ang_y = traj_imu_out.ang_y;
ang_z = traj_imu_out.ang_z;
acc_x = traj_imu_out.lin_x;
acc_y = traj_imu_out.lin_y;
acc_z = traj_imu_out.lin_z;

% Calculating dt for integral
imu_nsecs = imu_nsecs(4000:end,:);
time_imu = (imu_nsecs - min(imu_nsecs))/1000000000;
dt_imu = [time_imu(1); diff(time_imu)];
Fs = 1/mean(dt_imu);

% Magnetometer Data
traj_mag_out(1, :) = [];
mag_secs = traj_mag_out.secs;
mag_nsecs = traj_mag_out.nsecs;
mag_x = traj_mag_out.mag_x;
mag_y = traj_mag_out.mag_y;
mag_z = traj_mag_out.mag_z;

% Getting sensor bias while stationary
[mag_bias, gyro_bias, acc_bias] = get_bias();

% Determining hard and soft iron effects for magnetometer calibration
% scatter(mag_x(1000:4000,:)+0.0106, mag_y(1000:4000,:)+0.0085, 5, 'filled');
% ax = gca;
% ellipse = fit_ellipse(mag_x(1000:4000,:)+0.0106, mag_y(1000:4000,:)+0.0085, ax);

fig1 = figure;
% Integrate the gyro z-axis angular velocity to get yaw
% subplot(2, 1, 1);
ang_z = ang_z(4000:end,:);
ang_z_corr = ang_z + gyro_bias(1,3);
subplot(2, 4, 1);
plot_fft(ang_z_corr, Fs, length(ang_z_corr));
ang_z_corr = highpass(ang_z_corr, 5, Fs);
subplot(2, 4, 5);
plot_fft(ang_z_corr, Fs, length(ang_z_corr));

mag_x = mag_x(4000:end,:);
subplot(2, 4, 2);
plot_fft(mag_x, Fs, length(mag_x));
% mag_x = lowpass(mag_x, 50, Fs);
subplot(2, 4, 6);
plot_fft(mag_x, Fs, length(mag_x));

mag_y = mag_y(4000:end,:);
subplot(2, 4, 3);
plot_fft(mag_y, Fs, length(mag_y));
% mag_y = lowpass(mag_y, 50, Fs);
subplot(2, 4, 7);
plot_fft(mag_y, Fs, length(mag_y));

steps = (1:length(ang_z_corr));
steps = steps';
subplot(2, 4, 4);
yaw_gyro = cumtrapz(ang_z_corr.*dt_imu);
plot(steps, yaw_gyro);
% subplot(2, 1, 2);
yaw_mag = unwrap(-atan2(mag_y+0.0085, mag_x+0.0106));
subplot(2, 4, 8);
plot(steps, yaw_mag);
% subplot(3, 1, 3);
% yaw_mag = wrapToPi(yaw_mag);
fig4 = figure;
yaw_est = 0.75*yaw_gyro + 0.25*yaw_mag;
subplot(2, 1, 1);
plot((1:length(yaw_est)), yaw_est);
subplot(2, 1, 2);
plot((1:length(yaw(4000:end,:))), unwrap(yaw(4000:end,:)));
yaw_est = wrapToPi(yaw_est);

%Integrate forward acceleration into forward velocity
acc_x = acc_x(4000:end,:);
acc_y = acc_y(4000:end,:);
acc_z = acc_z(4000:end,:);
steps = (1:length(acc_x));
steps = steps';
% corr_acc_x = acc_x-acc_bias(1,1);
% corr_acc_y = acc_y-acc_bias(1,2);
% corr_acc_z = acc_z-acc_bias(1,3);
corr_acc_x = acc_x;
corr_acc_y = acc_y;
corr_acc_z = acc_z;
% corr_acc_x = corr_acc_x - mean(corr_acc_x);
% corr_acc_y = corr_acc_y - mean(corr_acc_y);
% corr_acc_z = corr_acc_z - mean(corr_acc_z);

% L = length(corr_acc_x);
% plot_fft(corr_acc_x, Fs, L);
% 
fig = figure;
subplot(2, 3, 1);
plot_fft(corr_acc_x, Fs, length(corr_acc_x));
corr_acc_x = lowpass(corr_acc_x, 100, Fs);
subplot(2, 3, 4);
plot_fft(corr_acc_x, Fs, length(corr_acc_x));
subplot(2, 3, 2);
plot_fft(corr_acc_y, Fs, length(corr_acc_y));
corr_acc_y = lowpass(corr_acc_y, 75, Fs);
subplot(2, 3, 5);
plot_fft(corr_acc_y, Fs, length(corr_acc_y));
subplot(2, 3, 3);
plot_fft(corr_acc_z, Fs, length(corr_acc_z));
corr_acc_z = lowpass(corr_acc_z, 100, Fs);
subplot(2, 3, 6);
plot_fft(corr_acc_z, Fs, length(corr_acc_z));
% 
% plot_fft(corr_acc_x, Fs, L);
fig_rest = figure;
% acc_x_ned = zeros(length(corr_acc_x), 1);
% acc_y_ned = zeros(length(corr_acc_y), 1);
% acc_z_ned = zeros(length(corr_acc_z), 1);
% for i=1:length(acc_y)
% %     R_ned2b = eulerToRotationMatrix(roll(i), pitch(i), yaw(i));
% %     R_b2ned = R_ned2b';
%     
% %     temp = (R_b2ned * [corr_acc_x(i); corr_acc_y(i); corr_acc_z(i)])+[0; 0; 9.81];
%     R_yaw = [cos(yaw_mag(i)) sin(yaw_mag(i)) 0; -sin(yaw_mag(i)) cos(yaw_mag(i)) 0; 0 0 1];
%     temp = R_yaw*[corr_acc_x(i); corr_acc_y(i); 0];
% %     temp = [corr_acc_x(i); corr_acc_y(i); 0];
%     acc_x_ned(i) = temp(1);
%     acc_y_ned(i) = temp(2);
% %     acc_z_ned(i) = temp(3);
% end

% acc_x_ned(1) = acc_x_ned(2);
% acc_y_ned(1) = acc_x_ned(2);

% vel_x = cumtrapz(acc_x_ned.*dt_imu);
% vel_y = cumtrapz(acc_y_ned.*dt_imu);
vel_x = cumtrapz(corr_acc_x.*dt_imu);
vel_y = cumtrapz(corr_acc_y.*dt_imu);
L = length(vel_x);
subplot(2, 3, 1);
plot_fft(vel_x, Fs, L);
vel_x = bandpass(vel_x, [5, 100], Fs);
% vel_x = lowpass(vel_x, 100, Fs);
% vel_x = highpass(vel_x, 5, Fs);
subplot(2, 3, 2);
plot_fft(vel_x, Fs, L);
subplot(2, 3, 3);
plot_fft(vel_y, Fs, L);
vel_y = bandpass(vel_y, [5, 100], Fs);
% vel_y = lowpass(vel_y, 100, Fs);
% vel_y = highpass(vel_y, 5, Fs);
subplot(2, 3, 4);
plot_fft(vel_y, Fs, L);
forward_vel_int = sqrt(vel_x.^2 + vel_y.^2);
L = length(forward_vel_int);
% subplot(2, 3, 2);
% plot_fft(forward_vel_int, Fs, L);
% forward_vel_int = bandpass(forward_vel_int, [5, 50], Fs);
% subplot(2, 3, 3);
% plot_fft(forward_vel_int, Fs, L);

subplot(2, 3, 5);
plot(steps, forward_vel_int);

subplot(2, 3, 6);
north_vel = diff(northing)./diff(gps_secs);
east_vel = diff(easting)./diff(gps_secs);
forward_vel = sqrt(north_vel.^2 + east_vel.^2);
plot((1:length(forward_vel)), forward_vel);

% north_pos = cumtrapz(vel_x);
% east_pos = cumtrapz(vel_y);
north_pos = cumtrapz(forward_vel_int.*cos(yaw_mag));
east_pos = cumtrapz(forward_vel_int.*sin(yaw_mag));
north_gps = cumtrapz(north_vel);
east_gps = cumtrapz(east_vel);

start_n = north_gps(1);
start_e = east_gps(1);
north_pos = (north_pos - north_pos(1))+start_n;
east_pos = (east_pos - east_pos(1))+start_e;

subplot(2, 3, 6);
scatter(east_pos, north_pos, 3, 'filled');
hold on;
scatter(east_gps, north_gps, 3, 'r', 'filled');
hold off;
% EECE 5554 Section 2
% Lab 2 Analysis
% Author: Robin Yohannan
clear;
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
ang_x = traj_imu_out.ang_x;
ang_y = traj_imu_out.ang_y;
ang_z = traj_imu_out.ang_z;
acc_x = traj_imu_out.lin_x;
acc_y = traj_imu_out.lin_y;
acc_z = traj_imu_out.lin_z;
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
subplot(3, 1, 1);
ang_z_corr = ang_z + gyro_bias(1,3);
steps = (1:length(ang_z));
steps = steps';
yaw_gyro = cumtrapz(ang_z);
plot(steps, unwrap(wrapToPi(yaw_gyro)));
subplot(3, 1, 2);
yaw_mag = -unwrap(atan2(mag_y+0.0085, mag_x+0.0106));
plot(steps, yaw_mag);
subplot(3, 1, 3);
plot(steps, unwrap(yaw));

fig2 = figure;
%Integrate forward acceleration into forward velocity
steps = (1:length(acc_x));
steps = steps';
vel_x = cumtrapz(acc_x);
fig3 = figure;
plot(steps, vel_x);
% plot(mag_x, mag_y);

% yaw_gyro = trapz(ang_x);
% subplot(3, 1, 1);
% plot(steps, rad2deg(roll));
% subplot(3, 1, 2);
% plot(steps, rad2deg(pitch));
% subplot(3, 1, 3);
% plot(steps, rad2deg(yaw));
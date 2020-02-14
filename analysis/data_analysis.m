% EECE 5554 Section 2
% Lab 2 Analysis
% Author: Robin Yohannan
clear;
close all;

gps_data = readtable('2020-02-04-21-19-47_trajectory_gps.csv', 'HeaderLines', 1);
imu_data = readtable('2020-02-04-21-19-47_trajectory_imu.csv', 'HeaderLines', 1);
mag_data = readtable('2020-02-04-21-19-47_trajectory_mag.csv', 'HeaderLines', 1);

gps_time_raw = gps_data{:,1};
gps_time = gps_time_raw - gps_time_raw(1);
gps_secs = gps_time / 1e9;
easting = gps_data{:,8};
northing = gps_data{:,9};

% IMU data
imu_time_raw = imu_data{:,1};
imu_time = imu_time_raw - imu_time_raw(1);
imu_secs = imu_time / 1e9;
imu_traj_secs = imu_secs(4000:end,:);
% Quaternion to Euler Angle Conversion
q_x = imu_data{:,5};
q_y = imu_data{:,6};
q_z = imu_data{:,7};
q_w = imu_data{:,8};
roll = atan2(2*(q_w.*q_x + q_y.*q_z), 1 - 2*(q_x.*q_x + q_y.*q_y));
sin_pitch = 2*(q_w.*q_y - q_z.*q_x);
complex_pitch = asin(sin_pitch).*(abs(sin_pitch) < 1.0);
out_of_range = (sign(sin_pitch)*(pi/2)).*(abs(sin_pitch) >= 1.0);
pitch = complex_pitch + out_of_range;
yaw = atan2(2*(q_w.*q_z + q_x.*q_y), 1 - 2*(q_y.*q_y + q_z.*q_z));

% Gyro and Accel Data
ang_x = imu_data{:,18};
ang_y = imu_data{:,19};
ang_z = imu_data{:,20};
acc_x = imu_data{:,30};
acc_y = imu_data{:,31};
acc_z = imu_data{:,32};

% Calculating dt for integral
dt_imu = [imu_secs(1); diff(imu_secs)];
dt_imu = dt_imu(4000:end,:);
Fs = 1/mean(dt_imu);

% Magnetometer Data
mag_time_raw = mag_data{:,1};
mag_time = mag_time_raw - mag_time_raw(1);
mag_secs = mag_time / 1e9;
mag_x = mag_data{:,5};
mag_y = mag_data{:,6};
mag_z = mag_data{:,7};

% Getting sensor bias while stationary
[mag_bias, gyro_bias, acc_bias] = get_bias();

% Determining hard and soft iron effects for magnetometer calibration
% scatter(mag_x(1000:4000,:)+0.0106, mag_y(1000:4000,:)+0.0085, 5, 'filled');
% ax = gca;
% ellipse = fit_ellipse(mag_x(1000:4000,:)+0.0106, mag_y(1000:4000,:)+0.0085, ax);

% Integrate the gyro z-axis angular velocity to get yaw
% subplot(2, 1, 1);
f1 = figure;
% subplot(3, 1, 1);
ang_z = ang_z(4000:end,:);
ang_z_corr = ang_z - gyro_bias(1,3);
% ang_z_corr = ang_z 
% plot_fft(ang_z_corr, Fs, length(ang_z_corr));
% ang_z_corr = highpass(ang_z_corr, 1, Fs);
% subplot(3, 1, 2);
% plot_fft(ang_z_corr, Fs, length(ang_z_corr));
% subplot(3, 1, 3);
yaw_gyro = cumtrapz(ang_z_corr);
yaw_gyro = yaw_gyro - yaw_gyro(1);
plot(imu_traj_secs, yaw_gyro);
hold on;
% subplot(3, 1, 2);
plot_fft(yaw_gyro, Fs, length(yaw_gyro));
% yaw_gyro = highpass(yaw_gyro, 1, Fs);

% f1 = figure;
% subplot(2, 1, 1);
mag_x = mag_x(4000:end,:);
% plot_fft(mag_x, Fs, length(mag_x));
% subplot(2, 1, 2);
% mag_x = lowpass(mag_x, 0.1, Fs);
% plot_fft(mag_x, Fs, length(mag_x));

mag_y = mag_y(4000:end,:);
% plot_fft(mag_y, Fs, length(mag_y));
% mag_y = lowpass(mag_y, 50, Fs);
% plot_fft(mag_y, Fs, length(mag_y));

yaw_gyro = cumtrapz(ang_z_corr.*dt_imu);
plot(imu_traj_secs, yaw_gyro);
yaw_mag = unwrap(-atan2(mag_y+0.0085, mag_x+0.0106));
yaw_est = 0.75*yaw_gyro + 0.25*yaw_mag;

%Integrate forward acceleration into forward velocity
acc_x = acc_x(4000:end,:);
acc_y = acc_y(4000:end,:);
acc_z = acc_z(4000:end,:);
% corr_acc_x = acc_x-acc_bias(1,1);
% corr_acc_y = acc_y-acc_bias(1,2);
% corr_acc_z = acc_z-acc_bias(1,3);
corr_acc_x = acc_x;
corr_acc_y = acc_y;
corr_acc_z = acc_z;
corr_acc_x = corr_acc_x - mean(corr_acc_x);
corr_acc_y = corr_acc_y - mean(corr_acc_y);
corr_acc_z = corr_acc_z - mean(corr_acc_z);

figw = figure;
subplot(2, 3, 1);
plot_fft(corr_acc_x, Fs, length(corr_acc_x));
% corr_acc_x = lowpass(corr_acc_x, 0.2, Fs);
subplot(2, 3, 2);
plot_fft(corr_acc_x, Fs, length(corr_acc_x));

subplot(2, 3, 3);
plot_fft(corr_acc_y, Fs, length(corr_acc_y));
% corr_acc_y = lowpass(corr_acc_y, 0.2, Fs);
subplot(2, 3, 4);
plot_fft(corr_acc_y, Fs, length(corr_acc_y));

% subplot(2, 3, 5);
% plot_fft(corr_acc_z, Fs, length(corr_acc_z));
% corr_acc_z = bandpass(corr_acc_z, [0.1, 0.5], Fs);
% subplot(2, 3, 6);
% plot_fft(corr_acc_z, Fs, length(corr_acc_z));

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

vel_x = cumtrapz(corr_acc_x);
vel_y = cumtrapz(corr_acc_y.*dt_imu);

figx = figure;
subplot(2, 2, 1);
plot_fft(vel_x, Fs, length(vel_x));
% vel_x = bandpass(vel_x, [5, 100], Fs);
% vel_x = lowpass(vel_x, 100, Fs);
% vel_x = highpass(vel_x, 5, Fs);
subplot(2, 2, 2);
plot_fft(vel_x, Fs, length(vel_x));

subplot(2, 2, 3);
plot_fft(vel_y, Fs, length(vel_y));
% vel_y = bandpass(vel_y, [5, 100], Fs);
% vel_y = lowpass(vel_y, 100, Fs);
% vel_y = highpass(vel_y, 5, Fs);
subplot(2, 2, 4);
plot_fft(vel_y, Fs, length(vel_y));

forward_vel_int = sqrt(vel_x.^2 + vel_y.^2);
fy = figure;
subplot(2, 1, 1);
plot_fft(forward_vel_int, Fs, length(forward_vel_int));
% forward_vel_int = bandpass(forward_vel_int, [5, 50], Fs);
subplot(2, 1, 2);
plot_fft(forward_vel_int, Fs, length(forward_vel_int));

% GPS differentiation for northing/easting velocity
north_vel = diff(northing)./diff(gps_secs);
east_vel = diff(easting)./diff(gps_secs);
forward_vel = sqrt(north_vel.^2 + east_vel.^2);
f5 = figure;
subplot(2, 1, 1);
plot((1:length(forward_vel)), forward_vel);
subplot(2, 1, 2);
plot(imu_traj_secs, forward_vel_int);

% north_pos = cumtrapz(vel_x);
% east_pos = cumtrapz(vel_y);
north_pos = cumtrapz(vel_x.*cos(yaw_mag));
east_pos = cumtrapz(vel_x.*sin(yaw_mag));
north_gps = cumtrapz(north_vel);
east_gps = cumtrapz(east_vel);

north_traj_gps = north_gps(96:end, 1);
east_traj_gps = east_gps(96:end, 1);
start_n = north_traj_gps(1);
start_e = east_traj_gps(1);
north_pos = (north_pos - north_pos(1))+start_n;
east_pos = (east_pos - east_pos(1))+start_e;
fz = figure;
scatter(east_pos, north_pos, 3, 'filled');
hold on;
scatter(east_traj_gps, north_traj_gps, 3, 'r', 'filled');
hold off;
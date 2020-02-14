% EECE 5554 Section 2 - Lab 2
% IMU Calibration and Dead Reckoning
% Author: Robin Yohannan

clear;
close all;
[gps_secs, easting, northing] = get_GPS_data();
[imu_secs, dt_imu, roll, pitch, yaw, ang_x, ang_y, ang_z, acc_x, acc_y, acc_z] = get_IMU_data();
[mag_secs, mag_x, mag_y, mag_z] = get_mag_data();
[ang_x_bias, ang_y_bias, ang_z_bias, acc_x_bias, acc_y_bias, acc_z_bias, mag_x_bias, mag_y_bias, mag_z_bias] = get_stat_bias();

% Index in the imu data where trajectory was started (after going in
% circles)
start_traj = 4000;
imu_traj_secs = imu_secs(start_traj:end,:);

% Calculating dt for integral
dt_imu_traj = dt_imu(start_traj:end,:);
Fs = 1/mean(dt_imu_traj);

% Part 3.1 - Magnetometer Calibration
% Here, a section of the magnetometer data when the car is driving in
% circles is taken and an ellipse is fitted on the data.
circles_mag_x = mag_x(1000:start_traj,:);
circles_mag_y = mag_y(1000:start_traj,:);
ellipse = fit_ellipse(circles_mag_x, circles_mag_y);
mag_x_hard_iron_offset = ellipse.X0_in;
mag_y_hard_iron_offset = ellipse.Y0_in;
mag_y_hard_iron_offset_ang = ellipse.phi;
soft_iron_offset = ellipse.long_axis/ellipse.short_axis;
mag_x_traj_corr = (mag_x(start_traj:end,:)-mag_x_hard_iron_offset);
mag_y_traj_corr = (mag_y(start_traj:end,:)-mag_y_hard_iron_offset)*soft_iron_offset;
circles_mag_x_corr = (circles_mag_x - mag_x_hard_iron_offset);
circles_mag_y_corr = (circles_mag_y - mag_y_hard_iron_offset)*soft_iron_offset;

% Part 3.2 - Yaw Estimate
% Now, the angular velocity around the z-axis is integrated to find the
% estimated yaw.

% Gyroscope needs to be shifted to zero since initial direction is unknown
ang_z_traj = ang_z(start_traj:end,:);
% ang_z_traj_corr = ang_z_traj - mean(ang_z_traj);
ang_z_traj_corr = (ang_z_traj - ang_z_bias)/40.0;
ang_z_traj_corr = ang_z_traj_corr - mean(ang_z_traj_corr);
yaw_gyro = cumtrapz(ang_z_traj_corr);
% With the corrected magnetometer data, the IMU yaw can be found by
% finding the angle between the 2 vectors
mag_x_traj_corr_lp = lowpass(mag_x_traj_corr, 0.005, Fs, 'Steepness', 0.9999);
mag_y_traj_corr_lp = lowpass(mag_y_traj_corr, 0.005, Fs, 'Steepness', 0.9999);
yaw_mag = unwrap(-atan2(mag_y_traj_corr, mag_x_traj_corr));
yaw_mag_lp = unwrap(-atan2(mag_y_traj_corr_lp, mag_x_traj_corr_lp));

alpha = 0.1;
yaw_est = (1 - alpha)*yaw_gyro + alpha*yaw_mag_lp;

% Part 3.3 - Forward Velocity Estimate

subplot(4,1,1); plot(imu_secs, acc_x); grid on;

acc_x_init_bias_removed = acc_x - mean(acc_x(1:200,:));

subplot(4,1,2); plot(imu_secs, acc_x_init_bias_removed); grid on;

first = acc_x_init_bias_removed(1:22137,:);
second = acc_x_init_bias_removed(22137+1:end,:);
second = second - mean(second(1:700,:));
acc_x_second_half_bias_removed = [first;second];

subplot(4,1,3); plot(imu_secs, acc_x_second_half_bias_removed); grid on;

acc_x_unbiased = remove_bias(acc_x_second_half_bias_removed, 0.4, 100);

subplot(4,1,4); plot(imu_secs, acc_x_unbiased); grid on;

acc_x_traj = acc_x_unbiased(start_traj:end,:);

yaw_traj = yaw_mag_lp;

vel_x_traj = cumtrapz(acc_x_traj);

vel_x_traj = vel_x_traj - min(vel_x_traj);

f1 = figure; plot(imu_traj_secs, vel_x_traj);

vel_x_traj = remove_bias(vel_x_traj, 0.4, 100);

f7 = figure; plot(imu_traj_secs, vel_x_traj);

x_dot = vel_x_traj;

vel_x_traj = vel_x_traj/7000.0;

% Calculating the rotation rate around the CoM
yaw_t = yaw(start_traj:end,:);
omega_t = diff(yaw_est)./diff(imu_traj_secs);
omega_vel = vel_x_traj(2:end,:).*omega_t;
acc_y_traj = acc_y(start_traj+1:end,:);
fig11 = figure;
subplot(2,1,1); plot(imu_traj_secs(2:end,:), omega_vel,'DisplayName','omega*Xdot'); grid on; lgd = legend; lgd.FontSize=20;
subplot(2,1,2); plot(imu_traj_secs(2:end,:), acc_y_traj,'DisplayName','ydotdot'); grid on; lgd = legend; lgd.FontSize=20;

vel_n_traj = vel_x_traj.*cos(yaw_traj-(pi/14));
vel_e_traj = vel_x_traj.*sin(yaw_traj-(pi/14));

pos_n = cumtrapz(vel_n_traj);
pos_e = cumtrapz(vel_e_traj);

% GPS differentiation for northing/easting velocity
gps_start = 100;
gps_traj_secs = gps_secs(gps_start:end,:);
northing_traj = northing(gps_start:end,:);
easting_traj = easting(gps_start:end,:);
north_vel_traj = diff(northing_traj)./diff(gps_traj_secs);
east_vel_traj = diff(easting_traj)./diff(gps_traj_secs);
forward_vel_traj = sqrt(north_vel_traj.^2 + east_vel_traj.^2);

f2 = figure; 
subplot(2, 1, 1); plot(imu_traj_secs, vel_x_traj, 'DisplayName', 'Forward Velocity (Acc - Adjusted)'); 
lgd = legend;
lgd.FontSize = 20;
subplot(2, 1, 2); plot(gps_traj_secs(2:end,:), forward_vel_traj, 'DisplayName', 'Forward Velocity (GPS)');
lgd = legend;
lgd.FontSize = 20;
f3 = figure;
pos_n = ((pos_n - pos_n(1)) + northing_traj(1));
pos_e = (pos_e - pos_e(1)) + easting_traj(1);

pos_e = pos_e - pos_e(1);
pos_n = pos_n - pos_n(1);
easting_traj = easting_traj - easting_traj(1);
northing_traj = northing_traj - northing_traj(1);

scatter(pos_e, pos_n, 3, 'filled');
hold on;
scatter(easting_traj, northing_traj, 3, 'r', 'filled');
grid on;
axis equal;
hold off;

% Estimating x_c
acc_y_diff = mean(acc_y_traj) - mean(omega_vel);
dot_omega = diff(omega_t)./diff(imu_traj_secs(1:end-1,:));
x_c_est = acc_y_diff/mean(dot_omega);
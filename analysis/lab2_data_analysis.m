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
% acc_x_unbiased = remove_bias(acc_x, 0.4, 100);

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

% % GPS differentiation for northing/easting velocity
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

% acc_x_traj = acc_x(4000:end,:);
% acc_x_stat_bias = mean(acc_x(1:500,:));

% s = [1 1482 3602 5522 5602 6882 9482 12002 14002 15242 15322 17202 17482 17962 19842 20682 20882 21522 21682 23602 23842 24962 25082 26042 27242 29602];
% acc_x_traj = acc_x(4000:end,:);
% acc_x_traj_corr = acc_x_traj - acc_x_bias;
% x_dot_traj_corr = cumtrapz(acc_x_traj_corr);
% acc_x_traj_corr = lowpass(acc_x_traj_corr, 1.0, Fs, 'Steepness', 0.9999);
% 
% acc_y_traj = acc_y(4000:end,:);
% acc_y_traj_corr = acc_y_traj - acc_y_bias;
% acc_y_traj_corr = lowpass(acc_y_traj_corr, 1.0, Fs, 'Steepness', 0.9999);
% 
% acc_x_s1 = acc_x_traj_corr(s(1):s(2)); acc_x_s2 = acc_x_traj_corr(s(3):s(4)); acc_x_s3 = acc_x_traj_corr(s(5):s(6));
% acc_x_s4 = acc_x_traj_corr(s(7):s(8)); acc_x_s5 = acc_x_traj_corr(s(9):s(10)); acc_x_s6 = acc_x_traj_corr(s(11):s(12));
% acc_x_s7 = acc_x_traj_corr(s(13):s(14)); acc_x_s8 = acc_x_traj_corr(s(15):s(16)); acc_x_s9 = acc_x_traj_corr(s(17):s(18));
% acc_x_s10 = acc_x_traj_corr(s(19):s(20)); acc_x_s11 = acc_x_traj_corr(s(21):s(22)); acc_x_s12 = acc_x_traj_corr(s(23):s(24));
% acc_x_s13 = acc_x_traj_corr(s(25):s(26));
% 
% acc_y_s1 = acc_y_traj_corr(s(1):s(2)); acc_y_s2 = acc_y_traj_corr(s(3):s(4)); acc_y_s3 = acc_y_traj_corr(s(5):s(6));
% acc_y_s4 = acc_y_traj_corr(s(7):s(8)); acc_y_s5 = acc_y_traj_corr(s(9):s(10)); acc_y_s6 = acc_y_traj_corr(s(11):s(12));
% acc_y_s7 = acc_y_traj_corr(s(13):s(14)); acc_y_s8 = acc_y_traj_corr(s(15):s(16)); acc_y_s9 = acc_y_traj_corr(s(17):s(18));
% acc_y_s10 = acc_y_traj_corr(s(19):s(20)); acc_y_s11 = acc_y_traj_corr(s(21):s(22)); acc_y_s12 = acc_y_traj_corr(s(23):s(24));
% acc_y_s13 = acc_y_traj_corr(s(25):s(26));
% 
% t1 = imu_traj_secs(s(1):s(2)+1); t2 = imu_traj_secs(s(3):s(4)+1); t3 = imu_traj_secs(s(5):s(6)+1); 
% t4 = imu_traj_secs(s(7):s(8)+1); t5 = imu_traj_secs(s(9):s(10)+1); t6 = imu_traj_secs(s(11):s(12)+1); 
% t7 = imu_traj_secs(s(13):s(14)+1); t8 = imu_traj_secs(s(15):s(16)+1); t9 = imu_traj_secs(s(17):s(18)+1); 
% t10 = imu_traj_secs(s(19):s(20)+1); t11 = imu_traj_secs(s(21):s(22)+1); t12 = imu_traj_secs(s(23):s(24)+1); 
% t13 = imu_traj_secs(s(25):s(26)+1);
% 
% dt1 = diff(t1); dt2 = diff(t2); dt3 = diff(t3); dt4 = diff(t4); dt5 = diff(t5); dt6 = diff(t6);
% dt7 = diff(t7); dt8 = diff(t8); dt9 = diff(t9); dt10 = diff(t10); dt11 = diff(t11); dt12 = diff(t12);
% dt13 = diff(t13);
% 
% yaw_op = yaw_est;
% yaw_s1 = yaw_op(s(1):s(2)+1); yaw_s2 = yaw_op(s(3):s(4)+1); yaw_s3 = yaw_op(s(5):s(6)+1); 
% yaw_s4 = yaw_op(s(7):s(8)+1); yaw_s5 = yaw_op(s(9):s(10)+1); yaw_s6 = yaw_op(s(11):s(12)+1); 
% yaw_s7 = yaw_op(s(13):s(14)+1); yaw_s8 = yaw_op(s(15):s(16)+1); yaw_s9 = yaw_op(s(17):s(18)+1); 
% yaw_s10 = yaw_op(s(19):s(20)+1); yaw_s11 = yaw_op(s(21):s(22)+1); yaw_s12 = yaw_op(s(23):s(24)+1); 
% yaw_s13 = yaw_op(s(25):s(26)+1);
% 
% o1 = diff(yaw_s1)./dt1; o2 = diff(yaw_s2)./dt2; o3 = diff(yaw_s3)./dt3; o4 = diff(yaw_s4)./dt4;
% o5 = diff(yaw_s5)./dt5; o6 = diff(yaw_s6)./dt6; o7 = diff(yaw_s7)./dt7; o8 = diff(yaw_s8)./dt8;
% o9 = diff(yaw_s9)./dt9; o10 = diff(yaw_s10)./dt10; o11 = diff(yaw_s11)./dt11; o12 = diff(yaw_s12)./dt12; 
% o13 = diff(yaw_s13)./dt13;
% 
% vel_x_s1 = cumtrapz(acc_x_s1.*dt1); vel_x_s2 = cumtrapz(acc_x_s2.*dt2); vel_x_s3 = cumtrapz(acc_x_s3.*dt3);
% vel_x_s4 = cumtrapz(acc_x_s4.*dt4); vel_x_s5 = cumtrapz(acc_x_s5.*dt5); vel_x_s6 = cumtrapz(acc_x_s6.*dt6);
% vel_x_s7 = cumtrapz(acc_x_s7.*dt7); vel_x_s8 = cumtrapz(acc_x_s8.*dt8); vel_x_s9 = cumtrapz(acc_x_s9.*dt9);
% vel_x_s10 = cumtrapz(acc_x_s10.*dt10); vel_x_s11 = cumtrapz(acc_x_s11.*dt11); vel_x_s12 = cumtrapz(acc_x_s12.*dt12);
% vel_x_s13 = cumtrapz(acc_x_s13.*dt13);
% 
% oX1 = cross([zeros(length(o1),1) zeros(length(o1),1) o1],[vel_x_s1 zeros(length(vel_x_s1),1) zeros(length(vel_x_s1),1)]); 
% oX2 = cross([zeros(length(o2),1) zeros(length(o2),1) o2],[vel_x_s2 zeros(length(vel_x_s2),1) zeros(length(vel_x_s2),1)]); 
% oX3 = cross([zeros(length(o3),1) zeros(length(o3),1) o3],[vel_x_s3 zeros(length(vel_x_s3),1) zeros(length(vel_x_s3),1)]); 
% oX4 = cross([zeros(length(o4),1) zeros(length(o4),1) o4],[vel_x_s4 zeros(length(vel_x_s4),1) zeros(length(vel_x_s4),1)]);
% oX5 = cross([zeros(length(o5),1) zeros(length(o5),1) o5],[vel_x_s5 zeros(length(vel_x_s5),1) zeros(length(vel_x_s5),1)]); 
% oX6 = cross([zeros(length(o6),1) zeros(length(o6),1) o6],[vel_x_s6 zeros(length(vel_x_s6),1) zeros(length(vel_x_s6),1)]); 
% oX7 = cross([zeros(length(o7),1) zeros(length(o7),1) o7],[vel_x_s7 zeros(length(vel_x_s7),1) zeros(length(vel_x_s7),1)]); 
% oX8 = cross([zeros(length(o8),1) zeros(length(o8),1) o8],[vel_x_s8 zeros(length(vel_x_s8),1) zeros(length(vel_x_s8),1)]);
% oX9 = cross([zeros(length(o9),1) zeros(length(o9),1) o9],[vel_x_s9 zeros(length(vel_x_s9),1) zeros(length(vel_x_s9),1)]); 
% oX10 = cross([zeros(length(o10),1) zeros(length(o10),1) o10],[vel_x_s10 zeros(length(vel_x_s10),1) zeros(length(vel_x_s10),1)]); 
% oX11 = cross([zeros(length(o11),1) zeros(length(o11),1) o11],[vel_x_s11 zeros(length(vel_x_s11),1) zeros(length(vel_x_s11),1)]); 
% oX12 = cross([zeros(length(o12),1) zeros(length(o12),1) o12],[vel_x_s12 zeros(length(vel_x_s12),1) zeros(length(vel_x_s12),1)]);
% oX13 = cross([zeros(length(o13),1) zeros(length(o13),1) o13],[vel_x_s13 zeros(length(vel_x_s13),1) zeros(length(vel_x_s13),1)]);
% 
% acc_Y_s1 = acc_y_s1-oX1(:,2); acc_Y_s2 = acc_y_s2-oX2(:,2); acc_Y_s3 = acc_y_s3-oX3(:,2); acc_Y_s4 = acc_y_s4-oX4(:,2);
% acc_Y_s5 = acc_y_s5-oX5(:,2); acc_Y_s6 = acc_y_s6-oX6(:,2); acc_Y_s7 = acc_y_s7-oX7(:,2); acc_Y_s8 = acc_y_s8-oX8(:,2);
% acc_Y_s9 = acc_y_s9-oX9(:,2); acc_Y_s10 = acc_y_s10-oX10(:,2); acc_Y_s11 = acc_y_s11-oX11(:,2); acc_Y_s12 = acc_y_s12-oX12(:,2);
% acc_Y_s13 = acc_y_s13-oX13(:,2);
% 
% vel_y_s1 = cumtrapz(acc_Y_s1.*dt1); vel_y_s2 = cumtrapz(acc_Y_s2.*dt2); vel_y_s3 = cumtrapz(acc_Y_s3.*dt3);
% vel_y_s4 = cumtrapz(acc_Y_s4.*dt4); vel_y_s5 = cumtrapz(acc_Y_s5.*dt5); vel_y_s6 = cumtrapz(acc_Y_s6.*dt6);
% vel_y_s7 = cumtrapz(acc_Y_s7.*dt7); vel_y_s8 = cumtrapz(acc_Y_s8.*dt8); vel_y_s9 = cumtrapz(acc_Y_s9.*dt9);
% vel_y_s10 = cumtrapz(acc_Y_s10.*dt10); vel_y_s11 = cumtrapz(acc_Y_s11.*dt11); vel_y_s12 = cumtrapz(acc_Y_s12.*dt12);
% vel_y_s13 = cumtrapz(acc_Y_s13.*dt13);
% 
% vel_x_int = [vel_x_s1; vel_x_s2; vel_x_s3; vel_x_s4; vel_x_s5; vel_x_s6; vel_x_s7; vel_x_s8; vel_x_s9; vel_x_s10; vel_x_s11; vel_x_s12; vel_x_s13];
% t = [t1; t2; t3; t4; t5; t6; t7; t8; t9; t10; t11; t12; t13];
% 
% % vel_n_s1 = vel_x_s1.*sin(yaw_s1); vel_n_s2 = vel_x_s2.*sin(yaw_s2); vel_n_s3 = vel_x_s3.*sin(yaw_s3);
% % vel_n_s4 = vel_x_s4.*sin(yaw_s4); vel_n_s5 = vel_x_s5.*sin(yaw_s5); vel_n_s6 = vel_x_s6.*sin(yaw_s6);
% % vel_n_s7 = vel_x_s7.*sin(yaw_s7); vel_n_s8 = vel_x_s8.*sin(yaw_s8); vel_n_s9 = vel_x_s9.*sin(yaw_s9);
% % vel_n_s10 = vel_x_s10.*sin(yaw_s10); vel_n_s11 = vel_x_s11.*sin(yaw_s11); vel_n_s12 = vel_x_s12.*sin(yaw_s12);
% % vel_n_s13 = vel_x_s13.*sin(yaw_s13);
% % 
% % vel_e_s1 = vel_x_s1.*cos(yaw_est(1:1482)); vel_e_s2 = vel_x_s2.*cos(yaw_est(3602:5522)); vel_e_s3 = vel_x_s3.*cos(yaw_est(5602:6882));
% % vel_e_s4 = vel_x_s4.*cos(yaw_est(9482:12002)); vel_e_s5 = vel_x_s5.*cos(yaw_est(14002:15242)); vel_e_s6 = vel_x_s6.*cos(yaw_est(15322:17202));
% % vel_e_s7 = vel_x_s7.*cos(yaw_est(17482:17962)); vel_e_s8 = vel_x_s8.*cos(yaw_est(19842:20682)); vel_e_s9 = vel_x_s9.*cos(yaw_est(20882:21522));
% % vel_e_s10 = vel_x_s10.*cos(yaw_est(21682:23602)); vel_e_s11 = vel_x_s11.*cos(yaw_est(23842:24962)); vel_e_s12 = vel_x_s12.*cos(yaw_est(25082:26042));
% % vel_e_s13 = vel_x_s13.*cos(yaw_est(27242:29602));
% 
% pos_n_s1 = cumtrapz(vel_x_s1.*dt1); pos_n_s2 = cumtrapz(vel_x_s2.*dt2); pos_n_s3 = cumtrapz(vel_x_s3.*dt3); pos_n_s4 = cumtrapz(vel_x_s4.*dt4);
% pos_n_s5 = cumtrapz(vel_x_s5.*dt5); pos_n_s6 = cumtrapz(vel_x_s6.*dt6); pos_n_s7 = cumtrapz(vel_x_s7.*dt7); pos_n_s8 = cumtrapz(vel_x_s8.*dt8);
% pos_n_s9 = cumtrapz(vel_x_s9.*dt9); pos_n_s10 = cumtrapz(vel_x_s10.*dt10); pos_n_s11 = cumtrapz(vel_x_s11.*dt11); pos_n_s12 = cumtrapz(vel_x_s12.*dt12);
% pos_n_s13 = cumtrapz(vel_x_s13.*dt13);
% 
% pos_e_s1 = cumtrapz(vel_y_s1.*dt1); pos_e_s2 = cumtrapz(vel_y_s2.*dt2); pos_e_s3 = cumtrapz(vel_y_s3.*dt3); pos_e_s4 = cumtrapz(vel_y_s4.*dt4);
% pos_e_s5 = cumtrapz(vel_y_s5.*dt5); pos_e_s6 = cumtrapz(vel_y_s6.*dt6); pos_e_s7 = cumtrapz(vel_y_s7.*dt7); pos_e_s8 = cumtrapz(vel_y_s8.*dt8);
% pos_e_s9 = cumtrapz(vel_y_s9.*dt9); pos_e_s10 = cumtrapz(vel_y_s10.*dt10); pos_e_s11 = cumtrapz(vel_y_s11.*dt11); pos_e_s12 = cumtrapz(vel_y_s12.*dt12);
% pos_e_s13 = cumtrapz(vel_y_s13.*dt13);
% 
% pos_n_s2 = (pos_n_s2 - min(pos_n_s2))+pos_n_s1(end);
% pos_n_s3 = (pos_n_s3 - min(pos_n_s3))+pos_n_s2(end);
% pos_n_s4 = (pos_n_s4 - min(pos_n_s4))+pos_n_s3(end);
% pos_n_s5 = (pos_n_s5 - min(pos_n_s5))+pos_n_s4(end);
% pos_n_s6 = (pos_n_s6 - min(pos_n_s6))+pos_n_s5(end);
% pos_n_s7 = (pos_n_s7 - min(pos_n_s7))+pos_n_s8(end);
% pos_n_s8 = (pos_n_s8 - min(pos_n_s8))+pos_n_s7(end);
% pos_n_s9 = (pos_n_s9 - min(pos_n_s9))+pos_n_s8(end);
% pos_n_s10 = (pos_n_s10 - min(pos_n_s10))+pos_n_s9(end);
% pos_n_s11 = (pos_n_s11 - min(pos_n_s11))+pos_n_s10(end);
% pos_n_s12 = (pos_n_s12 - min(pos_n_s12))+pos_n_s11(end);
% pos_n_s13 = (pos_n_s13 - min(pos_n_s13))+pos_n_s12(end);
% 
% pos_e_s2 = (pos_e_s2 - min(pos_n_s2))+pos_e_s1(end);
% pos_e_s3 = (pos_e_s3 - min(pos_n_s3))+pos_e_s2(end);
% pos_e_s4 = (pos_e_s4 - min(pos_n_s4))+pos_e_s3(end);
% pos_e_s5 = (pos_e_s5 - min(pos_n_s5))+pos_e_s4(end);
% pos_e_s6 = (pos_e_s6 - min(pos_n_s6))+pos_e_s5(end);
% pos_e_s7 = (pos_e_s7 - min(pos_n_s7))+pos_e_s8(end);
% pos_e_s8 = (pos_e_s8 - min(pos_n_s8))+pos_e_s7(end);
% pos_e_s9 = (pos_e_s9 - min(pos_n_s9))+pos_e_s8(end);
% pos_e_s10 = (pos_e_s10 - min(pos_n_s10))+pos_e_s9(end);
% pos_e_s11 = (pos_e_s11 - min(pos_n_s11))+pos_e_s10(end);
% pos_e_s12 = (pos_e_s12 - min(pos_n_s12))+pos_e_s11(end);
% pos_e_s13 = (pos_e_s13 - min(pos_n_s13))+pos_e_s12(end)
% 
% pos_n = [pos_n_s1; pos_n_s2; pos_n_s3; pos_n_s4; pos_n_s5; pos_n_s6; pos_n_s7; pos_n_s8; pos_n_s9; pos_n_s10; pos_n_s11; pos_n_s12; pos_n_s13];
% pos_e = [pos_e_s1; pos_e_s2; pos_e_s3; pos_e_s4; pos_e_s5; pos_e_s6; pos_e_s7; pos_e_s8; pos_e_s9; pos_e_s10; pos_e_s11; pos_e_s12; pos_e_s13];
% 
% vel_n = [vel_x_s1; vel_x_s2; vel_x_s3; vel_x_s4; vel_x_s5; vel_x_s6; vel_x_s7; vel_x_s8; vel_x_s9; vel_x_s10; vel_x_s11; vel_x_s12; vel_x_s13];
% vel_e = [vel_y_s1; vel_y_s2; vel_y_s3; vel_y_s4; vel_y_s5; vel_y_s6; vel_y_s7; vel_y_s8; vel_y_s9; vel_y_s10; vel_y_s11; vel_y_s12; vel_y_s13];
% dt = [dt1; dt2; dt3; dt4; dt5; dt6; dt7; dt8; dt9; dt10; dt11; dt12; dt13];
% 
% % pos_n = cumtrapz(vel_n.*dt);
% % pos_e = cumtrapz(vel_e.*dt);
% pos_n = (pos_n - pos_n(1))+4690000;
% pos_e = (pos_e - pos_e(1))+327800;
% 
% % f2 = figure; subplot(2, 1, 1); plot_fft(acc_x_traj_corr, Fs, length(acc_x_traj_corr));
% % acc_x_traj_corr_lp = acc_x_traj_corr;%lowpass(acc_x_traj_corr, 1, Fs, 'Steepness', 0.9999);
% % subplot(2, 1, 2); plot_fft(acc_x_traj_corr_lp, Fs, length(acc_x_traj_corr_lp));
% 
% % x_dot_traj = cumtrapz(acc_x_traj);
% 
% % GPS differentiation for northing/easting velocity
% gps_traj_secs = gps_secs(96:end,:);
% northing_traj = northing(96:end,:);
% easting_traj = easting(96:end,:);
% north_vel_traj = diff(northing_traj)./diff(gps_traj_secs);
% east_vel_traj = diff(easting_traj)./diff(gps_traj_secs);
% forward_vel_traj = sqrt(north_vel_traj.^2 + east_vel_traj.^2);
% 
% % Integrated north and east position using integrated NE velocity estimates
% % pos_n = zeros(length(vel_n), 1);
% % pos_e = zeros(length(vel_e), 1);
% % pos_n(1) = northing_traj(1);
% % pos_e(1) = easting_traj(1);
% 
% % vel_y_s2 = cumtrapz(acc_y_s2);
% % forward_vel_s2 = sqrt((vel_y_s2/40).^2 + (vel_x_s2/40).^2);
% % fvel_n_s2 = forward_vel_s2.*cos(yaw_s2);
% % fvel_e_s2 = forward_vel_s2.*sin(yaw_s2);
% 
% % pos_n = zeros(length(vel_n_s2), 1);
% % pos_e = zeros(length(vel_e_s2), 1);
% % pos_n(1) = 4690000;
% % pos_e(1) = 327800;
% % vel_e_s2 = (vel_x_s2/40).*cos(yaw_s2);
% % vel_n_s2 = (vel_x_s2/40).*sin(yaw_s2);
% % dt2 = diff(t2);
% 
% % for i = 1:length(dt)
% %     pos_n(i+1) = pos_n(i) + vel_n(i+1)*dt(i);
% %     pos_e(i+1) = pos_e(i) + vel_e(i+1)*dt(i);
% % end
% 
% % for i = 1:length(dt2)
% %     pos_n(i+1) = pos_n(i) + fvel_n_s2(i+1)*dt2(i);
% %     pos_e(i+1) = pos_e(i) + fvel_e_s2(i+1)*dt2(i);
% % end
% 
% % cum_vel = forward_vel_traj(1);
% % x_dot_traj = zeros(length(acc_x_traj_corr_lp), 1);
% % for i = 1:length(acc_x_traj_corr_lp)
% %     cum_vel = cum_vel + acc_x_traj_corr_lp(i)*dt_imu_traj(i);
% %     x_dot_traj(i) = cum_vel;
% % end
% 
% % Plots
% % indices for each circle: 1184, 2170, 2962, 3653
% % subplot(1, 2, 1);
% % scatter(circles_mag_x, circles_mag_y, 5, 'g', 'filled');
% % ax = gca;
% % fit_ellipse(circles_mag_x, circles_mag_y, ax, 'm');
% % axis equal;
% % grid on;
% % subplot(1, 2, 2);
% % scatter(circles_mag_x_corr, circles_mag_y_corr, 5, 'c', 'filled');
% % ax = gca;
% % e2 = fit_ellipse(circles_mag_x_corr, circles_mag_y_corr, ax, 'r');
% % axis equal;
% % grid on;
% % subplot(2, 1, 1);
% % subplot(2, 1, 2);
% % f2 = figure;
% % hold on;
% % plot(imu_traj_secs, yaw_gyro, 'DisplayName', 'gyro yaw');
% % subplot(2, 1, 1);
% % plot(imu_traj_secs, yaw_mag, 'DisplayName', 'mag yaw');
% % lg = legend;
% % lg.FontSize = 20;
% % grid on;
% % subplot(2, 1, 2);
% % plot(imu_traj_secs, yaw_mag_lp, 'DisplayName', 'mag yaw (low-pass)');
% % plot(imu_traj_secs, yaw_est, 'DisplayName', 'combined');
% % plot(imu_traj_secs, unwrap(yaw(4000:end,:)), 'DisplayName', 'IMU');
% % lg = legend;
% % lg.FontSize = 20;
% % grid on;
% % % hold off;
% % f3 = figure;
% % subplot(2, 1, 1); plot(t, vel_x_int);
% % subplot(2, 1, 2); plot(gps_traj_secs(2:end,:), forward_vel_traj);
% f2 = figure;
% scatter(pos_e, pos_n, 3, 'filled');
% hold on;
% scatter(easting_traj, northing_traj, 3, 'r', 'filled');
% hold off;
% % f1 = figure;
% % subplot(2,1,1); plot(t, vel_x_int/40.0, 'DisplayName', 'Forward Velocity (Acc) - Adjusted');
% % grid on;
% % lg = legend;
% % lg.FontSize = 20;
% % subplot(2,1,2); plot(gps_traj_secs(2:end,:), north_vel_traj, 'DisplayName', 'Forward Velocity (GPS)');
% % grid on;
% % lg = legend;
% % lg.FontSize = 20;
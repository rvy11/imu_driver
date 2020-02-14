function [ang_x_bias, ang_y_bias, ang_z_bias, acc_x_bias, acc_y_bias, acc_z_bias, mag_x_bias, mag_y_bias, mag_z_bias] = get_stat_bias()
    stat_imu_data = readtable('2020-02-04-21-18-04_stationary_imu.csv', 'HeaderLines', 1);
    stat_mag_data = readtable('2020-02-04-21-18-04_stationary_mag.csv', 'HeaderLines', 1);
   
    % Gyro and Accel Data
    ang_x = stat_imu_data{:,18};
    ang_y = stat_imu_data{:,19};
    ang_z = stat_imu_data{:,20};
    acc_x = stat_imu_data{:,30};
    acc_y = stat_imu_data{:,31};
    acc_z = stat_imu_data{:,32};
    
    % Magnetometer Data
    mag_x = stat_mag_data{:,5};
    mag_y = stat_mag_data{:,6};
    mag_z = stat_mag_data{:,7};
    
    ang_x_bias = mean(ang_x);
    ang_y_bias = mean(ang_y);
    ang_z_bias = mean(ang_z);
    
    acc_x_bias = mean(acc_x);
    acc_y_bias = mean(acc_y);
    acc_z_bias = mean(acc_z);
    
    mag_x_bias = mean(mag_x);
    mag_y_bias = mean(mag_y);
    mag_z_bias = mean(mag_z);
end
function [imu_secs, dt_imu, roll, pitch, yaw, ang_x, ang_y, ang_z, acc_x, acc_y, acc_z] = get_IMU_data()
    imu_data = readtable('2020-02-04-21-19-47_trajectory_imu.csv', 'HeaderLines', 1);
    
    % IMU time
    imu_time_raw = imu_data{:,1};
    imu_time = imu_time_raw - imu_time_raw(1);
    imu_secs = imu_time / 1e9;
   
    % Calculating dt for integral
    dt_imu = [imu_secs(1); diff(imu_secs)];
    
    % IMU orientations (quaternions)
    q_x = imu_data{:,5};
    q_y = imu_data{:,6};
    q_z = imu_data{:,7};
    q_w = imu_data{:,8};
    
    % Convert quaternions to roll
    roll = atan2(2*(q_w.*q_x + q_y.*q_z), 1 - 2*(q_x.*q_x + q_y.*q_y));
    
    % Convert quaternions to pitch
    sin_pitch = 2*(q_w.*q_y - q_z.*q_x);
    complex_pitch = asin(sin_pitch).*(abs(sin_pitch) < 1.0);
    out_of_range = (sign(sin_pitch)*(pi/2)).*(abs(sin_pitch) >= 1.0);
    pitch = complex_pitch + out_of_range;
    
    % Convert quaternions to yaw
    yaw = atan2(2*(q_w.*q_z + q_x.*q_y), 1 - 2*(q_y.*q_y + q_z.*q_z));
    
    % Gyro and Accel Data
    ang_x = imu_data{:,18};
    ang_y = imu_data{:,19};
    ang_z = imu_data{:,20};
    acc_x = imu_data{:,30};
    acc_y = imu_data{:,31};
    acc_z = imu_data{:,32};
end
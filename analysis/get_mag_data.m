function [mag_secs, mag_x, mag_y, mag_z] = get_mag_data()
    mag_data = readtable('2020-02-04-21-19-47_trajectory_mag.csv', 'HeaderLines', 1);
    % Magnetometer Data
    mag_time_raw = mag_data{:,1};
    mag_time = mag_time_raw - mag_time_raw(1);
    mag_secs = mag_time / 1e9;
    mag_x = mag_data{:,5};
    mag_y = mag_data{:,6};
    mag_z = mag_data{:,7};
end
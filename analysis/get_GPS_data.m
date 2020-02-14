function [gps_secs, easting, northing] = get_GPS_data()
    gps_data = readtable('2020-02-04-21-19-47_trajectory_gps.csv', 'HeaderLines', 1);
    gps_time_raw = gps_data{:,1};
    gps_time = gps_time_raw - gps_time_raw(1);
    gps_secs = gps_time / 1e9;
    easting = gps_data{:,8};
    northing = gps_data{:,9};
end
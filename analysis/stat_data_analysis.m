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
mean = (mag1(1:end-1,:) + mag2 + mag3(1:end-4,:)  + mag4)/4.0;
plot((1:size(mean)), mean);

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
mean = (g1(1:last,:) + g2(1:last,:) + g3(1:last,:)  + g4(1:last,:))/4.0;
plot((1:size(mean)), mean);

fig3 = figure;
a1 = imu1{:,[30, 31, 32]};
a2 = imu2{:,[30, 31, 32]};
a3 = imu3{:,[30, 31, 32]};
a4 = imu4{:,[30, 31, 32]};
last = min([length(a1) length(a2) length(a3) length(a4)]);
mean = (a1(1:last,:) + a2(1:last,:) + a3(1:last,:)  + a4(1:last,:))/4.0;
plot((1:size(mean)), mean);

clear all;close all;
rawdata_gyro = csvread('Gyroscope.csv',1,0);
% gyro_t: convert milli time to sec
gyro_t = rawdata_gyro(:,1)/10^3;
gyro_x = rawdata_gyro(:,3);
gyro_y = rawdata_gyro(:,4);
gyro_z = rawdata_gyro(:,5);

%% visualize sensing rate
plot(gyro_t,gyro_x,gyro_t,gyro_y,gyro_t,gyro_z)
gyro_time = datetime(gyro_t,'convertfrom','posixtime','TimeZone','America/New_York');
plot(diff(gyro_t))
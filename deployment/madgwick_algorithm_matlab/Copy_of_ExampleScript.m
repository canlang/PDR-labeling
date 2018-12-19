% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ?90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script 

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data
addpath('isilab');      % include quaternion library
rawdata_gyr = csvread('Gyroscope.csv',1,0);
rawdata_acc = csvread('Accelerometer.csv',1,0);
rawdata_mag = csvread('MagneticField.csv',1,0);

% 1. 시간의 플롯을 보았을때 센싱인터벌이 일정하지 않은 구간이 존재함.
% 2. 로우데이터를 보았을때 센싱인터벌은 2e-2로 보임
% 3. 이를 기반으로 AHRS를 동작시켜 보려함 (2018-11-8)
if 0
    subplot(211)
    plot(diff(rawdata_acc(:,2)))
    hold on
    plot(diff(rawdata_gyr(:,2)))
    plot(diff(rawdata_mag(:,2)))
    hold off
    legend('acc','gyro','mag')
    subplot(212)
    plot(rawdata_acc(:,2))
    hold on
    plot(rawdata_gyr(:,2))
    plot(rawdata_mag(:,2))
    hold off
    legend('acc','gyro','mag')
end
% hist(diff(rawdata_acc(:,2)),diff(rawdata_gyr(:,2)))
data_avail_range = min([length(rawdata_gyr),length(rawdata_acc),length(rawdata_mag)]);

Accelerometer = rawdata_acc(1:data_avail_range,3:5);
Gyroscope = rawdata_gyr(1:data_avail_range,3:5)*180/pi;
Magnetometer = rawdata_mag(1:data_avail_range,3:5);

% without Magnetometer
Magnetometer = Accelerometer;
% Magnetometer = zeros(size(Magnetometer));
% Magnetometer = repmat(Magnetometer(1,:),length(Magnetometer),1);

time = (rawdata_gyr(1:data_avail_range,2)-rawdata_gyr(1,2))/1e9;
%%
% load('ExampleData.mat');
%%
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm
AHRS = MadgwickAHRS('SamplePeriod', 2e-2, 'Beta', 0.1);
% AHRS = MahonyAHRS('SamplePeriod', 2e-2, 'Kp', 0.5);

% AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

set(gcf,'units','points','position',[1200,500,800,500])
sdf(gcf,'sj2')

print -clipboard -dbitmap

%% Visualize path with Orientation (omitted step detection = assume walking)

location = zeros(length(euler),2);
% aerospace sequence?
for i=1:length(location)
    yaw = euler(i,3)+90;
    step = 0.7;
    trM = [step*(0*cosd(yaw) - 1*sind(yaw));step*(0*sind(yaw) + 1*cosd(yaw))];
    if i == 1
        location(i,:) = (trM+[0;0])';
    else
        location(i,:) = (trM+location(i-1,:)')';
    end
end
figure
plot(location(:,1),location(:,2))

set(gcf,'units','points','position',[1200,500,800,800])
sdf(gcf,'sj2')

% print -clipboard -dbitmap
%% Step detection (by peak detection of L2 norm from accelerometer)

acc_norm = vecnorm(Accelerometer,2,2);
figure
findpeaks(acc_norm,time,'MinPeakProminence',300)

%% End of script
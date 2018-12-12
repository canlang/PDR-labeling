clear;close all;clc;

addpath('input_rawdata')
raw_acc = csvread('Accelerometer.csv',1,0);
raw_gyr = csvread('Gyroscope.csv',1,0);

acc_time = datetime(raw_acc(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');
gyr_time = datetime(raw_gyr(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');

acc_mag = vecnorm(raw_acc(:,3:5),2,2);
acc_mag = acc_mag - mean(acc_mag);

%% resample
T_acc = timetable(seconds(raw_acc(:,2)/1e9),raw_acc(:,3:5),acc_mag);
T_gyr = timetable(seconds(raw_gyr(:,2)/1e9),raw_gyr(:,3:5));
T_acc = sortrows(T_acc);
T_gyr = sortrows(T_gyr);

TT = synchronize(T_acc,T_gyr,'regular','linear','TimeStep',seconds(2e-2));
% TT = synchronize(T_acc,T_gyr,'commonrange','linear','TimeStep',seconds(2e-2));
% TT = synchronize(T_acc,T_gyr,'intersection','TimeStep',seconds(2e-2));
% TT = synchronize(T_acc,T_gyr,'commonrange','linear');

% TT = synchronize(T_acc,T_gyr);
% TT = synchronize(T_acc,T_gyr,'regular','nearest','TimeStep',seconds(2e-2));

% TT = synchronize(T_acc,T_gyr,'commonrange','SampleRate',50,'method','nearest');
% TT = synchronize(T_acc,T_gyr,'intersection','mean');
Accelerometer = TT.Var1_T_acc;
Gyroscope = TT.Var1_T_gyr*180/pi;
acc_mag = TT.acc_mag;

time = seconds(TT.Time(:)-(TT.Time(1)));
rate = median(diff(time)); % cal sample rate

% acc = resample(acc_mag,time,20);
% [Accelerometer,time] = resample(acc_mag, raw_acc(:,2)/1e9,20);

%% find step point (step) and time labeling
% threshold should be tuned experimentally to match a person's level
minPeakHeight = std(acc_mag);

[pks,locs] = findpeaks(acc_mag,'MinPeakDistance',...
    .3/rate,'MinPeakHeight',minPeakHeight);   % .3s 이내의 피크는 무시

%%  
addpath(genpath('madgwick_algorithm_matlab'));
AHRS = MadgwickAHRS('SamplePeriod', rate, 'Beta', 0.1); % sample rate: 2e-2

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Accelerometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.


%%

estloc = zeros(length(locs),2);
% phi,theta,psi: roll pitch yaw (aerospace sequence?)
for i=1:length(locs)
% for i=1:length(estloc)
    t_i = locs(i);
    yaw = euler(t_i,3)+180;
    step = 0.7;
    trM = [step*(1*cosd(yaw) - 0*sind(yaw));step*(1*sind(yaw) + 0*cosd(yaw))];
%     trM = [step*(0*cosd(yaw) - 1*sind(yaw));step*(0*sind(yaw) + 1*cosd(yaw))];
    if all(estloc == 0)
        estloc(i,:) = (trM+[0;0])';
    else
        estloc(i,:) = (trM+estloc(i-1,:)')';
    end
end
%%
figure
plot(estloc(:,1),estloc(:,2),'xr-','MarkerSize',8)
% ylim([-10 50])
axis image
% axis 'auto'
grid on

set(gcf,'units','points','position',[1200,500,1200,800])
sdf(gcf,'sj2')

print -clipboard -dbitmap
clear;close all;clc;

addpath('input_rawdata')
rawdata_acc = csvread('Accelerometer.csv',1,0);
acc_t = rawdata_acc(:,1)/10^3;         % first column is posix
acc_x = rawdata_acc(:,3);
acc_y = rawdata_acc(:,4);
acc_z = rawdata_acc(:,5);

acc_time = datetime(acc_t,'convertfrom','posixtime','TimeZone','America/New_York');
acc_mag = vecnorm(rawdata_acc(:,3:5),2,2);
acc_mag = acc_mag - mean(acc_mag);
time = (rawdata_acc(:,2)-(rawdata_acc(1,2)))/1e9;
rate = median(diff(time)); % cal sample rate

% subplot(211)
% plot(acc_t,acc_x,acc_t,acc_y,acc_t,acc_z)
% 
% subplot(212)
% plot(acc_time,acc_l2norm )
%% find step point (step) and time labeling
vf = 1;

% .3s 이내의 피크는 무시
minPeakHeight = std(acc_mag);       % threshold should be tuned experimentally to match a person's level 
[pks,locs] = findpeaks(acc_mag,'MinPeakDistance',.3/rate,'MinPeakHeight',minPeakHeight);
if vf
    plot(time,acc_mag,time(locs),pks,'or')
    text(time(locs)+.22,pks,num2str((1:numel(pks))'))

    xlabel('Time (sec)')
    ylabel('Acc. (L2norm)')
    axis tight
    set(gcf,'units','points','position',[500,500,1200,800])
    sdf(gcf,'sj2')
end

clear;close all;clc;

addpath('input_rawdata')
raw_acc = csvread('Accelerometer.csv',1,0);
raw_gyr = csvread('Gyroscope.csv',1,0);

acc_time = datetime(raw_acc(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');
gyr_time = datetime(raw_gyr(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');

acc_mag = vecnorm(raw_acc(:,3:5),2,2);
acc_mag = acc_mag - mean(acc_mag);

%% resample
T_acc = sortrows(timetable(seconds(raw_acc(:,2)/1e9),raw_acc(:,3:5),acc_mag));
T_gyr = sortrows(timetable(seconds(raw_gyr(:,2)/1e9),raw_gyr(:,3:5)));

TT = synchronize(T_acc,T_gyr,'regular','linear','TimeStep',seconds(2e-2));
% TT = synchronize(T_acc,T_gyr,'intersection','linear','SampleRate',50);
% TT = synchronize(T_acc,T_gyr,newTimes,'linear','SampleRate',2e-2);
Accelerometer = TT.Var1_T_acc;
Gyroscope = TT.Var1_T_gyr*180/pi;
acc_norm = TT.acc_mag;

time = seconds(TT.Time(:)-(TT.Time(1)));
rate = median(diff(time)); % cal sample rate

% acc = resample(acc_mag,time,20);
% [Accelerometer,time] = resample(acc_mag, raw_acc(:,2)/1e9,20);

%% find step point (step) and time labeling
% threshold should be tuned experimentally to match a person's level 
minPeakHeight = std(acc_norm);       

[pks,locs] = findpeaks(acc_norm,'MinPeakDistance',...
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


%% particle filter
data1 = readtable('batch.csv');
area_x = data1.x;
area_y = data1.y;

add_x = 23.2:.5:25;
add_y = 10.13*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 23.2:.5:25;
add_y = 10.63*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 23.2:.5:25;
add_y = 11.13*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 23.2:.5:25;
add_y = 11.63*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 32.8:.5:35.0;
add_y = 19.53*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 32.8:.5:35.0;
add_y = 19.03*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 32.8:.5:35.0;
add_y = 18.53*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 32.8:.5:35.0;
add_y = 18.03*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

add_x = 32.8:.5:35.0;
add_y = 17.53*ones(length(add_x),1);
area_x = [area_x;add_x'];
area_y = [area_y;add_y];

    
A = imread('N1-7F.png','BackgroundColor',[1 1 1]);
% A = imread('huawei_3rd-floor_gray_extended.png','BackgroundColor',[1 1 1]);


xWorldLimits = [-1 1650/20];
yWorldLimits = [-1 660/20];
RA = imref2d(size(A),xWorldLimits,yWorldLimits);
imshow(flipud(A),RA);
axis xy;

% draw learning data
hold on
plot(area_x,area_y,'.','MarkerSize', 10)
% for save eps
sdf(gcf,'sj2')

% initialize particle
n = 1000;
% 1. only road
% rand_idx = randi(length(data1.x),n,1);
% ps.x = data1.x(rand_idx);
% ps.y = data1.y(rand_idx);

% 3. initial area
ps.x = 24+random('normal',0,.5,[n,1]);
ps.y = 11+random('normal',0,.5,[n,1]);
ps.hist_x = ps.x;
ps.hist_y = ps.y;
ps.sl = .6+random('normal',0,.01,[n,1]);
ps.init_heading = random('Uniform', -pi,pi,[n,1]);
ps.prob = ones(n,1)*(1/n);

hold on 
h_ps = scatter(ps.x,ps.y,20,'g','filled','MarkerFaceAlpha',.2);
h_pm = plot(mean(ps.x),mean(ps.y),'ms');
legend('reference point','particle','mean')
hold off
hold off

%%
video_flag = 1;
if video_flag
%     v = VideoWriter('vids/pdr_particle_labeling_.mp4','MPEG-4');
    v = VideoWriter('vids/pf_labeling_sl_0.01.avi','Motion JPEG AVI');
    v.FrameRate = 10;
    v.Quality = 100;
    open(v);
    frame = getframe(gcf);
    writeVideo(v,frame);
end

estloc = zeros(length(locs),2);
% phi,theta,psi: roll pitch yaw (aerospace sequence?)
for i=1:length(locs)
    time_i = locs(i);
    % PREDICT
    yaw = euler(time_i,3);
    ps.hist_x(:,end+1) = ps.x;
    ps.hist_y(:,end+1) = ps.y;
    ps.x = ps.x + ps.sl.*cos(ps.init_heading+deg2rad(yaw));
    ps.y = ps.y + ps.sl.*sin(ps.init_heading+deg2rad(yaw));
    % UPDATE
    [physical_d,I] = pdist2([area_x,area_y],[ps.x,ps.y],'euclidean','Smallest',1);
    ps.prob = 1./physical_d;
    ps.prob(physical_d>.5) = 0;
    
    % RESAMPLE
    % When re-distribute particles
    if sum(ps.prob) == 0
        rand_idx = randi(length(area_x),n,1);
        ps.x = area_x(rand_idx);
        ps.y = area_y(rand_idx);
        ps.init_heading = random('Uniform', 0,2*pi,[n,1]);
        ps.prob = ones(n,1)*(1/n);
    else
        ps.prob = ps.prob./sum(ps.prob);
    end
    % Continues previous particles
    
    resample_idx = randsample(1:n,n,true,ps.prob);
    phy_move_noise_range = .01;
    ps.x = ps.x(resample_idx) + phy_move_noise_range*rand(n,1) - phy_move_noise_range/2;
    ps.y = ps.y(resample_idx) + phy_move_noise_range*rand(n,1) - phy_move_noise_range/2;
    ps.sl = ps.sl(resample_idx)+random('normal',0,.01,[n,1]);
    ps.hist_x = ps.hist_x(resample_idx,:);
    ps.hist_y = ps.hist_y(resample_idx,:);
    ps.init_heading = ps.init_heading(resample_idx)+random('normal',0,.1,[n,1]);
    set(h_ps,'XData',ps.x,'YData',ps.y)                             % ps result
    set(h_pm,'XData',mean(ps.x),'YData',mean(ps.y));    
    drawnow
    if video_flag
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end
if video_flag  
    close(v);
end
% return
%%
% close all
figure
imshow(flipud(A),RA);
axis xy;
for i=1:length(locs)
    hold on
    plot(ps.hist_x(i,:),ps.hist_y(i,:))
end

set(gcf,'units','points','position',[1000,200,1500,800])
sdf(gcf,'sj2')

print -clipboard -dbitmap
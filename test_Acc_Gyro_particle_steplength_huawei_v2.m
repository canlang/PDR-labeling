clear;close all;clc;

% addpath('input_rawdata')
% raw_acc = csvread('Accelerometer.csv',1,0);
% raw_gyr = csvread('Gyroscope.csv',1,0);
% 
% acc_time = datetime(raw_acc(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');
% gyr_time = datetime(raw_gyr(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');
% 
% acc_mag = vecnorm(raw_acc(:,3:5),2,2);
% acc_mag = acc_mag - mean(acc_mag);
rawdata = load_rawdata('input_rawdata');



%% resample
T_acc = timetable(seconds(raw_acc(:,2)/1e9),raw_acc(:,3:5),acc_mag);
T_gyr = timetable(seconds(raw_gyr(:,2)/1e9),raw_gyr(:,3:5));

% TT = synchronize(T_acc,T_gyr,'intersection','linear','SampleRate',50);
TT = synchronize(T_acc,T_gyr,'commonrange','linear','TimeStep',seconds(2e-2));
% TT = synchronize(T_acc,T_gyr,newTimes,'linear','SampleRate',2e-2);
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


%% load image map
A = imread('huawei_3rd-floor_gray_extended.png','BackgroundColor',[1 1 1]);
xWorldLimits = [0 4428*0.03625];
yWorldLimits = [0 1790*0.03625];
RA = imref2d(size(A),xWorldLimits,yWorldLimits);
imshow(flipud(A),RA);
axis xy;
% for save eps
set(gcf,'units','points','position',[200,200,1200,600])
sdf(gcf,'sj2')

% inbound & outbound
inbound = csvread('inbound.csv',1,0);
outbound = csvread('outbound.csv',1,0);
outbound2 = csvread('outbound2.csv',1,0);
outbound3 = csvread('outbound3.csv',1,0);

inbound = [(inbound(:,1))-(369361 - 4428 * 0.03625),inbound(:,2)-(3459867 - 1790 * 0.03625)];
outbound = [(outbound(:,1))-(369361 - 4428 * 0.03625),outbound(:,2)-(3459867 - 1790 * 0.03625)];
outbound2 = [(outbound2(:,1))-(369361 - 4428 * 0.03625),outbound2(:,2)-(3459867 - 1790 * 0.03625)];
outbound3 = [(outbound3(:,1))-(369361 - 4428 * 0.03625),outbound3(:,2)-(3459867 - 1790 * 0.03625)];

x = inbound(:,1);
y = inbound(:,2);
shp = polyshape(x,y);
ox = outbound(:,1);
oy = outbound(:,2);
shp = subtract(shp,polyshape(ox,oy));
ox = outbound2(:,1);
oy = outbound2(:,2);
shp = subtract(shp,polyshape(ox,oy));
ox = outbound3(:,1);
oy = outbound3(:,2);
shp = subtract(shp,polyshape(ox,oy));


% initialize particle
n = 1000;
% 1. only road
% rand_idx = randi(length(data1.x),n,1);
% ps.x = data1.x(rand_idx);
% ps.y = data1.y(rand_idx);

% 3. start points (tranportation area)
init_point = [mean([148.2,143.4]),50.26];
ps.x = init_point(1)+random('normal',0,.5,[n,1]);
ps.y = init_point(2)+random('normal',0,.5,[n,1]);
ps.hist_x = ps.x;
ps.hist_y = ps.y;
ps.sl = .6+random('normal',0,.01,[n,1]);
ps.init_heading = random('Uniform', -pi,pi,[n,1]);
ps.prob = ones(n,1)*(1/n);

hold on
plot(shp,'FaceAlpha',.5,'EdgeColor','r')
h_ps = scatter(ps.x,ps.y,20,'g','filled','MarkerFaceAlpha',.2);
h_pm = plot(mean(ps.x),mean(ps.y),'ms');
legend('reference point','particle','mean')
hold off

%%
video_flag = 0;
if video_flag
    v = VideoWriter('vids/pf_labeling_sl_0.01_h_0.1_huawei.mp4','MPEG-4');
%     v = VideoWriter('vids/pdr_particle_labeling_.mp4','MPEG-4');    
%     v = VideoWriter('vids/pf_labeling_sl_0.01_huawei.avi','Motion JPEG AVI');
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
%     [physical_d,I] = pdist2([area_x,area_y],[ps.x,ps.y],'euclidean','Smallest',1);
    in = isinterior(shp,ps.x,ps.y);
    ps.prob(in) = 1;
    ps.prob(~in) = 0;
    
    % RESAMPLE
    % When re-distribute particles
    if sum(ps.prob) == 0
        rand_idx = randi(n,n,1);
        ps.x = ps.x(rand_idx);
        ps.y = ps.y(rand_idx);
        ps.init_heading = random('Uniform', 0,2*pi,[n,1]);
        ps.prob = ones(n,1)*(1/n);
    else
        ps.prob = ps.prob./sum(ps.prob);
    end
    % Continues previous particles    
    resample_idx = randsample(1:n,n,true,ps.prob);
%     phy_move_noise_range = .1;
    phy_move_noise_range = .01;
    ps.x = ps.x(resample_idx) + phy_move_noise_range*rand(n,1) - phy_move_noise_range/2;
    ps.y = ps.y(resample_idx) + phy_move_noise_range*rand(n,1) - phy_move_noise_range/2;
    ps.sl = ps.sl(resample_idx)+random('normal',0.00,.01,[n,1]);
    ps.hist_x = ps.hist_x(resample_idx,:);
    ps.hist_y = ps.hist_y(resample_idx,:);
%     ps.init_heading = ps.init_heading(resample_idx)+random('normal',0,.01,[n,1]);
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
subplot(211)
imshow(flipud(A),RA);
axis xy;
for i=1:length(locs)
    hold on
    plot(ps.hist_x(i,:),ps.hist_y(i,:))
end
subplot(212)
imshow(flipud(A),RA);
axis xy;
end_point = [12.8,23.98];
k_idx = knnsearch([ps.hist_x(:,end),ps.hist_y(:,end)],end_point,'k',1);
hold on
plot(ps.hist_x(k_idx,:),ps.hist_y(k_idx,:));

% first time is initial time (not coundted at step detection)
trace_rs = [seconds([TT.Time(1);TT.Time(locs)]),...
    ps.hist_x(k_idx,:)',ps.hist_y(k_idx,:)'];
% csvwrite('trace_rs.csv',trace_rs);
dlmwrite('input_rawdata/trace_rs.csv',trace_rs,'precision', '%8.2f');

set(gcf,'units','points','position',[1000,200,1500,1200])
sdf(gcf,'sj2')
tightfig(gcf);

print -clipboard -dbitmap


function data = load_rawdata(datapath)
acc = csvread(fullfile(datapath,'Accelerometer.csv'),1,0);
gyr = csvread(fullfile(datapath,'Gyroscope.csv'),1,0);
data.acc = acc;
data.gyr = gyr;
end

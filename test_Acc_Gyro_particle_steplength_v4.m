clear;close all;clc;
%% load raw (IMU) data
target_rawdata_paths = getNameFolds('input_rawdata');
for j=1:length(target_rawdata_paths)
rawdata = load_rawdata(fullfile('input_rawdata',target_rawdata_paths{j}));
%% inbound & outbound
layout = loadjson('N1-7F-HiRes2.json');

x = layout.in(:,1);
y = layout.in(:,2);
shp = polyshape(x,y);
for i = 1:length(layout.out)
    ox = layout.out{i}(:,1);
    oy = layout.out{i}(:,2);
    shp = subtract(shp,polyshape(ox,oy));
end
%% reference (landmark)

% init = [mean([42.6,44.65]),mean([14.49,18.22]);mean([89.2,91.38]),mean([14.49,18.22])]; % for small N1 map
init = [mean([23.05,25.14]),mean([9.6,12.32]);mean([67.56,69.8]),mean([11.05,9.56])]; % for highres N1 map

% reference : end point info.
% end_point = [12.8,23.98];
% end_point = [74,20];
end_point = [mean([42.29,51.23]),mean([12.24,17.09])];


%% load image map
% pixelpermeter = 7.5;    % N1 7F_2.png
pixelpermeter = 20;    % N1 7F_2.png

[I,I_map,I_alpha] = imread('N1-7F.png','BackgroundColor',[1 1 1]);
A = imread('N1-7F.png','BackgroundColor',[1 1 1]);
% [I,I_map,I_alpha] = imread('N1-7F_3.png');
[I_h,I_w,I_as] = size(I);

% pixelpermeter = 27.5862;    % huawei_3rd-floor (0.03625)
% A = imread('huawei_3rd-floor_gray_extended.png','BackgroundColor',[1 1 1]);
% xWorldLimits = [0 I_w/pixelpermeter];
% yWorldLimits = [0 I_h/pixelpermeter];

xWorldLimits = [-1 1650/pixelpermeter]; % * N1 map not matcing the limit *
yWorldLimits = [-1 660/pixelpermeter];

RA = imref2d(size(I),xWorldLimits,yWorldLimits);
% imshow(flipud(I),RA,'InitialMagnification', 'fit','Colormap',I_map);
imshow(flipud(I),RA);
% h = imshow(flipud(I),RA,'Colormap',I_map);
axis xy;
% for save eps

set(gcf,'units','points','position',[200,200,1200,600])
sdf(gcf,'sj2')

%% resample for synchronize
rate = 2e-2;
processed_data = resample_rawdata(rawdata,rate);
%% find step point (step) and time labeling
% threshold should be tuned experimentally to match a person's level 
minPeakHeight = std(processed_data.acc_norm);       
[pks,locs] = findpeaks(processed_data.acc_norm,'MinPeakDistance',...
    .3/rate,'MinPeakHeight',minPeakHeight);   % .3s 이내의 피크는 무시 (가정: 발걸음이 .3초 내에는 2번이뤄지지 않음)
%%
addpath(genpath('madgwick_algorithm_matlab'));
AHRS = MadgwickAHRS('SamplePeriod', rate, 'Beta', 0.1); % sample rate: 2e-2

quaternion = zeros(length(processed_data.Time), 4);
for t = 1:length(processed_data.Time)
    AHRS.Update(processed_data.Gyroscope(t,:) * (pi/180), ...
        processed_data.Accelerometer(t,:), ...
        processed_data.Accelerometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
std_euler = stdfilt(deg2rad(unwrap(euler(:,3))));
% plot(processed_data.Time, std_euler)
%%
% initialize particle
n = 1000;
% 1. only road
% rand_idx = randi(length(data1.x),n,1);
% ps.x = data1.x(rand_idx);
% ps.y = data1.y(rand_idx);

% 3. start points (tranportation area)
% init_point = [mean([148.2,143.4]),50.26];
% ps.x = init_point(1)+random('normal',0,.5,[n,1]);
% ps.y = init_point(2)+random('normal',0,.5,[n,1]);

% depends on number of init (matrix's row)
for i=1:size(init,1)
    % [ps.x, ps.y] = deal(init + [random('normal',0,.5,[n,1]),random('normal',0,.5,[n,1])]);
    m = fix(n/size(init,1));
    if ~exist('ps','var')
        ps.x = init(i,1)+random('normal',0,.5,[m,1]);
        ps.y = init(i,2)+random('normal',0,.5,[m,1]);        
    else
        if i == size(init,1)
            m = m+mod(n,size(init,1));
        end
        ps.x = [ps.x;init(i,1)+random('normal',0,.5,[m,1])];
        ps.y = [ps.y;init(i,2)+random('normal',0,.5,[m,1])]; 
    end    
end
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
    v = VideoWriter('vids/pf_labeling_n1_multiple_init_area.mp4','MPEG-4');
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
%     ps.init_heading = ps.init_heading(resample_idx)+random('normal',0,std_euler(time_i),[n,1]);
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
imshow(flipud(I),RA);
axis xy;
for i=1:length(locs)
    hold on
    plot(ps.hist_x(i,:),ps.hist_y(i,:))
end
subplot(212)
imshow(flipud(I),RA);
axis xy;


% find best path
[k_idx,k_d] = knnsearch([ps.hist_x(:,end),ps.hist_y(:,end)],end_point,'k',1);
sorted_k = sortrows([k_idx,k_d],2);
sorted_k(1)
% visusalize result
hold on
plot(ps.hist_x(k_idx,:),ps.hist_y(k_idx,:));
text(1,3,['Weight = ',num2str(1/min(k_d))]);
hold off

% first time is initial time and it must be added manually,
% because it's not coundted at step detection
trace_rs = [[processed_data.Time(1);processed_data.Time(locs)],...
    ps.hist_x(k_idx,:)',ps.hist_y(k_idx,:)'];
% file write
dlmwrite(fullfile('input_rawdata',strcat(target_rawdata_paths{j},'.csv')),trace_rs,'precision', '%8.2f');

set(gcf,'units','points','position',[1000,200,1500,1200])
sdf(gcf,'sj2')
tightfig(gcf);

print -clipboard -dbitmap
% save result image
print(fullfile('input_rawdata',target_rawdata_paths{j}),'-dpng')
% ,'-r300')
close all,clearvars -except target_rawdata_paths
end

function data = resample_rawdata(rawdata,rate)
% TT = synchronize(T_acc,T_gyr,'intersection','linear','SampleRate',50);
% TT = synchronize(T_acc,T_gyr,newTimes,'linear','SampleRate',2e-2);

% Var1 : 3vector of Accelerometer, Var2: norm vector of Acc.
T_acc = timetable(seconds(rawdata.acc(:,2)/1e9),rawdata.acc(:,3:5),rawdata.acc_norm,...
    'VariableNames',{'acc','acc_norm'});
% Var1 : 3vector of Gyroscope
T_gyr = timetable(seconds(rawdata.gyr(:,2)/1e9),rawdata.gyr(:,3:5),...
    'VariableNames',{'gyr'});
T_acc = sortrows(T_acc);
T_gyr = sortrows(T_gyr);

% TT = synchronize(T_acc,T_gyr,'commonrange','linear','TimeStep',seconds(rate));
TT = synchronize(T_acc,T_gyr,'regular','linear','TimeStep',seconds(rate));

data.Accelerometer = TT.acc;
data.Gyroscope = TT.gyr*180/pi;
data.acc_norm = TT.acc_norm;

data.Time = seconds(TT.Time(:));
% data.Time = seconds(TT.Time(:)-(TT.Time(1)));
data.Rate = median(diff(data.Time)); % cal sample rate
end
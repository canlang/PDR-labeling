clear;close all;clc;
%% load raw data (from IMU)
target_rawdata_paths = getNameFolds('input_rawdata');
%%
for j=1:length(target_rawdata_paths)
rawdata = load_rawdata(fullfile('input_rawdata',target_rawdata_paths{j}));
trace_info = regexp(target_rawdata_paths{j},'_','split');

%% layout info. inbound & outbound
layout = loadjson(strcat([trace_info{3},'.json']));

x = layout.in(:,1);
y = layout.in(:,2);
shp = polyshape(x,y);
for i = 1:length(layout.out)
    ox = layout.out{i}(:,1);
    oy = layout.out{i}(:,2);
    shp = subtract(shp,polyshape(ox,oy));
end

%% load image map
pixelpermeter = layout.ppm; 
if isfile(strcat([trace_info{3},'.png']))
%     [I,I_map,I_alpha] = imread(strcat([trace_info{3},'.png']),'BackgroundColor',[1 1 1]);
    I = imread(strcat([trace_info{3},'.png']),'BackgroundColor',[1 1 1]);
else
    [I,I_map,I_alpha] = imread(strcat([trace_info{3},'.jpg']));
end
[I_h,I_w,I_as] = size(I);
xWorldLimits = [0 I_w/pixelpermeter];
yWorldLimits = [0 I_h/pixelpermeter];
RA = imref2d(size(I),xWorldLimits,yWorldLimits);
% imshow(flipud(I),RA);
imshow(flipud(I),RA,'InitialMagnification', 'fit');
% h = imshow(flipud(I),RA,'Colormap',I_map);
axis xy;
set(gcf,'units','points','position',[200,200,1200,600])
sdf(gcf,'sj2')

%% resample for synchronize
rate = 2e-2;
processed_data = resample_rawdata(rawdata,rate);
%% find step point (step) and time labeling
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

%%
n = 1000;
% start points (tranportation area)
if strcmp(trace_info{1},'EV')
    init = layout.ev;
elseif strcmp(trace_info{1},'ESCAL')
    init = layout.escal;
else
    init = layout.stair;
end
    
for i=1:size(init,1)
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
%     ,'Motion JPEG AVI');
    v.FrameRate = 10;
    v.Quality = 100;
    open(v);
    frame = getframe(gcf);
    writeVideo(v,frame);
end

estloc = zeros(length(locs),2);
for i=1:length(locs)
    time_i = locs(i);
    % transition prop
    yaw = euler(time_i,3);
    ps.hist_x(:,end+1) = ps.x;
    ps.hist_y(:,end+1) = ps.y;
    ps.x = ps.x + ps.sl.*cos(ps.init_heading+deg2rad(yaw));
    ps.y = ps.y + ps.sl.*sin(ps.init_heading+deg2rad(yaw));
    % update prop
    in = isinterior(shp,ps.x,ps.y);
    ps.prob(in) = 1;
    ps.prob(~in) = 0;
    ps.prob(ps.sl<.4) = 0;
    
    % resampling
    if sum(ps.prob) == 0
        rand_idx = randi(n,n,1);
        ps.x = ps.x(rand_idx);
        ps.y = ps.y(rand_idx);
        ps.init_heading = random('Uniform', 0,2*pi,[n,1]);
        ps.prob = ones(n,1)*(1/n);
    else
        ps.prob = ps.prob./sum(ps.prob);
    end  
    resample_idx = randsample(1:n,n,true,ps.prob);
    phy_move_noise_range = .01;
    ps.x = ps.x(resample_idx) + phy_move_noise_range*rand(n,1) - phy_move_noise_range/2;
    ps.y = ps.y(resample_idx) + phy_move_noise_range*rand(n,1) - phy_move_noise_range/2;
    ps.sl = ps.sl(resample_idx)+random('normal',0.00,.01,[n,1]);
    ps.hist_x = ps.hist_x(resample_idx,:);
    ps.hist_y = ps.hist_y(resample_idx,:);
    ps.init_heading = ps.init_heading(resample_idx)+random('normal',0,.1,[n,1]);
    
    % draw updated particles
    set(h_ps,'XData',ps.x,'YData',ps.y);
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
figure
subplot(211)
imshow(flipud(I),RA);
axis xy;
for i=1:length(locs)
    hold on
    plot(ps.hist_x(i,:),ps.hist_y(i,:))
end
% subplot(212)
% imshow(flipud(I),RA);
% axis xy;

% reference : end point info.
if strcmp(trace_info{2},'EV')
    end_point = layout.ev;
elseif strcmp(trace_info{2},'ESCAL')
    end_point = layout.escal;
else
    end_point = layout.stair;
end

% find best fit path
[k_idx,k_d] = knnsearch([ps.hist_x(:,end),ps.hist_y(:,end)],end_point,'k',1);
sorted_k = sortrows([k_idx,k_d],2);
% sorted_k(1) % nextline changed
fprintf ( 1, 'found best index : %d\n',sorted_k(1));

% visusalize result
hold on
est_p = plot(ps.hist_x(sorted_k(1),:),ps.hist_y(sorted_k(1),:),'-.r*');
text(1,3,['Penalty Score = ',num2str(min(k_d))]);
hold off

legend(est_p,'Estimated path (Best fit)')

% first time is initial time and it must be added manually,
% because it's not coundted at step detection
trace_rs = [[processed_data.Time(1);processed_data.Time(locs)],...
    ps.hist_x(sorted_k(1),:)',ps.hist_y(sorted_k(1),:)'];
% file write
dlmwrite(fullfile('input_rawdata',...
    strcat([target_rawdata_paths{j},'_',num2str(min(k_d),'%.1f'),'.csv'])),...
    trace_rs,'precision', '%8.2f');

set(gcf,'units','points','position',[1000,200,1500,1200])
sdf(gcf,'sj2')
tightfig(gcf);

% print -clipboard -dbitmap
% print(fullfile('input_rawdata',target_rawdata_paths{j}),'-dpng')
print(fullfile('input_rawdata',...
    strcat([target_rawdata_paths{j},'_',num2str(min(k_d),'%.1f'),'.png'])),'-dpng')
% ,'-r300')

% -- CLEAR ALL VARIABLES FOR ITERATIVE LOOP
close all,clearvars -except target_rawdata_paths
end

%% local function -----------------------------------------------------------------
function nameFolds = getNameFolds(pathFolder)
d = dir(pathFolder);
isub = [d(:).isdir]; %# returns logical vector
nameFolds = {d(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];
% nameFolds(~cellfun(@isempty,regexp(nameFolds, '\d{6}'))) = [];
end
function data = resample_rawdata(rawdata,rate)
% Var1 : 3 vectors of Accelerometer, Var2: norm vector of Acc.
T_acc = timetable(seconds(rawdata.acc(:,2)/1e9),rawdata.acc(:,3:5),rawdata.acc_norm,...
    'VariableNames',{'acc','acc_norm'});
% Var1 : 3 vectors of Gyroscope
T_gyr = timetable(seconds(rawdata.gyr(:,2)/1e9),rawdata.gyr(:,3:5),...
    'VariableNames',{'gyr'});
T_acc = sortrows(T_acc);
T_gyr = sortrows(T_gyr);

TT = synchronize(T_acc,T_gyr,'regular','linear','TimeStep',seconds(rate));

data.Accelerometer = TT.acc;
data.Gyroscope = TT.gyr*180/pi;
data.acc_norm = TT.acc_norm;

data.Time = seconds(TT.Time(:));
% data.Time = seconds(TT.Time(:)-(TT.Time(1)));
data.Rate = median(diff(data.Time)); % cal sample rate
end
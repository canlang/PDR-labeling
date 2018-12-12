clear all, close all;

filename = 'step_angle_location.txt';
roadfilename = 'n1_roadnetwork.txt';

delimiterIn = ',';
A = importdata(filename, delimiterIn);
R = importdata(roadfilename,delimiterIn);

% WIFI ESTIMATION LOCATION LOAD
wifiLoc = A.data(ismember(A.textdata,'W'),:);

% MAP-IMAGE DRAW
[imdata, map, alpha] = imread('7f.png');
[h,w,p] = size(imdata);
fig = figure('position',[100,100, w+200,h+200]);
image(imdata)
axis equal

% ROAD-NETWORK DRAW
rowSize = size(R,1);
for i = 1:rowSize
    line(R(i,[1,3])/0.1219, 370-R(i,[2,4])/0.1219,...
        'linewidth',4,'color',[.8,.8,.8]);
end
inter = unique([R(:,1),R(:,2);R(:,3),R(:,4)],'rows');
%%
% hold on 
% plot(wifiLoc(:,2)/0.1219, 370-wifiLoc(:,3)/0.1219, 'x')
% hold off

% circle_n1(wifiLoc(1,2), wifiLoc(1,3),20,-90);
locHis  = zeros(657,3);
loc     = A.data(1,:);
theta   = -90;
step    = 0.6;
turn    = 0;

locHis(1,:) = loc;
% locHis = [];

rowSize = size(A.data,1);
logging = false;
for i = 2:60
    switch A.textdata{i,1}
        case 'W'
            if logging
                disp 'WiFi'
            end
            
        case 'A'
            if logging
                disp 'Angle update'
            end
            theta = -90 + A.data(i,2);
            
%             locHis(i,:) = locHis(i-1,:);
            
            if i<6
                startIdx = 1;
            else
                startIdx = i-5;
            end
            if all(ismember([A.textdata{startIdx:i-1,1}], 'A') == 0)
                disp 'heading start'
                turn = i;
            end
        case 'S'
            if logging
                disp 'Step count'
            end
            loc(1) = A.data(i,1);
            loc(2) = loc(2) + step*(0*cosd(theta) - 1*sind(theta));
            loc(3) = loc(3) + step*(0*sind(theta) + 1*cosd(theta));
            circle_n1(loc(2),loc(3),5, theta,'m');
            locHis(i,:) = loc;
%             locHis(end+1,:) = [loc(1),loc(2)];
            
            if i<6
                startIdx = 1;
            else
                startIdx = i-4;
            end
            if all(ismember([A.textdata{startIdx:i,1}], 'A') == 0) && turn
                disp 'heading end'
                window = A.data(turn:i,2);      % turn start index & end index
                hwindow = window(ismember([A.textdata{turn:i}], 'A'));
                if abs(hwindow(1)-hwindow(end)) > 80
                    disp 'TURN DETECT!'
                    turnIdx = round((turn+(i-5))/2);
                    turnTime = A.data(turnIdx,1)-800;
                    [turnLoc, ~] = getStepLoc(locHis, turnTime);
                    
                    [IDX,D] = knnsearch(inter,turnLoc(2:3),'k',1);
                    NN_1 = inter(IDX,:);
                    
                    f = @(x)turnPointErr(x,locHis,turnTime,NN_1);
                    optScale = fminunc(f,1);
                    
%                     getStepLength(locHis(turnIdx-19:turnIdx,:))
                    currTime = A.data(i,1);
                    [newHis, newLoc] = adjustStepLength(...
                        turnTime-10000, currTime, locHis, optScale);
                    
                    step = step*optScale;
                    loc(2:3) = newLoc(end,:);
                    hold on
                    plot(newLoc(:,1)/.1219,370-newLoc(:,2)/.1219,'ob',...
                        'linewidth',1.3,'MarkerSize',12);                    
%                     plot(newHis(:,2)/.1219,370-newHis(:,3)/.1219,'+g',...
%                         'linewidth',1,'MarkerSize',10);                    
                    plot(NN_1(1)/.1219,370-NN_1(2)/.1219,'xr',...
                        'linewidth',1.3,'MarkerSize',12);
                    hold off
                    break
                end
                turn = 0;
            end
    end
end

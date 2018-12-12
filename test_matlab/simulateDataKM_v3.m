clear all, close all;

filename = 'step_angle_location.txt';
roadfilename = 'n1_roadnetwork.txt';
delimiterIn = ',';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SMARTPHONE DATA LOAD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = importdata(filename, delimiterIn);
R = importdata(roadfilename,delimiterIn);

%wifiLoc    : WIFI LOCATION
wifiLoc = A.data(ismember(A.textdata,'W'),:);

% inter     : INTERSECTION POINT
inter = unique([R(:,1),R(:,2);R(:,3),R(:,4)],'rows');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WIFI LOCATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hold on 
% plot(wifiLoc(:,2)/0.1219, 370-wifiLoc(:,3)/0.1219, 'x')
% hold off
% circle_n1(wifiLoc(1,2), wifiLoc(1,3),20,-90);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITIALIZE THE CURRENT STATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
loc     = A.data(1,:);
theta   = -90;
step    = 0.5;
turn    = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCATION HISTORY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
locHis  = zeros(657,3);
locHis(1,:) = loc;

f = figure('visible','on');
clf(f);
set(f,'position',[100,100, 1000,1100]);

rowSize = size(A.data,1);
logging = false;
k = 1;
for i = 2:600
    if A.textdata{i,1} == 'S'
        % MAP-IMAGE DRAW
        [imdata, map, alpha] = imread('7f.png');
%         [h,w,p] = size(imdata);

        sh(1) = subplot(3,1,1);
        image(imdata)
        axis equal
        grid on

        % ROAD-NETWORK DRAW
        rowSize = size(R,1);
        for j = 1:rowSize
            line(R(j,[1,3])/0.1219, 370-R(j,[2,4])/0.1219,...
                'linewidth',4,'color',[.8,.8,.8]);
        end
    end
    switch A.textdata{i,1}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% WIFI DATA INPUT
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'W'
            if logging
                disp 'WiFi'
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% HEADING DATA INPUT
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        case 'A'
            if logging
                disp 'Heading update'
            end
            %%% HEADING UPDATE
            theta = -90 + A.data(i,2);

            %%% FOR TURNING POINT DETECTION
            if i<6
                startIdx = 1;
            else
                startIdx = i-5;
            end
            if all(ismember([A.textdata{startIdx:i-1,1}], 'A') == 0)
                disp 'heading start'
                turn = i;
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% STEP DATA INPUT
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'S'
            if logging
                disp 'Step count'
            end
            loc(1) = A.data(i,1);
            loc(2) = loc(2) + step*(0*cosd(theta) - 1*sind(theta));
            loc(3) = loc(3) + step*(0*sind(theta) + 1*cosd(theta));
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% MM
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [wayIdx,~] = my1nnIntersection(R, loc(2:3));
            ploc = project_point_to_line_segment(R(wayIdx,[1,2]),R(wayIdx,[3,4]),loc([2,3]));
%             loc([2,3]) = ploc;
            circle_n1(ploc(1),ploc(2),10, theta,'r');
            
%             circle_n1(loc(2),loc(3),10, theta,'g');
            locHis(i,:) = loc;
            
            
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
                    turnTime = A.data(turnIdx,1)-500;
                    [turnLoc, ~] = getStepLoc(locHis, turnTime);
                    
                    [IDX,D] = knnsearch(inter,turnLoc(2:3),'k',1);
                    NN_1 = inter(IDX,:);
                    
                    %%% FIND OPTIMAL : STEP LENGTH & heading..?
                    errfun = @(x)turnPointErr3(x,locHis,turnTime,12000,NN_1);
%                     options = optimoptions(@fminunc,'GradObj','on');
%                     optVals = fminunc(errfun,[1,0],options);

                    optVals = fminunc(errfun,[1,0]);
                    optScale = optVals(1);
                    optHeading = optVals(2);
                    
                    %% optimization dimensino 2D plotting
                    [x,y] = meshgrid(0:.1:2, -15:.5:15);
                    errfun = @(x,y)turnPointErr2(x,y,locHis,turnTime,12000,NN_1);
                    z = arrayfun(errfun,x,y);
                    subplot(3,1,3)
                    imagesc(x(1,:),y(:,1),z);
                    xlabel('step length scale')
                    ylabel('heading angle')
                    
                    
%                     nb = mupad;
%                     setVar(nb,errfun)
                    
                    %%% update heading from opti
%                     theta = theta + optHeading/2;
                    
%                     getStepLength(locHis(turnIdx-19:turnIdx,:))
                    currTime = A.data(i,1);
                    [newHis, newLoc] = adjustStepLength(...
                        turnTime-12000, currTime, locHis, optScale);
                    
                    locHis = newHis;
                    step = step*optScale;
                    
                    loc(2:3) = newLoc(end,:);
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%% MM
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     [wayIdx,~] = my1nnIntersection(R, loc(2:3));
%                     ploc = project_point_to_line_segment(R(wayIdx,[1,2]),R(wayIdx,[3,4]),loc([2,3]));
%                     loc([2,3]) = ploc;
%                     circle_n1(ploc(1),ploc(2),10, theta,'r');
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    subplot(3,1,1)
                    hold on
                    plot(newLoc(:,1)/.1219,370-newLoc(:,2)/.1219,'ob',...
                        'linewidth',1.3,'MarkerSize',12);                    
%                     plot(newHis(:,2)/.1219,370-newHis(:,3)/.1219,'+g',...
%                         'linewidth',1,'MarkerSize',10);                    
                    plot(NN_1(1)/.1219,370-NN_1(2)/.1219,'xr',...
                        'linewidth',1.3,'MarkerSize',12);
                    hold off
                end
                turn = 0;
            end
    end
    if A.textdata{i,1} == 'S'
        subplot(3,1,2)
        axis equal
        grid on
        
        hold on
        plot(loc(2),loc(3), 'x','MarkerSize',11,'LineWidth',1.2)
        hold off
        
        M(k) = getframe(gcf);
        k = k+1;
    end
end
%%
% close all
% [h,w,p] = size(M(1).cdata);
% hf = figure;
% set(hf,'position',[100,100,w,h]);
% axis off
% movie(hf,M,1);

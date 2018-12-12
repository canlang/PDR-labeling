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

plot(wifiLoc(:,2), wifiLoc(:,3),'x')
axis equal

dist = zeros(size(R,1),1);
for i = 1:size(R,1)
    dist(i) = seglineToPoint(R(i,:), wifiLoc(10,2:3));
end
hold on
widx = 20;
plot(wifiLoc(widx,2),wifiLoc(widx,3), 'rx')
[idx, d] = my1nnIntersection(R,wifiLoc(widx,2:3));
line(R(idx,[1,3]), R(idx,[2,4]),...
                'linewidth',4,'color',[.8,.8,.8]);
            
projP = project_point_to_line_segment(R(idx,[1,2]),R(idx,[3,4]),wifiLoc(widx,[2 3]));
plot(projP(1),projP(2),'o');
%%
% v = arrayfun(@(x,y) adjustRotation(0,0,x,y,30), wifiLoc(:,1), wifiLoc(:,2),'UniformOutput',false);
target = arrayfun(@(x,y) adjustRotation(40,20,x,y,-180), wifiLoc(:,2), wifiLoc(:,3));
hold on
plot(x1,y1,'bx')
hold off
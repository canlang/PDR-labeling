clear all;close all;
imu_filename = 'step_angle_location.txt';
roadfilename = 'n1_roadnetwork.txt';
A = importdata(imu_filename,',');
R = importdata(roadfilename,',');

inter = unique([R(:,1),R(:,2);R(:,3),R(:,4)],'rows');
[imdata, map, alpha] = imread('7f.png');


image(imdata)
axis equal
grid on

% ROAD-NETWORK DRAW
rowSize = size(R,1);
for j = 1:rowSize
    line(R(j,[1,3])/0.1219, 370-R(j,[2,4])/0.1219,...
        'linewidth',4,'color',[.8,.8,.8]);
end
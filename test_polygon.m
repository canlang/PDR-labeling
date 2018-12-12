clear;close all;clc;
inbound = csvread('inbound.csv',1,0);
outbound = csvread('outbound.csv',1,0);
outbound2 = csvread('outbound2.csv',1,0);
outbound3 = csvread('outbound3.csv',1,0);

inbound = [(inbound(:,1))-(369361 - 4428 * 0.03625),inbound(:,2)-(3459867 - 1790 * 0.03625)];
outbound = [(outbound(:,1))-(369361 - 4428 * 0.03625),outbound(:,2)-(3459867 - 1790 * 0.03625)];
outbound2 = [(outbound2(:,1))-(369361 - 4428 * 0.03625),outbound2(:,2)-(3459867 - 1790 * 0.03625)];
outbound3 = [(outbound3(:,1))-(369361 - 4428 * 0.03625),outbound3(:,2)-(3459867 - 1790 * 0.03625)];


A = imread('huawei_3rd-floor_gray_extended.png','BackgroundColor',[1 1 1]);
xWorldLimits = [0 4428*0.03625];
yWorldLimits = [0 1790*0.03625];
RA = imref2d(size(A),xWorldLimits,yWorldLimits);
imshow(flipud(A),RA);
axis xy;
%%

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

% shp = alphaShape(x,y);
% shp = polyshape({x,ox},{y,oy});
hold on
plot(shp)
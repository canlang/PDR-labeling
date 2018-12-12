clear; close all;
addpath('xyz file operations')
addpath(genpath('pathfinding'))

%%
% A = imread('N1-7F.png','BackgroundColor',[1 1 1]);
% xWorldLimits = [-1 1650/20];
% yWorldLimits = [-1 660/20];
% RA = imref2d(size(A),xWorldLimits,yWorldLimits);
% imshow(flipud(A),RA);
% axis image
% axis xy

%% CREATE GRID AREA
data1 = readtable('batch.csv');

x = data1.x;
y = data1.y;

XI = min(x):.5:max(x);
YI = min(y):.5:max(y);
[X,Y] = meshgrid(XI,YI);

shp = alphaShape(x,y);
% shp.Alpha = 2;            % Alpha parameter: 
in = inShape(shp,X,Y);
xg = X(in);
yg = Y(in);
zg = ones(length(xg),1);
[X,Y,Z] = xyz2grid(xg,yg,zg);

Z(isnan(Z))=0;

% hold on
% plot(xg,yg,'x')

%% CREATE GRAPH
pad = 5;
map = zeros(size(Z,1)+pad,size(Z,2)+pad);
hai = fix(pad/2)+1;
map(hai:end-hai,hai:end-hai) = Z;
g=Graph(map); %Generating a graph from a mobility map.
%% 
initv=1;
shortpathv=initv;
finalv=initv;
[xmax ymax]=size(map);
for i=1:100
    last=size(shortpathv,2);
    initv=shortpathv(last); 
    ShowPath(g,map,shortpathv);    
    axis image
    axis xy
    axis on
    set(gcf,'units','points','position',[200,500,1000,400])
    sdf(gcf,'sj2')
    [x y]=ginput(1);
    x=round(x);
    y=round(y);
    if x>0&&x<=xmax&&y>0&&y<=ymax
        finalv=g.node_index_map(x,y);
        if finalv>0
            [shortpathv g]=GetPath(g,initv,finalv);
            disp('initv')
            disp(initv)
            disp('finalv')
            disp(finalv)
            disp('Total distance from init_node to final_node')
            disp(g.node_list(initv).wmin)
        end
    end       
end
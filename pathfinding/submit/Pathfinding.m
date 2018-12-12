%% Pathfinding
% An example how to use class Graph.
% Usage: 
%   g=Graph(map);
%   Constructor, where map is 2D matrix of mobility map.
%
%   ShowPath(g,map,shortpathv); 
%   Displaying where shortpathv is list of nodes on shortest path.
%
%   [shortpathv g]=GetPath(g,initv,finalv); Computing the shortest path
%   between init_node and final_node.

clear;
map=imread('test5.png');
map=double(map(:,:,1))/255; % normalizing to 1
%map=[1 1 1; 0 0 0; 1 1 1]
g=Graph(map); %Generating a graph from a mobility map.
initv=1;
shortpathv=initv;
finalv=initv;
[xmax ymax]=size(map);
for i=1:100
    last=size(shortpathv,2);
    initv=shortpathv(last); 
    ShowPath(g,map,shortpathv);    
    [x y]=ginput(1);
    x=round(x);Description
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
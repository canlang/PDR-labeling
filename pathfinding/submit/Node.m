%% Node
classdef Node
    properties
        posx %position x
        posy %position y
        visited %is the node is visited
        wmin %Shortest distance from the init node to this node 
        vmin %The parent node of the shortest path
        n %number of neight bour of this node
        edge_list %edge list
    end
    methods
        function v=Node(posx,posy,visited,wmin,vmin,n,edge_list)
            v.posx=posx;
            v.posy=posy;
            v.visited=visited;
            v.wmin=wmin;
            v.vmin=vmin;
            v.n=n;
            v.edge_list=edge_list;
        end
    end
end
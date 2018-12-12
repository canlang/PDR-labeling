%% Graph class difinition
% The class has a constructor g=Graph(map), whre map is mobility 2D matrix
% and following methos.
%
% function g=Reset(g): Reset properties in a node g.
%
% function [shortpathv g]=GetPath(g,init_node,final_node):getting shortest
% path from final_node to init node.
%
% function g=ComputeDistance(g,final_node): Computing sum of distance from
% final node to every node in the graph.
%
% function g=Visit(g,Queue): A recursive funtion in order to visit every
% node once.
%
% function [g Q]=UpdateWmin(g,parent_index):
%
% function shortpathxy=ShowPath(g,map,shortpathv)
%
%Author: Wasit Limprasert Email:wasit7(at)gmail.com Date:06Feb2012
classdef Graph
    properties        
        node_list %list of nodes in the graph
        node_index_map %mobility map matrix
        n
    end
    methods
        %% function g=Graph(map)
        % generating graph from a mobility map.
        function g=Graph(map)
            [xmax ymax]=size(map);
            node_list_index=0;
            mapv=zeros(xmax,ymax);            
            for x=1:xmax
                for y=1:ymax
                    if map(x,y)
                        node_list_index=node_list_index+1;
                        mapv(x,y)=node_list_index;
                    end
                end
            end
            g.node_index_map=mapv;
            for x=1:xmax
                for y=1:ymax
                    parent=map(x,y);%mobility of the parent node.
                    if parent            
                        %e1 e2 e3
                        %e4 v  e5
                        %e6 e7 e8
                        edge_list=[];
                        e=[];
                        if(x-1>=1&&y-1>=1)%e1
                            %mobility of the child node
                            childv=mapv(x-1,y-1);
                            child=map(x-1,y-1);
                            if child
                                e=Edge(childv,2.828/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end                        
                        if(x-1>=1)%e2
                            childv=mapv(x-1,y);
                            child=map(x-1,y);
                            if child
                                e=Edge(childv,2/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        if(x-1>=1&&y+1<=ymax)%e3
                            childv=mapv(x-1,y+1);
                            child=map(x-1,y+1);
                            if child
                                e=Edge(childv,2.828/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        if(y-1>=1)%e4
                            childv=mapv(x,y-1);
                            child=map(x,y-1);
                            if child
                                e=Edge(childv,2/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        if(y+1<=ymax)%e5
                            childv=mapv(x,y+1);
                            child=map(x,y+1);
                            if child
                                e=Edge(childv,2/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        if(x+1<=xmax&&y-1>=1)%e6
                            childv=mapv(x+1,y-1);
                            child=map(x+1,y-1);
                            if child
                                e=Edge(childv,2.828/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        if(x+1<=xmax)%e7
                            childv=mapv(x+1,y);
                            child=map(x+1,y);
                            if child
                                e=Edge(childv,2/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        if(x+1<=xmax&&y+1<=ymax)%e8
                            childv=mapv(x+1,y+1);
                            child=map(x+1,y+1);
                            if child
                                e=Edge(childv,2.828/(child+parent));
                                edge_list=[edge_list; e];
                            end
                        end
                        n=size(edge_list,1);
                        wmin=Inf;
                        vmin=0;
                        visited=0;
                        posx=x;
                        posy=y;
                        v=Node(posx,posy,visited,wmin,vmin,n,edge_list);
                        g.node_list=[g.node_list; v];                         
                    end
                end                
            end
            g.n=size(g.node_list,1);
        end
        %% function g=Reset(g)
        % initializing node parameters.
        function g=Reset(g)
            for i=1:size(g.node_list)
                g.node_list(i).visited=0;
                g.node_list(i).wmin=Inf;
                g.node_list(i).vmin=0;
            end
        end
        %% function [shortpathv g]=GetPath(g,init_node,final_node)
        % Displaying nodes index on shortest path after visiting all nodes.
        function [shortpathv g]=GetPath(g,init_node,final_node)
           g=Reset(g);
           g=ComputeDistance(g,final_node);
           shortpathv=[init_node];
           if final_node>0               
               if g.node_list(init_node).vmin
                   v=init_node;
                   while v~=final_node
                        v=g.node_list(v).vmin;
                        shortpathv=[shortpathv v];
                   end
                   disp('--->Found.')
                   shortpathv=[shortpathv final_node]
               else
                   disp('--->Could not found.')
                   shortpathv=[init_node]
               end
           end
        end
        %% function g=ComputeDistance(g,final_node) 
        % Computing shortest distance from final_node to all nodes in the
        % graph.
        function g=ComputeDistance(g,final_node)            
            g.node_list(final_node).wmin=0;
            g.node_list(final_node).vmin=final_node;
            g=Visit(g,final_node);
        end
        %% function g=Visit(g,Queue)
        % Visiting parent node in the list Queue. This is a recursive
        % function all nodes will be visited once and distacnce from a
        % particular node to final node will be computed recursively.
        function g=Visit(g,Queue)
            imax=size(Queue,1);
            Queue_next=[];
            for i=1:imax
                if ~g.node_list(Queue(i)).visited
                    [g Q]=UpdateWmin(g,Queue(i));
                    Queue_next=[Queue_next; Q];
                    g.node_list(Queue(i)).visited=1;
                end
            end
            if ~isempty(Queue_next)
                g=Visit(g,Queue_next);
            end
        end
        %% function [g Q]=UpdateWmin(g,parent_index)
        % Comparing and updating Wmin and Vmin in order to find shortest
        % path.
        function [g Q]=UpdateWmin(g,parent_index)
            %g graph
            parent=g.node_list(parent_index);
            num_child=parent.n; %number of edge of the child
            Q=[];
            for e=1:num_child
                edge=g.node_list(parent_index).edge_list(e); %edge from a parent node to a child node
                child=g.node_list(edge.node_index); %checking node
                neww=parent.wmin+edge.w;
                if neww<child.wmin
                    g.node_list(edge.node_index).wmin=neww;
                    g.node_list(edge.node_index).vmin=parent_index;
                end                
                if ~g.node_list(edge.node_index).visited
                    Q=[Q; edge.node_index];
                end
            end
        end
        %% function shortpathxy=ShowPath(g,map,shortpathv)
        % Displaying
        function shortpathxy=ShowPath(g,map,shortpathv)
            delete(gca);
            [xmax ymax]=size(map);
            
            [x y]=meshgrid([0.5:xmax+0.5],[0.5:ymax+0.5]);
            surface(x',y',zeros(xmax+1,ymax+1),[map zeros(xmax,1);zeros(1,ymax+1)],'EdgeColor','none');
            colormap(gray(255));
            hold on
            shortpathxy=[];
            lastv=size(shortpathv,2);
            for i=1:lastv
               shortpathxy=[shortpathxy [g.node_list(shortpathv(i)).posx; g.node_list(shortpathv(i)).posy]];
            end
            plot(shortpathxy(1,:),shortpathxy(2,:),'r-')
            plot(shortpathxy(1,lastv),shortpathxy(2,lastv),'s','MarkerFaceColor','g')
            plot(shortpathxy(1,1),shortpathxy(2,1),'s','MarkerFaceColor','r')
            camroll(270)
            axis off
        end
    end
end
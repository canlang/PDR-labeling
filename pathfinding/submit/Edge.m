%% Edge
classdef Edge
    properties
       node_index %neighbour node index
       w %distance from a parent node to a connected child node
    end
    methods
        function e=Edge(node_index,w)
            e.node_index=node_index;
            e.w=w;
        end
    end
end
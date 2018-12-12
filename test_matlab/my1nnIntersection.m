function [idx, dist] = my1nnIntersection(R, p)
    dist = zeros(size(R,1),1);
    for i = 1:size(R,1)
        dist(i) = seglineToPoint(R(i,:), p);
    end
    
    idx = find(dist == min(dist));
    dist = min(dist);
end
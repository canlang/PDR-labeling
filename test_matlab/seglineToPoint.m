function dist = seglineToPoint(segLine, p)
%   http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment

    x1 = segLine(1);
    y1 = segLine(2);
    x2 = segLine(3);
    y2 = segLine(4);
    x3 = p(1);
    y3 = p(2);
    
    px = x2-x1;
    py = y2-y1;
    
    something = px*px + py*py;

    u =  ((x3 - x1) * px + (y3 - y1) * py) / something;

    if (u > 1)
        u = 1;
    elseif (u < 0)
        u = 0;
    end

    x = x1 + u * px;
    y = y1 + u * py;

    dx = x - x3;
    dy = y - y3;

    dist = sqrt(dx*dx + dy*dy);
end
function [x2,y2] = adjustRotation(x0,y0,x1,y1,angle)
    delta = [x0,y0];
    v = [x1,y1] - delta;
    v = [...
        cosd(angle)*v(1)-sind(angle)*v(2),...
        sind(angle)*v(1)+cosd(angle)*v(2)];
    v = v + delta;
    x2 = v(1);
    y2 = v(2);
end
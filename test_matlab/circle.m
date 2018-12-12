function circle(x,y,r,h)
    %x and y are the coordinates of the center of the circle
    %r is the radius of the circle
    %0.01 is the angle step, bigger values will draw the circle faster but
    %you might notice imperfections (not very smooth)
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    
    % heading angle calc
    x_dot = x + r*(0*cosd(h) - 1*sind(h));
    y_dot = y + r*(0*sind(h) + 1*cosd(h));
    
    hold on
    plot(x+xp,y+yp);
    line([x,x_dot], [y,y_dot]);
    hold off
end
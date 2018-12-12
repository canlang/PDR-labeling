clear all, close all;
y = 2;
x = 1:10;
% x_pdr = x + rand(1,10)-.5;
x_pdr = x;
x_wps = x + normrnd(0,1,1,10);


c_loc   = 0; 
c_head  = -90;
c_v     = 0;


delta = 0.05;
time = 0:delta:10;
for t = time
    % --    DRAW ALL ESTIMATION     --
    subplot(2,1,2)
    plot(x_pdr,y,'x','LineWidth', 1,'MarkerSize',10);
    hold on
    plot(x_wps,y,'--rs','LineWidth',1,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','c',...
                'MarkerSize',10);
    hold off
    % --    UPDATE FROM PDR     --
    %       HEADING
    c_head = c_head - 0.2;
    %       VELOCITY
    if floor(t) == t && floor(t)+1 <= time(end)
        c_v     = abs(c_loc-x_pdr(floor(t)+1));
    % --    DRAW NEXT STEP      --    
    end
    if floor(t)+1 < time(end)
        hold on
        plot(x_wps(floor(t)+1),y,'rs','LineWidth', 1, ...
            'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize', 12);
        plot(x_pdr(floor(t)+1),y,'rx','LineWidth', 1.1, 'MarkerSize', 10);
        hold off
    else
        hold on
        plot(x_wps(end),y,'rs','LineWidth', 1, ...
            'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize', 12);
        plot(x_pdr(end),y,'rx','LineWidth', 1.1, 'MarkerSize', 10);
        hold off
    end
    ylim([0, 4]);
    axis equal
    grid on
    
    
    subplot(2,1,1)
    plot(x,y,'o','LineWidth', 1,'MarkerSize',10);
    ylim([0, 4]);
    axis equal
    grid on
    
    c_loc = c_loc + delta*c_v;
    circle(c_loc,y,0.5,c_head)
    i = find(time==t);
    M(i) = getframe;
end
%%
% clf
% axes('position', [0 0 1 1])
% movie(M,1)
    
    
% while s_loc < x(end)
%     plot(x_,y,'x');
%     daspect([1 1 1])    
%     circle(1,y,0.5)
% end
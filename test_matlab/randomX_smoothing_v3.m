clear all, close all;

x = [0:9;zeros(1,10)];

% noise of pdr
% x_pdr = x;          % no error case
h = -90;
step_length = 1;

x_pdr = zeros(2,10);
for i = 1:9
    h = h-2;
    x_pdr(1,i+1) = x_pdr(1,i) + step_length*(0*cosd(h) - 1*sind(h));
    x_pdr(2,i+1) = x_pdr(2,i) + step_length*(0*sind(h) + 1*cosd(h));    
end

% noise of wps
x_wps = x;
x_wps(1,:) = x_wps(1,:) + normrnd(0,1.5,1,10);
x_wps(2,:) = x_wps(2,:) + normrnd(0,1,1,10);
% x_wps = x_wps + normrnd(0,3,2,10);

c_loc   = [0;0]; 
c_head  = -90;
c_v     = [0;0];


delta = 0.05;
time = 0:delta:10;
for t = time
    % --    DRAW ALL ESTIMATION     --
    subplot(2,1,2)
    plot(x_pdr(1,:),x_pdr(2,:),'x','LineWidth', 1,'MarkerSize',10);
    hold on
    plot(x(1,:), x(2,:), '-','LineWidth',1.5)
    plot(x_wps(1,:),x_wps(2,:),'rs','LineWidth',1,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','c',...
                'MarkerSize',10);
    hold off
    % --    CURRENT STATE UPDATE FROM PDR & WI-FI    --
    %       HEADING
    c_head = c_head - 0.2;
    %       VELOCITY
    if (floor(t) == t) && (floor(t)+1 <= time(end))
%         c_v = [abs(c_loc(1)-x_pdr(1,floor(t)+1)); 0];
        w1 = 1/1; w2 = 1/3;
        L = (w1*x_pdr(:,floor(t)+1) + w2*x_wps(:,floor(t)+1))/(w1+w2);
        c_v = (L - c_loc);
    end
    %       LOCATION
    c_loc = c_loc + delta*c_v;
    % --    DRAW NEXT STEP      --    
    if floor(t)+1 < time(end)
        hold on
        plot(x_wps(1,floor(t)+1),x_wps(2,floor(t)+1),'rs','LineWidth', 1, ...
            'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize', 12);
        plot(x_pdr(1,floor(t)+1),x_pdr(2,floor(t)+1),'rx','LineWidth', 1.1,...
            'MarkerSize', 10);
        hold off
    else
        hold on
        plot(x_wps(1,end),x_wps(2,end),'rs','LineWidth', 1, ...
            'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize', 12);
        plot(x_pdr(1,end),x_pdr(2,end),'rx','LineWidth', 1.1, 'MarkerSize', 10);
        hold off
    end
    ylim([-2, 2]);
    axis equal
    grid on
    
    subplot(2,1,1)
    plot(x(1,:),x(2,:),'+r');
%     ,'LineWidth', 1,'MarkerSize',10);
    ylim([-2, 2]);
    axis equal
    grid on
    
    
    circle(c_loc(1),c_loc(2),0.5,c_head)
    i = find(time==t);
    M(i) = getframe(gcf);
end
%%
% clf
% axes('position', [0 0 1 1])
close all
[h,w,p] = size(M(1).cdata);
hf = figure;
set(hf,'position',[150,150,w+200,h+200]);
axis off
movie(M,1);
mplay(M)
    
% while s_loc < x(end)
%     plot(x_,y,'x');
%     daspect([1 1 1])    
%     circle(1,y,0.5)
% end
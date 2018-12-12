clear all; close all;

acc = load('accelerometer.txt');
mag = load('magneticfield.txt');
ori = load('orientation.txt');
chk = load('checkpoints.txt');

st = acc(1);
acc(:,1)=(acc(:,1)-st)/1000;
mag(:,5)=(mag(:,1)-chk(1))/1000;
mag(:,1)=(mag(:,1)-st)/1000;
ori(:,5)=(ori(:,1)-chk(1))/1000;
ori(:,1)=(ori(:,1)-st)/1000;
chk = (chk-st)/1000;


xa=acc(:,2);
ya=acc(:,3);
za=acc(:,4);
accnorm=sqrt(power(xa,2)+power(ya,2)+power(za,2))-9.8;

xm=mag(:,2);
ym=mag(:,3);
zm=mag(:,4);

xo=ori(:,2);
yo=ori(:,3);
zo=ori(:,4);

ori1=ori(ori(:,5)<0,:);
ori2=ori(ori(:,5)>0,:);
xo1=ori1(:,2);
yo1=ori1(:,3);
zo1=ori1(:,4);
xo2=ori2(:,2);
yo2=ori2(:,3);
zo2=ori2(:,4);

%% acc 1.
t=acc(:,1);

close all
figure('position',[200 600 2000 500])
plot(t,accnorm,'b','linewidth',1.5);
xlabel('time(s)','FontSize',20,'FontName','Times New Roman');
ylabel('m/s^2','FontSize',20,'FontName','Times New Roman');

gridxy(chk,'Color',[0.9 1.0 0.2],'linewidth',5);
% 1.1
% [pks,locs]=findpeaks(accnorm(1:167),'minpeakheight',1);
% [pks,locs]=findpeaks(accnorm(acc(:,1)>150 & acc(:,1)<200),'minpeakheight',1);
[pks,locs]=findpeaks(accnorm,'minpeakheight',1);
hold all
plot(t(locs),pks,'mo','linewidth',3,'markersize',11);
hold off
set(gca,'FontSize',18,'FontName','Times New Roman','LineWidth',1.5);

ylim([-4 5])

%% acc 2.
% [pks,locs]=findpeaks(n(1:167),'minpeakheight',1);
[pks,locs]=findpeaks(accnorm,'minpeakheight',1);
x=[0 0];
time = knnsearch(ori(:,1),t(locs));
% time(1:40)=time(41);
for i=1:size(time,1)
%     n5 : -345 degree
    x(end+1,1) = x(end,1) + 0.55*cos((xo(time(i)))/180*pi);
    x(end,2) = x(end-1,2) + 0.55*sin((xo(time(i)))/180*pi);
end

x=(x/0.05);
x(:,1)=x(:,1)+865;
x(:,2)=x(:,2)+177;

close all
% rgb=imread('n5map_crop2.png');
% [m,n,a]=size(rgb);
% figure('Units','pixels','Position',[800 200 n m])
% image(rgb);
% set(gca,'Position',[0 0 1 1])

figure('position', [200 200, 1000, 1000])
daspect([1 1 1])

hold all
% plot(x(:,1),x(:,2),'--x');

plot(x(1:1:53,1),x(1:1:53,2),'x','LineWidth',2.5,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','g',...
                'MarkerSize',11);
            
% plot(x(1:end,1),x(1:end,2),'--mo',...
%                 'LineWidth',2,...
%                 'MarkerEdgeColor','k',...
%                 'MarkerFaceColor',[.49 1 .63],...
%                 'MarkerSize',10)
            
% daspect([1 1 1])       % x,y,z ratio
hold off

%% ori
t=ori(:,1);
% plot3(xo,yo,zo,'.');

% 1.
% plot3(xo1,yo1,zo1,'.');
% hold all
% plot3(xo2,yo2,zo2,'.');
% hold off
% grid on
% xlabel('azimuth')
% ylabel('pinch')
% zlabel('roll')

% 2.
close all
figure('position', [2590 800 1000 400])

subplot(2,1,1)
plot(t,xo);
xlabel('time(s)','FontSize',20,'FontName','Times New Roman');
ylabel('\alpha','FontSize',20,'FontName','Times New Roman');
ylim([0,360]);
set(gca,'FontSize',18,'FontName','Times New Roman','LineWidth',1.5);
gridxy(chk,'Color',[0.9 1.0 0.2],'linewidth',5);

subplot(2,1,2)
plot(t,sin(xo/180*pi));
% y=lpfilter(sin(xo/360*pi),5,800);
hold all
% plot(t,yo);
hold off
xlabel('time(s)','FontSize',20,'FontName','Times New Roman');
ylabel('sin(\alpha)','FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',18,'FontName','Times New Roman','LineWidth',1.5);
gridxy(chk,'Color',[0.9 1.0 0.2],'linewidth',5);

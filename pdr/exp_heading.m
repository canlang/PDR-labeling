clear all, close all;

addpath('2');
mag = load('MagneticLog.txt');

stat = load('StateLog2.txt');
even = load('EventLog2.txt');

rmpath('2');

% acc(:,1:2) = (acc(:,1:2)-even(1))/1000;
mag(:,1:2) = (mag(:,1:2)-even(1))/1000;
stat=stat(:,1);
stat = (stat-even(1))/1000;

t=mag(:,1);
x=mag(:,3);
y=mag(:,4);
z=mag(:,5);
n=sqrt(power(x,2)+power(y,2)+power(z,2));

plot(t,x);

hold all
% plot(t,x);
plot(t,y);
plot(t,z);

gridxy(stat,'Color',[0.9 1.0 0.2],'linewidth',3);
hold off

% turning_idx=zeros(1,2);
% for i=1:2
%     d=abs(acc(:,1)-stat(i));
%     min_val = min(d);
%     turning_idx(i) = find(d == min_val);
% end


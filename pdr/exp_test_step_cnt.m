clear all, close all;
acc = load('AccelerationLog.txt');
% mag = load('MagneticLog.txt');
% agl = load('AngleSpeedLog.txt');
% dir = load('DirectionLog.txt.');
stat = load('StateLog.txt');
even = load('EventLog.txt');

acc(:,1:2) = (acc(:,1:2)-even(1))/1000;
% mag(:,1:2) = (mag(:,1:2)-even(1))/1000;
stat=stat(:,1);
stat = (stat-even(1))/1000;

t=acc(:,1);
x=acc(:,3);
y=acc(:,4);
z=acc(:,5);
n=sqrt(power(x,2)+power(y,2)+power(z,2))-9.8;

plot(t,n);

hold all
% plot(t,x);
% plot(t,y);
% plot(t,z);
% plot(mag(:,1),mag(:,3));

gridxy(stat,'Color',[0.9 1.0 0.2],'linewidth',3);
hold off

% turning_idx=zeros(1,2);
% for i=1:2
%     d=abs(acc(:,1)-stat(i));
%     min_val = min(d);
%     turning_idx(i) = find(d == min_val);
% end


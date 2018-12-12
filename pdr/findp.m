close all, clear all;
s = load('Results2.txt');
s2 = load('Result3.txt');
% s2 = s2(600:3400,:);
s2 = s2(600:1200,:);

% s = s(800:1500);
p = findpeaks(s);
ss = smooth(s,50);
% plot(s2);

x=s2(:,1);
y=s2(:,2);

% trapz(x,y)*x(end)
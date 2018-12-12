clear all; close all;

X = load('Result3.txt');
Fs=3600;
n=5;
Wn=50;
Fn=Fs/2;
ftype = 'low';
[b,a] = butter(n,Wn/Fn,ftype);
y=filter(b,a,X);
m=X;m2=X;
m(:,2)=medfilt1(X(:,2),3);
m2(:,2)=medfilt1(y(:,2),3);

idx = 600:1000;
% idx = 1:200;
y = y(idx,:);
X = X(idx,:);
m = m(idx,:);
m2 = m2(idx,:);

plot(y(:,1),y(:,2));
hold all
plot(X(:,1),X(:,2));
plot(m(:,1),m(:,2));
plot(m2(:,1),m2(:,2));
hold off
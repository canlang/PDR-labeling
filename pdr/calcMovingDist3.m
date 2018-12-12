% calc. moving distance
close all, clear all;

r = load('Result3.txt');
lfh = load('LittleFast_Hand_Ver_1.txt');
lfp = load('LittleFast_Pocket_Hor_1.txt');
lfp2 = load('LittleFast_Pocket_Ver_1.txt');
lsh = load('LittleSlow_Hand_Ver_1.txt');
nh = load('Normal_Hand_Ver_1.txt');
sh = load('Slow_Hand_Ver_1.txt');

% data = [(0:0.1:2*pi)' (sin(0:0.1:2*pi))'];
data = r;
% data = lfh;
% data = lfp;
% data = lfp2;
% data = lsh;


for i = 2:size(data,1)
    x = data(1:i,1); % time
    y = data(1:i,2); % acc.
    v = trapz(x,y);
    data(i,3) = v;
end

data(:,4) = smooth(data(:,3),100);  % v'

for i = 2:size(data,1)
    x = data(1:i,1); % time
    y = data(1:i,3); % velocity
    data(i,5) = trapz(x,y);
    
    x = data(1:i,1); % time
    y = data(1:i,3); % velocity
    data(i,6) = trapz(x,y);
end

subplot(1,2,1);
plot(data(:,5));
hold all
plot(data(:,3));
plot(data(:,2));
hold off
ylim([-20, 50]);
legend('distance', 'velocity', 'acc.','location','best');

subplot(1,2,2);
plot(data(:,6));
hold all
plot(data(:,4));
plot(data(:,2));
hold off
ylim([-20, 50]);
legend('distance', 'velocity', 'acc.','location','best');

h = figure(1);
set(h,'Position', [200 500 2000 400]);
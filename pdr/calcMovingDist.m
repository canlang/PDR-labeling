% calc. moving distance
close all, clear all;

r = load('Result3.txt');
lfh = load('LittleFast_Hand_Ver_1.txt');
lfp = load('LittleFast_Pocket_Hor_1.txt');
lfp2 = load('LittleFast_Pocket_Ver_1.txt');
lsh = load('LittleSlow_Hand_Ver_1.txt');
nh = load('Normal_Hand_Ver_1.txt');
sh = load('Slow_Hand_Ver_1.txt');

data = r;
% data = lfh;
% data = lfp;

for i = 2:size(data,1)
    x = data(1:i,1); % time
    y = data(1:i,2); % acc.
    v = trapz(x,y);
    d = v*x(end);
    data(i,[3 4]) = [v d];
end

idx = 600:1000;

plot(data(idx,4));
hold all
plot(data(idx,3));
plot(data(idx,2));
hold off
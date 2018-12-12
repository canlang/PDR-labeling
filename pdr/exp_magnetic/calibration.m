clear all, close all;
mag = load('magneticfield.txt');
chk = load('checkpoints.txt');

start_time = mag(1);
mag(:,end+1) = (mag(:,1)-start_time)/1000;
chk = (chk-start_time)/1000;

t = mag(:,end);
xyz = mag(:,2:4);

hold all
for i = 1:3
    plot(t,xyz(:,i));
end
hold off

gridxy(chk,'Color',[0.9 1.0 0.2],'linewidth',5);
clear all, close all;
% x = 1:10;
% 
% h = -90;
% h_drift = -0.2;
% step_length = 1;
% 
% x_dot = zeros(2,10);
% 
% 
% for i = 2:size(x,2)
%     disp (i)
%     h = h + h_drift;
%     x_dot(1,i) = x(i) + step_length*(0*cosd(h) - 1*sind(h));
%     x_dot(2,i) = y + step_length*(0*sind(h) + 1*cosd(h));
% end
% 
% plot(x_dot(1,:), x_dot(2,:),'x')

h = -90;
step_length = 1;

x_pdr = zeros(2,10);
for i = 1:9
    h = h-2;
    x_pdr(1,i+1) = x_pdr(1,i) + step_length*(0*cosd(h) - 1*sind(h));
    x_pdr(2,i+1) = x_pdr(2,i) + step_length*(0*sind(h) + 1*cosd(h));    
end

plot(x_pdr(1,:),x_pdr(2,:),'x')
axis equal
clear all, close all;

filename = 'step_angle_location.txt';
roadfilename = 'n1_roadnetwork.txt';

delimiterIn = ',';
A = importdata(filename, delimiterIn);
R = importdata(roadfilename,delimiterIn);

% angle = zeros(657,1);
angle = zeros(60,1);
current = 0;
for i = 1:size(angle,1)
    switch A.textdata{i}
        case 'A'
            current = A.data(i,2);
%         otherwise
%             angle(i) = current;
    end
    angle(i) = current;
end
plot(angle);

% angle = A.data(ismember(A.textdata,'A'),:);
% plot(angle(:,2));

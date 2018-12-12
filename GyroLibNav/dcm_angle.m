function [rz, ry, rx] = dcm_angle(Cbn)
%  [rz, ry, rx] = dcm_angle(Cbn)
%  Transforms direction cosine matrix to Euler angles
%
%   Input arguments:
%   Cbn -  Direction cosine matrix [3,3]
%
%   Output arguments:
%   rz, ry, rx - Euler angles around Z, Y and X axes correspondingly

rz = atan2(Cbn(1,2),Cbn(1,1));
ry = -atan2(Cbn(1,3),sqrt(1-Cbn(1,3)^2));
rx = atan2(Cbn(2,3),Cbn(3,3));
end
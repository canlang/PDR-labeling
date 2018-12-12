function C = dcmnormalize(C)
%  C = dcmnormalize(C)
%  %Normalize Direction Cosine Matrix (DCM)
%
%   Input arguments:
%   C -  Direction cosine matrix [3,3]
%
%   Output arguments:
%   C -  Direction cosine matrix [3,3]

delta_12 = C(1,:)*C(2,:)';
C(1,:) = C(1,:) - 1/2*delta_12*C(2,:);
C(2,:) = C(2,:) - 1/2*delta_12*C(1,:);
C(3,:) = cross(C(1,:),C(2,:));
C(1,:) = C(1,:)./sqrt((C(1,:))*(C(1,:))');
C(2,:) = C(2,:)./sqrt((C(2,:))*(C(2,:))');
C(3,:) = C(3,:)./sqrt((C(3,:))*(C(3,:))');
end

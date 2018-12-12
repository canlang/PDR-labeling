function [phi,the,psi]= ang_eulero321(BN) 

% set 3-2-1 standard
% phi roll  (rotation around x-body-axis)   [  -pi,  pi]
% the pitch (rotation around y-body-axis)   [-pi/2,pi/2]
% psi yaw   (rotation around z-body-axis)   [  -pi,  pi]

% [        c(th)c(psi)                        c(th)c(psi)               -s(th)     ]
% [s(phi)s(th)c(psi)-c(phi)s(psi)   s(phi)s(th)s(psi)+c(phi)c(psi)    s(phi)c(th)  ]
% [c(phi)s(th)c(psi)+s(phi)s(psi)   c(phi)s(th)s(psi)-s(phi)c(psi)    c(phi)c(th)  ]

the=asin(-BN(1,3));
acthe=abs(cos(the));
if(acthe > 1e-10)
    phi=atan2(BN(2,3),BN(3,3));
    psi=atan2(BN(1,2),BN(1,1));
else
    phi=0;
    if(BN(1,3) > 0) %theta=pi/2
        psi=atan2(-BN(3,2),-BN(3,1));
    else
        psi=atan2(BN(3,2),BN(3,1));
    end
end
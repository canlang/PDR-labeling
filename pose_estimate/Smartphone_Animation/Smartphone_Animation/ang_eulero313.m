function [ra,i,om]= ang_eulero313(BN) 

% set 3-1-3 standard
% ra RAAN                 precession       [-pi,pi]         
% i  inclination          nutation         [  0,pi]         
% om argument of perigee  spin             [-pi,pi]         

% [ c(om)c(ra)-s(om)s(ra)c(i)        c(om)s(ra)+s(om)c(ra)c(i)         s(om)s(i)   ]
% [-s(om)c(ra)-c(om)s(ra)c(i)       -s(om)s(ra)+c(om)c(ra)c(i)         c(om)s(i)   ]
% [       s(ra)s(i)                         -c(ra)s(i)                    c(i)     ]

i=acos(BN(3,3));
asi=abs(sin(i));
if(asi > 1e-10)
    ra=atan2(BN(3,1),-BN(3,2));
    om=atan2(BN(1,3),BN(2,3));
else
    ra=0;
    om=atan2(BN(1,2),BN(2,2));
end
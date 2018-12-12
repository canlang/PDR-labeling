function [q0,q1,q2,q3]= quaternion(BN)

if (1+BN(1,1)+BN(2,2)+BN(3,3)>0.000001)
    q0=sqrt(1+BN(1,1)+BN(2,2)+BN(3,3))/2; % q0>0 (rotazione corta)
else
    q0=0;
end
if (q0~=0)
    q1=(BN(2,3)-BN(3,2))/(4*q0);
    q2=(BN(3,1)-BN(1,3))/(4*q0);
    q3=(BN(1,2)-BN(2,1))/(4*q0);
else
    q1=sqrt((BN(1,1)+1)/2);
    q2=sqrt((BN(2,2)+1)/2);
    q3=sqrt((BN(3,3)+1)/2);
    if (q1==max([q1,q2,q3]))
        if ((BN(2,3)-BN(3,2))<0)
            q1 = -q1;
            q2 = -q2;
            q3 = -q3;
        else
            if ((BN(1,3)+BN(3,1))<0) %q1>0 sicuro
                q3 = -q3;
            end
            if ((BN(1,2)+BN(2,1))<0)
                q2 = -q2;
            end
        end
    elseif (q2==max([q1,q2,q3]))
        if ((BN(3,1)-BN(1,3))<0)
            q1 = -q1;
            q2 = -q2;
            q3 = -q3;
        else
            if ((BN(1,2)+BN(2,1))<0) %q2>0 sicuro
                q1 = -q1;
            end
            if ((BN(3,2)+BN(2,3))<0)
                q3 = -q3;
            end
        end
    elseif (q3==max([q1,q2,q3]))
        if ((BN(1,2)-BN(2,1))<0)
            q1 = -q1;
            q2 = -q2;
            q3 = -q3;
        else
            if ((BN(1,3)+BN(3,1))<0) %q3>0 sicuro
                q1 = -q1;
            end
            if ((BN(3,2)+BN(2,3))<0)
                q2 = -q2;
            end
        end
    else
        q1=0;
        q2=0;
        q3=0;
    end
end
            
        
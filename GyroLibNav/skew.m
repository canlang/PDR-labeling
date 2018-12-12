function C=skew(w)
%Creates skew-symmetric matrix C  from vector w
C=[
    0   -w(3)  w(2)
    w(3)   0   -w(1)
    -w(2)  w(1)    0];
end


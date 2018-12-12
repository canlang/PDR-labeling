%% function [C_INERtoBODY] = triad(B_in,A_in,B_b,A_b)
% All inputs are column vectors and respectively:
% B_in = local magnetic field
% A_in = inertial accelerations
% B_b = body magnetic field
% A_b = body accelerations

function [C_INERtoBODY] = triad(A_in,B_in,A_b,B_b)

a_in   = A_in./norm(A_in);                          % unit vector for local acceleration
b_in   = B_in./norm(B_in);                          % unit vector for local magnetic field given from IGRF
v1_in  = cross(a_in,b_in)./norm(cross(a_in,b_in));  % l1 first  vector for local triad -->( g x b )
v2_in  = cross(a_in,v1_in);                         % l2 second vector for local triad -->( g x l1)
v3_in  = cross(v1_in,v2_in);                        % l3 third  vector for local triad -->(l1 x l2)

a_b    = A_b./norm(A_b);                            % unit vector for body magnetic field given from smartphone data
b_b    = B_b./norm(B_b);                            % unit vector for body accelerations  given from smartphone data
v1_b   = cross(a_b,b_b)./norm(cross(a_b,b_b));      % b1 first  vector for body triad -->( g x b )
v2_b   = cross(a_b,v1_b);                           % b2 second vector for body triad -->( g x b1)
v3_b   = cross(v1_b,v2_b);                          % b3 third  vector for body triad -->(b1 x b2)

BODY   = [v1_b,v2_b,v3_b];                          % body  reference system matrix
INER   = [v1_in,v2_in,v3_in];                       % local reference system matrix
C_INERtoBODY = BODY*INER';                          % attitude matrix to pass from L to B
    
end
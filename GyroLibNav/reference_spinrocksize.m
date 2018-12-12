function [Rn_ref, Vn_ref, Euler_ref, dThe_ref, dUps_ref, dt, gn, Rn0, Vn0, Cbn0] = ...
    reference_spinrocksize
% Creates reference data for SINS algorithm simulation
% Reference movement of the body - SPIN-ROCK-SIZE truth model
% Ref: Paul G. Savage, Strapdown System Performance Analysis
%
%   Output arguments:
%   Rn_    - Reference body position in navigation frame, m
%   Vn_    - Reference body velocity in navigation frame, m/s
%   Euler_ - Reference body attitude angles, rad
%   dThe_  - Reference data from the gyroscope (angle increments), rad
%   dUps_  - Reference data from the accelerometer (vel. increments), m/s
%   dt   - Computer cycle, sec
%   gn   - Gravity vector in navigation frame, m/s^2
%   Rn0  - Initial position, m
%   Vn0  - Initial velocity, m/s
%   Cbn0 - Initial attitude direction cosine matrix (DCM)

%% Simulation parameters
%Simulation time, sec.
tsim = 600;
%Simulation time step (computer cycle), sec.
dt = 1/100;

%% Reference movement kinematic parameters
A  = 0.5;
B  = 1.0;
Om = 0.3;
lb = [0.0; 7.0; 0.0];
C0 = angle_dcm(pi/2, pi/6, pi/6);
ug = C0*[1;0;0];

%% Gravity
gn = [0;0;9.81]; %gravity in navigation frame

%% Logs
Rn_ref    = zeros(tsim/dt,3);
Vn_ref    = zeros(tsim/dt,3);
dThe_ref  = zeros(tsim/dt,3);
dUps_ref  = zeros(tsim/dt,3);
Euler_ref = zeros(tsim/dt,3);

%% Simulation
t = 0;
cnt = 0;
gamma_  = A*t+B*sin(Om*t);
dgamma_ = A+B*Om*cos(Om*t);
fa_ = B*Om*cos(Om*t);
fb_ = (A^2+1/2*B^2*Om^2)*t+2*A*B*sin(Om*t)+1/2*B^2*Om*sin(Om*t)*cos(Om*t);
Cbn0 = eye(3);
Rn0 = Cbn0*lb;
Vn0 = dgamma_*Cbn0*cross(ug,lb);
Cbn_ = Cbn0;
while (t < tsim)
    
    t = t+dt;
    cnt = cnt+1;
    
    gamma = A*t+B*sin(Om*t);
    dgamma = A+B*Om*cos(Om*t);
    fa = B*Om*cos(Om*t);
    fb = (A^2+1/2*B^2*Om^2)*t+2*A*B*sin(Om*t)+1/2*B^2*Om*sin(Om*t)*cos(Om*t);

    %angle increment
    dThe_ref(cnt,:) = (gamma-gamma_)*ug;
    
    %velocity increment
    dUps_ref(cnt,:) = ((fa-fa_)*skew(ug)+(fb-fb_)*skew(ug)*skew(ug))*lb;
    
    %attitude
    Cbb = eye(3)+sin(gamma)*skew(ug)+(1-cos(gamma))*skew(ug)*skew(ug);
    Cbn = Cbn0*Cbb;
    [Euler_ref(cnt,1), Euler_ref(cnt,2), Euler_ref(cnt,3)] = dcm_angle(Cbn');
    
    %velocity
    Vn_ref(cnt,:) = dgamma*Cbn*cross(ug,lb);
    
    %position
    Rn_ref(cnt,:) = Cbn*lb;
    
    %add gravity
    dUps_ref(cnt,:) = dUps_ref(cnt,:)'-1/2*(Cbn'*gn+Cbn_'*gn)*dt;
    
    gamma_ = gamma;
    fa_ = fa;
    fb_ = fb;
    Cbn_ = Cbn;
end


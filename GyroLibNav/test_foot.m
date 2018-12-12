function result = test_foot( dThe, dUps, g, dt)

% result = test_foot( dThe, dUps, g, dt)
% Implements test routine for the simple foot-mounted integrated nav system
% using data from the IMU only and employing so-called ZUPT updates  
%
%   Input arguments:
%   dThe  - Data from the gyroscope (angle increments), rad
%   dUps  - Data from the accelerometer (velocity increments), m/s
%   g  - Gravity vector in navigation frame, m/s^2
%   dt - Computer cycle, sec.

%   Output arguments:
%   result.rn - Estimated position in navigation frame, m
%   result.vb - Estimated velocity in body frame, m/s
%   result.euler - Estimated attitude angles, rad,
%   result.df - Estimated gyroscope angles increments bias, rad
%   result.dw - Estimated accelerometer velocities increments bias, m/s

%% figure
close all;
figure;
view(3);
axis equal;
hold on; grid on;
set(gca,'Xlim',[-1 5]);
set(gca,'Ylim',[-3 1.5]);
set(gca,'Zlim',[-1 1]);
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
set(gcf,'renderer','opengl');
animation_rate = 1;

%% INS Axes
ai1 = [0; 0; 1.5;]*0.5;
ai2 = [0; 1.5; 0;]*0.5;
ai3 = [1.5; 0; 0;]*0.5;
sz =  [0; 0; 0;];
pai1 = plot3([sz(1) ai1(1)],[sz(2) ai1(2)],...
    [sz(3) ai1(3)],'b-','linewidth',1);
pai2 = plot3([sz(1) ai2(1)],[sz(2) ai2(2)],...
    [sz(3) ai2(3)],'g-','linewidth',1);
pai3 = plot3([sz(1) ai3(1)],[sz(2) ai3(2)],...
    [sz(3) ai3(3)],'r-','linewidth',1);

%% Cube patch for INS attitude animation
Vert = [ 0 0 0 ; 1 0 0 ; 1 1 0 ; 0 1 0 ; 0 0 1 ; 1 0 1 ; 1 1 1 ; 0 1 1];
Vert(:,1) = (Vert(:,1)-0.5)*0.5;
Vert(:,2) = (Vert(:,2)-0.5)*0.5;
Vert(:,3) = (Vert(:,3)-0.5)*0.5;
Faces = [ 1 2 6 5 ; 2 3 7 6 ; 3 4 8 7 ; 4 1 5 8 ; 1 2 3 4 ; 5 6 7 8 ];
ptch.Vertices = Vert;
ptch.Faces = Faces;
ptch.FaceVertexCData = gray(6);
ptch.FaceColor = 'flat';
ptch.EdgeColor = 'black';
ptch.LineWidth = 1;
patch_handle = patch(ptch);

%% Markers
%Text
th = text(-1,0.5,1,'Time ');
set(th,'FontSize',10,'Color','b');
%Trajectory trails
hi = plot3(0,0,0,'r-');

%% Logs
Nsim = size(dThe,1); %number of simulation steps
Rn_ = zeros(Nsim,3);
Vb_ = zeros(Nsim,3);
Euler_  = zeros(Nsim,3);
df_  = zeros(Nsim,3);
dw_  = zeros(Nsim,3);

%% ZUPT parameters
%ZUPT "measurements noises"
nzv = 1e-7;
nzw = 1e-1;
the_threshold = 2e-3;

%% Initial State
%Position, Velocity and Attitude
Rn   = [0.0; 0.0; 0.0];
Vb   = [0.0; 0.0; 0.0];
%Accelerometer-based angles for initial attitude
fb = mean(dUps(1:600,:));
ry =  atan2(fb(1),sqrt(fb(2)^2+fb(3)^2));
rx = -atan2(fb(2),sqrt(fb(1)^2+fb(3)^2));
Cbn  = angle_dcm(0,ry,rx)';
%Sensors biases
dw   = zeros(3,1);
df   = zeros(3,1);

%Covariance matrix
P    = zeros(15,15);
pos_sig = 1e-2;
P(1:3,1:3)     = diag([pos_sig, pos_sig, pos_sig]); %position
vel_sig = 1e-7;
P(4:6,4:6)     = diag([vel_sig, vel_sig, vel_sig]); %velocity
att_sig = 1e-7;
P(7:9,7:9)     = diag([att_sig, att_sig, att_sig]); %attitude
acc_sig = 1e-2;
P(10:12,10:12) = diag([acc_sig, acc_sig, acc_sig]); %acc. bias
gyr_sig = eps;
P(13:15,13:15) = diag([gyr_sig, gyr_sig, gyr_sig]); %qyr. bias

%% EKF Parameters
vel_noise   = 1e-3;  %velocity noise
att_noise   = 1e-4;  %attitude noise
accl_noise  = 1e-12; %accelerometers bias noise
gyro_noise  = 1e-16; %gyroscopes bias noise

%% Main loop
for t=1:Nsim
    
    %% Inertial Sensors Readings
    dthe = dThe(t,:)';
    dups = dUps(t,:)';
    
    %% SINS mechanization
    %Compensate inertial sensors biases
    dthe = dthe-dw*dt;
    dups = dups-df*dt;
    %Call mechanization function
    [Cbn, Vb, Rn] = sins_mech(Cbn, Vb, Rn, dthe, dups, g, dt);
    
    %% Kalman Predict
    %INS Error Model
    [F, Q] = eror_model(Vb, dthe, Cbn, g, dt, vel_noise, att_noise,...
        accl_noise,gyro_noise);
    %Covariance predict
    P = F*P*F'+Q;
    
    %% Kalman Update - ZUPT
    the_norm = norm(dthe);
    if (the_norm < the_threshold)
        %measurement vector
        Wb = dthe/dt;
        v = [Vb; Wb;];
        %measurement matrix
        H = zeros(6,15);
        H(1:3,4:6)   = eye(3);
        H(4:6,13:15) = eye(3);      
        %measurement noise covariance
        R = diag([nzv; nzv; nzv; nzw; nzw; nzw]);
        %Update
        I = eye(15);
        K = (P*H')/(H*P*H'+ R);
        P = (I-K*H)*P*(I-K*H)' + K*R*K';
        x = K*v;  
        %Correct Position with Position error estimate
        Rn = Rn-x(1:3,1);
        %Correct Velocity vith Velocity error estimate
        Vb = Vb-x(4:6,1);
        %Correct Attitude DCM
        En  = skew(x(7:9,1));
        I   = eye(3);
        Cbn = (I+En)*Cbn;
        %Normalize Cbn matrix
        Cbn = dcmnormalize(Cbn);
        %Update gyro bias estimate
        df = df+x(10:12,1);
        %Update accel bias estimate
        dw = dw+x(13:15,1);  
    end
    
    %% Collect Logs
    Rn_(t,:)    = Rn;
    Vb_(t,:)    = Vb;
    [Euler_(t,1), Euler_(t,2), Euler_(t,3)]  = dcm_angle(Cbn');
    dw_(t,:)    = dw*dt;
    df_(t,:)    = df*dt;
    
    %% Animations
    if(mod(t,animation_rate) == 0)
                
        %% Estimated:
        %trajectory
        set(hi,...
            'Xdata', Rn_(1:1:t,1),...
            'Ydata', Rn_(1:1:t,2),...
            'Zdata', Rn_(1:1:t,3))
        %body and body axes
        a1_ = Cbn*ai1+Rn;
        a2_ = Cbn*ai2+Rn;
        a3_ = Cbn*ai3+Rn;
        s_ = Rn';
        set(pai1,...
            'Xdata',[s_(1) a1_(1)],...
            'Ydata',[s_(2) a1_(2)],...
            'Zdata',[s_(3) a1_(3)]);
        set(pai2,...
            'Xdata',[s_(1) a2_(1)],...
            'Ydata',[s_(2) a2_(2)],...
            'Zdata',[s_(3) a2_(3)]);
        set(pai3,...
            'Xdata',[s_(1) a3_(1)],...
            'Ydata',[s_(2) a3_(2)],...
            'Zdata',[s_(3) a3_(3)]);
        
        %% Cube
        Vert_ = Vert;
        for j=1:size(Vert,1)
            Vert_(j,:) = (Vert(j,:)*Cbn')+[Rn(1,1), Rn(2,1), Rn(3,1)];
        end
        set(patch_handle,'Vertices',Vert_);
        
        set(th,'String',sprintf('Time %2.1f sec.',t*dt));
        drawnow;
        
    end
end

%% Plot Results
t = (1:Nsim)*dt;
figure('Name','Position');
hold on; grid on; axis tight;
plot(t,Rn_,'LineWidth',2);
legend('X','Y','Z');
ylabel('Position, m');
xlabel('Time,sec');

figure('Name','Velocity');
hold on; grid on; axis tight;
plot(t,Vb_,'LineWidth',2);
legend('V_x','V_y','V_z');
ylabel('Velocity, m/s');
xlabel('Time,sec');

figure('Name','Attitude');
hold on; grid on;  axis tight;
plot(t,rad2deg(Euler_),'LineWidth',2);
legend('Z','Y','X');
ylabel('Attitude, deg');
xlabel('Time,sec');

figure('Name','Gyro Bias');
hold on; grid on; axis tight;
plot(t,dw_,'LineWidth',2);
legend('dw_x','dw_y','dw_z');
xlabel('Time,sec');

figure('Name','Accel Bias');
hold on; grid on; axis tight;
plot(t,df_,'LineWidth',2);
legend('df_x','df_y','df_z');
xlabel('Time,sec');

result.rn = Rn_;
result.vb = Vb_;
result.euler = Euler_;
result.df = df_;
result.dw = dw_;

end

%% INS Mechanization
function [Cbn_new, Vb_new, Pos_new]= sins_mech(Cbn, Vb, Pos,...
    dThet, dUpsl, g, dt)
%Update Attutude
Cnb = Cbn';
rot_norm=norm(dThet);
sr_a=1-(rot_norm^2/6)+(rot_norm^4/120);
sr_b=(1/2)-(rot_norm^2/24)+(rot_norm^4/720);
Cbb=eye(3)+sr_a*skew(dThet)+sr_b*skew(dThet)*skew(dThet);
Cbn_new=Cbn*Cbb;
Cnb_new = Cbn_new';
%Update Velocity (in b-frame)
%Specific force component
dVsf_= dUpsl+(Cnb*g*dt);
dVsf = dUpsl+(Cnb_new*g*dt);
dVsf_med = 1/2*(dVsf_+dVsf);
%Coriolis component
dVcor=cross(Vb,dThet);
Vb_=Vb+dVsf_med+dVcor;
dVcor_=cross(Vb_,dThet);
dVcor_med = 1/2*(dVcor_+dVcor);
%Update Velocity
Vb_new=Vb+dVsf_med+dVcor_med;
%Update Position
Pos_new=Pos+1/2*(Cbn*Vb+Cbn_new*Vb_new)*dt;
end

%% INS Error Model
function [F,Q] = eror_model(Vb,dthe,Cbn,g,dt,...
    vel_noise,att_noise,...
    accl_noise,gyro_noise)
    wb = dthe/dt;
    Cnb = Cbn';
    N=zeros(9,6);
    N(7:9,4:6)=-Cbn;
    N(4:6,1:3)=eye(3);
    N(4:6,4:6)=skew(Vb);
    A=zeros(9);
    A(4:6,4:6)=-skew(wb);
    A(4:6,7:9)=Cnb*skew(-g);
    A(1:3,4:6)=Cbn;
    A(1:3,7:9)=skew(Cbn*Vb);
    Fnd=eye(9)+A*dt+A*A*dt^2/2;
    Rag = diag([...
        vel_noise, vel_noise, vel_noise,...
        att_noise, att_noise, att_noise]);
    Qn = N*Rag*N';
    Qnd=dt/2*(Fnd*Qn+Qn*Fnd');
    F = zeros(15);
    Q = zeros(15);
    F(1:9,1:9) = Fnd;
    F(10:15,10:15) = eye(6);
    F(1:9,10:15) = N*dt;
    Q(1:9,1:9) = Qnd;
    Qimu = diag([...
        accl_noise, accl_noise, accl_noise,...
        gyro_noise, gyro_noise, gyro_noise]);
    Q(10:15,10:15) = Qimu;
    Q(1:9,10:15)=N*Qimu*dt/2;
    Q(10:15,1:9)=Q(1:9,10:15)';
end
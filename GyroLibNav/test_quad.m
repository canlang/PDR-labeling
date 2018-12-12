function test_quad( dThe_ref, dUps_ref, Rn_ref, Vb_ref, Euler_ref,...
    g, dt)

% test_quad( dThe_ref, dUps_ref, Rn_ref, Vb_ref, Euler_ref, g, dt)
% Implements test routine for the simple integrated system of quadrocopter
% using the simulated data from the IMU and "GPS" 
%
%   Input arguments:
%   dThe_ref  - Reference data from the gyroscope (angle increments), rad
%   dUps_ref  - Reference data from the accelerometer (vel. increments), m/s
%   Rn_ref    - Reference quadrocopter position in navigation frame, m
%   Vb_ref    - Reference quadrocopter velocity in body frame, m/s
%   Euler_ref - Reference quadrocopetr  attitude angles, rad
%   g  - Gravity vector in navigation frame, m/s^2
%   dt - Computer cycle, sec.

%% Reproducible results
rng(100);

%% figure
close all;
figure;
view(3);
axis equal;
hold on; grid on;
set(gca,'Xlim',[min(Rn_ref(:,1))-20 max(Rn_ref(:,1))+20]);
set(gca,'Ylim',[min(Rn_ref(:,2))-20 max(Rn_ref(:,2))+20]);
set(gca,'Zlim',[min(Rn_ref(:,3))-20 max(Rn_ref(:,3))+20]);
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
set(gcf,'renderer','opengl');
animation_rate = 50;

%% Target axes
at1 = [0; 0; 2.5]*10;
at2 = [0; 2.5; 0]*10;
at3 = [2.5; 0; 0]*10;
st =  [0; 0; 0];
pat1 = plot3([st(1) at1(1)],[st(2) at1(2)],...
    [st(3) at1(3)],'b--','linewidth',0.1);
pat2 = plot3([st(1) at2(1)],[st(2) at2(2)],...
    [st(3) at2(3)],'g--','linewidth',0.1);
pat3 = plot3([st(1) at3(1)],[st(2) at3(2)],...
    [st(3) at3(3)],'r--','linewidth',0.1);

%% INS Axes
ai1 = [0; 0; 1.5;]*8;
ai2 = [0; 1.5; 0;]*8;
ai3 = [1.5; 0; 0;]*8;
sz =  [0; 0; 0;];
pai1 = plot3([sz(1) ai1(1)],[sz(2) ai1(2)],...
    [sz(3) ai1(3)],'b-','linewidth',1);
pai2 = plot3([sz(1) ai2(1)],[sz(2) ai2(2)],...
    [sz(3) ai2(3)],'g-','linewidth',1);
pai3 = plot3([sz(1) ai3(1)],[sz(2) ai3(2)],...
    [sz(3) ai3(3)],'r-','linewidth',1);

%% Cube patch for INS attitude animation
Vert = [ 0 0 0 ; 1 0 0 ; 1 1 0 ; 0 1 0 ; 0 0 1 ; 1 0 1 ; 1 1 1 ; 0 1 1]*6;
Vert(:,1) = (Vert(:,1)-3);
Vert(:,2) = (Vert(:,2)-3);
Vert(:,3) = (Vert(:,3)-3);
Faces = [ 1 2 6 5 ; 2 3 7 6 ; 3 4 8 7 ; 4 1 5 8 ; 1 2 3 4 ; 5 6 7 8 ];
ptch.Vertices = Vert;
ptch.Faces = Faces;
ptch.FaceVertexCData = gray(6);
ptch.FaceColor = 'flat';
ptch.EdgeColor = 'black';
ptch.FaceAlpha = 0.1;
ptch.LineWidth = 1;
patch_handle = patch(ptch);

%% Markers
%Text
th = text(-25,25,20,'Time ');
set(th,'FontSize',10,'Color','b');
%Trajectory trails
ht = plot3(0,0,0,'k');
hi = plot3(0,0,0,'r');

%% Logs
Nsim = size(dThe_ref,1); %number of simulation steps
Rn_ = zeros(Nsim,3);
Vb_ = zeros(Nsim,3);
Euler_  = zeros(Nsim,3);
dEuler_ = zeros(Nsim,3);
df_  = zeros(Nsim,3);
dw_  = zeros(Nsim,3);

%% IMU Parameters
%gyro noise
nthe = 0.8e-5;
%accel noise
nups = 0.13e-3;
%gyro bias
bthe = [1.e-5;-1e-5;2e-5];
%accel bias
bups = [1.e-4;-1e-4;2e-4];

%% "GPS" Parameters
% "GPS" position measurement noise
ngp = 5e0;
% "GPS" velocity measurement noise
ngv = 1e-1;
% 5Hz GPS update rate 
gps_update_rate = 20;

%% ZUPT parameters
zupt_count = 0.5e4; 
%ZUPT "measurements noises"
nzp = 1e-2;
nzv = 1e-3;
nzw = 1e-4;

%% Initial State
%Position, Velocity and Attitude
Rn   = [0.0; 0.0; 0.0];
Vb   = [0.0; 0.0; 0.0];
Cbn  = eye(3);
dw   = zeros(3,1);
df   = zeros(3,1);
%Covariance matrix
P    = zeros(15,15);
pos_sig = 1e-1;
P(1:3,1:3)     = diag([pos_sig, pos_sig, pos_sig]); %position
vel_sig = 1e-2;
P(4:6,4:6)     = diag([vel_sig, vel_sig, vel_sig]); %velocity
att_sig = 1e-3;
P(7:9,7:9)     = diag([att_sig, att_sig, att_sig]); %attitude
acc_sig = 1e-3;
P(10:12,10:12) = diag([acc_sig, acc_sig, acc_sig]); %acc. bias
gyr_sig = 1e-3;
P(13:15,13:15) = diag([gyr_sig, gyr_sig, gyr_sig]); %qyr. bias

%% EKF Parameters
vel_noise   = 1e-4;  %velocity noise
att_noise   = 1e-6;  %attitude noise
accl_noise  = 1e-10; %accelerometers bias noise
gyro_noise  = 1e-16; %gyroscopes bias noise

%% Main loop
for t=1:Nsim
    
    %% Reference
    Rnref  = Rn_ref(t,:)';
    Cbnref = angle_dcm(Euler_ref(t,1), Euler_ref(t,2), Euler_ref(t,3))';
    Vnref  = Cbnref*Vb_ref(t,:)';
    
    %% Inertial Sensors
    dthe_ref = dThe_ref(t,:)';
    dthe = single(dthe_ref+bthe+randn(3,1)*nthe);
    dups_ref = dUps_ref(t,:)';
    dups = single(dups_ref+bups+randn(3,1)*nups);
    
    %% "GPS"
    if (mod(t,gps_update_rate) == 0)
        GPS_pos = Rnref+randn(3,1)*ngp;
        GPS_vel = Vnref+randn(3,1)*ngv;
        gps_update = 1;
    else
        gps_update = 0;
    end
    
    %% ZUPT
    if (t < zupt_count)
        zupt_update = 1;
    else
        zupt_update = 0;
    end    
    
    %% SINS mechanization
    %Compensate inertial sensors biases
    dthe = dthe-dw*dt;
    dups = dups-df*dt;
    [Cbn, Vb, Rn] = sins_mech(Cbn, Vb, Rn, dthe, dups, g, dt);
    
    %% Kalman Predict
    %INS Error Model
    [F, Q] = eror_model(Vb, dthe, Cbn, g, dt, vel_noise, att_noise,...
        accl_noise,gyro_noise);
    %Covariance predict
    P = F*P*F'+Q;
    
    %% Kalman Update - "GPS"
    if (gps_update == 1)
        %measurement vector
        v = zeros(6,1);
        %position measurement predict
        rg = GPS_pos;
        rg_hat = Rn;
        v(1:3,1) = rg_hat-rg;
        %velocity measurement predict
        vg = GPS_vel;
        vg_hat = Cbn*Vb;
        v(1:3,1) = rg_hat-rg;
        v(4:6,1) = vg_hat-vg;
        %measurement matrix
        H = zeros(6,15);
        H(1:3,1:3)   =  eye(3);
        H(4:6,4:6)   =  Cbn;
        H(4:6,7:9)   =  skew(Cbn*Vb);
        %measurement noise covariance
        R = zeros(6);
        R(1:3,1:3) = eye(3)*ngp^2;
        R(4:6,4:6) = eye(3)*ngv^2;
        %Update
        I = eye(15);
        S = (H*P*H'+ R);
        K = (P*H')/S;
        P = (I-K*H)*P*(I-K*H)' + K*R*K';
        x = K*v;
        %Correct Position 
        Rn = Rn-x(1:3,1);
        %Correct Velocity 
        Vb = Vb-x(4:6,1);
        %Correct Attitude
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
    
    %% Kalman Update - ZUPT
    if (zupt_update == 1)
        %measurement vector
        Wb = dthe/dt;
        v = [Vb; Wb; Rn(3,1)];
        %measurement matrix
        H = zeros(7,15);
        H(1:3,4:6)   = eye(3);
        H(4:6,13:15) = eye(3);      
        H(7,3)  = 1;
        %measurement noise covariance
        R = diag([nzv; nzv; nzv; nzw; nzw; nzw; nzp]);
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
    Euler_(t,:) = dcm_angle(Cbn');
    dw_(t,:)    = dw;
    df_(t,:)    = df;
    %Attitude errors
    Cref = angle_dcm(Euler_ref(t,1), Euler_ref(t,2), Euler_ref(t,3));
    Cerr = Cref*Cbn;
    [dEuler_(t,1), dEuler_(t,2), dEuler_(t,3)] = dcm_angle(Cerr);    
    
    %% Animations
    if(mod(t,animation_rate) == 0)
        
        %% Reference:
        %trajectory
        set(ht,...
            'Xdata',Rn_ref(1:10:t,1),...
            'Ydata',Rn_ref(1:10:t,2),...
            'Zdata',Rn_ref(1:10:t,3))
        %body and body axes
        a1_ = Cbnref*at1+Rn_ref(t,:)';
        a2_ = Cbnref*at2+Rn_ref(t,:)';
        a3_ = Cbnref*at3+Rn_ref(t,:)';
        s_ = Rn_ref(t,:);
        set(pat1,...
            'Xdata',[s_(1) a1_(1)],...
            'Ydata',[s_(2) a1_(2)],...
            'Zdata',[s_(3) a1_(3)]);
        set(pat2,...
            'Xdata',[s_(1) a2_(1)],...
            'Ydata',[s_(2) a2_(2)],...
            'Zdata',[s_(3) a2_(3)]);
        set(pat3,...
            'Xdata',[s_(1) a3_(1)],...
            'Ydata',[s_(2) a3_(2)],...
            'Zdata',[s_(3) a3_(3)]);
        
        %% Estimated:
        %trajectory
        set(hi,...
            'Xdata', Rn_(1:10:t,1),...
            'Ydata', Rn_(1:10:t,2),...
            'Zdata', Rn_(1:10:t,3))
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
subplot(2,1,1);
hold on; grid on; axis tight;
plot(t,Rn_,'LineWidth',2);
plot(t,Rn_ref,'--','LineWidth',2);
legend('X_{est}','Y_{est}','Z_{est}','X_{ref}','Y_{ref}','Z_{ref}');
ylabel('Position, m');
xlabel('Time,sec');
subplot(2,1,2);
hold on; grid on; axis tight;
plot(t,Rn_ref-Rn_,'LineWidth',2);
ylabel('Position error, m');
xlabel('Time,sec');

figure('Name','Velocity');
subplot(2,1,1);
hold on; grid on; axis tight;
plot(t,Vb_,'LineWidth',2);
plot(t,Vb_ref,'--','LineWidth',2);
legend('VX_{est}','VY_{est}','VZ_{est}','VX_{ref}','VY_{ref}','VZ_{ref}');
ylabel('Velocity, m/s');
xlabel('Time,sec');
subplot(2,1,2);
hold on; grid on; axis tight;
plot(t,Vb_ref-Vb_,'LineWidth',2);
ylabel('Velocity error, m/s');
xlabel('Time,sec');

figure('Name','Attitude');
subplot(2,1,1);
hold on; grid on;  axis tight;
plot(t,rad2deg(Euler_),'LineWidth',2);
plot(t,rad2deg(Euler_ref),'--','LineWidth',2);
legend('Z_{est}','Y_{est}','X_{est}','Z_{ref}','Y_{ref}','X_{ref}');
ylabel('Attitude, deg');
xlabel('Time,sec');
subplot(2,1,2);
hold on; grid on; axis tight;
plot(t,rad2deg(dEuler_),'LineWidth',2);
ylabel('Attitude error, deg');
xlabel('Time,sec');

figure('Name','Gyro Bias');
hold on; grid on; axis tight;
plot(t,dw_*dt,'LineWidth',2);
legend('dw_x','dw_y','dw_z');
xlabel('Time,sec');

figure('Name','Accel Bias');
hold on; grid on; axis tight;
plot(t,df_*dt,'LineWidth',2);
legend('df_x','df_y','df_z');
xlabel('Time,sec');

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
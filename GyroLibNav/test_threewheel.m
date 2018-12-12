function test_threewheel(Rn_ref, Vb_ref, Euler_ref, dThe_ref, dt)

% test_threewheel(Rn_ref, Vb_ref, Euler_ref, dThe_ref, dt)
% Implements test routine for the simple integrated system of three-wheeled
% robot using the simulated data from the two wheels odometers, azimuth
% gyroscope and "GPS"; 
%
%   Input arguments:
%   Rn_ref    - Reference robot position in navigation frame, m
%   Vb_ref    - Reference robot velocity in body frame, m/s
%   Euler_ref - Reference robot body attitude angles, rad
%   dThe_ref  - Reference data from the gyroscope (angle increments), rad
%   dt   - Computer cycle, sec.

%% Reproducible results
rng(100);

%% Figure
close all;
figure;
view(2);
axis equal;
hold on; grid on;
set(gca,'Xlim',[-60 60]);
set(gca,'Ylim',[-60 60]);
set(gca,'Zlim',[-5 5]);
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
set(gcf,'renderer','opengl');

%% Target axes
at1 = [0; 0; 2.5]*8;
at2 = [0; 2.5; 0]*8;
at3 = [2.5; 0; 0]*8;
st =  [0; 0; 0];
pat1 = plot3([st(1) at1(1)],[st(2) at1(2)],...
    [st(3) at1(3)],'b--','linewidth',0.5);
pat2 = plot3([st(1) at2(1)],[st(2) at2(2)],...
    [st(3) at2(3)],'g--','linewidth',0.5);
pat3 = plot3([st(1) at3(1)],[st(2) at3(2)],...
    [st(3) at3(3)],'r--','linewidth',0.5);

%% INS Axes
ai1 = [0; 0; 1.5;]*7;
ai2 = [0; 1.5; 0;]*7;
ai3 = [1.5; 0; 0;]*7;
sz =  [0; 0; 0;];
pai1 = plot3([sz(1) ai1(1)],[sz(2) ai1(2)],...
    [sz(3) ai1(3)],'b-','linewidth',2);
pai2 = plot3([sz(1) ai2(1)],[sz(2) ai2(2)],...
    [sz(3) ai2(3)],'g-','linewidth',2);
pai3 = plot3([sz(1) ai3(1)],[sz(2) ai3(2)],...
    [sz(3) ai3(3)],'r-','linewidth',2);

%% Cube patch for INS attitude animation
Vert = [ 0 0 0 ; 1 0 0 ; 1 1 0 ; 0 1 0 ; 0 0 1 ; 1 0 1 ; 1 1 1 ; 0 1 1];
Vert(:,1) = (Vert(:,1)-0.5)*5;
Vert(:,2) = (Vert(:,2)-0.5)*5;
Vert(:,3) = (Vert(:,3)-0.5)*5;
Faces = [ 1 2 6 5 ; 2 3 7 6 ; 3 4 8 7 ; 4 1 5 8 ; 1 2 3 4 ; 5 6 7 8 ];
ptch.Vertices = Vert;
ptch.Faces = Faces;
ptch.FaceVertexCData = gray(6);
ptch.FaceColor = 'flat';
patch_handle = patch(ptch);

%% Markers
hu = plot(0,0,'b.','MarkerSize',20);
ht = plot(0,0,'ks','MarkerSize',10);
hm = plot(0,0,'ko','MarkerSize',5,'MarkerFaceColor','r','LineWidth',1);
th = text(-55,-55, 'Time ');
set(th,'FontSize',12,'Color','b');
tail_ref = plot(0,0,'k.-');
tail_est = plot(0,0,'b.-');
tail_mes = plot(0,0,'r.');
tail_length = 60;
cnt_tail = 0;
plg = plot([NaN NaN],[NaN NaN],'k-.','linewidth', 1.0);

%% Logs
Nsim = size(Rn_ref,1); %Number of simulation steps
Rn_       = zeros(Nsim,2);
Vb_       = zeros(Nsim,1);
Euler_    = zeros(Nsim,1);
Euler_err = zeros(Nsim,1);
Rn_err    = zeros(Nsim,2);
Vb_err    = zeros(Nsim,1);
Vx_odo    = zeros(Nsim,1);
Wz_odo    = zeros(Nsim,1);
dlr_      = zeros(Nsim,1);
dll_      = zeros(Nsim,1);
dw_       = zeros(Nsim,1);
Pos_ref   = NaN(Nsim,2);
Pos_est   = NaN(Nsim,2);
Pos_mes   = NaN(Nsim,2);

%% GPS Parameters
ngp = 5e0;
ngv = 1e-1;
lb = [10.0; 10.0; 0.0]; %GPS - odometer lever
gps_rate  = 100;

%% Odometer Parameters
lr_ = 0.3; %right wheel circumference length
ll_ = 0.3; %left  wheel circumference length
d   = 0.4; %wheels axis length
%Odometer errors
err_lr = -2e-2;
err_ll = -1e-2;
err_d  =  0.0;
nodo   =  1e-5;

%% Gyroscope Parameters
nthe = 1e-6; %gyro noise
bthe = 1e-5; %gyro bias
gyro_rate  = 1;

%% Initial State:
%Position and Heading
pos_err = [5.0; -5.0]; %meters
Rn = [
    Rn_ref(1,1)+pos_err(1,1)
    Rn_ref(1,2)+pos_err(2,1)
    ];
he = 30.0; %deg
Cbn = angle_dcm(Euler_ref(1,1), 0, 0)'*angle_dcm(he*pi/180,0,0)';
%Covariance matrix
P=zeros(6);
P(1:2,1:2)=eye(2)*1e-1; %position errors
P(3,3)=1e-2;            %heading error
P(4:5,4:5)=eye(2)*1e-4; %wheel radiuses errors
P(6,6)=1e-4;            %gyro bias  error
%Wheel Errors and Gyroscope Bias Initial Estimates
dlr = 0.0;
dll = 0.0;
dw  = 0.0;

%% EKF Parameters:
pos_noise = 1e-4; %position noise
att_noise = 1e-6; %heading noise
whe_noise = 1e-16; %wheel error noise
bia_noise = 1e-20; %gyroscope bias noise

%% Main loop
for t=1:Nsim
    
    %% Reference:
    Rnref = Rn_ref(t,:)';
    Cbnref = angle_dcm(Euler_ref(t,1), Euler_ref(t,2), Euler_ref(t,3))';
    Vbref = Vb_ref(t,:)';
    Wbref = dThe_ref(t,:)'/dt;
    
    %% Sensors Measurements:
    %Odometer - wheels angular rates
    fir = Vbref(1,1)/(lr_+err_lr)+...
        Wbref(3,1)*(d+err_d)/(2*(lr_+err_lr))+nodo*randn;
    fil = Vbref(1,1)/(ll_+err_ll)-...
        Wbref(3,1)*(d+err_d)/(2*(ll_+err_ll))+nodo*randn;
    %Gyroscope
    dThe = dThe_ref(t,3)+bthe+nthe*randn;
    %GPS
    rg_ref = Rnref+Cbnref*lb;
    rg = rg_ref+ngp*randn(3,1);
    vg_ref = Cbnref*(Vbref+skew(Wbref)*lb);
    vg = vg_ref+ngv*randn(3,1);    
    
    %% Odometer+gyro mechanization
    %compensate odometer errors
    lr  = lr_-dlr; %right wheel length
    ll  = ll_-dll; %left  wheel length
    %Compensate gyroscope error
    dthe = dThe-dw*dt; %angle increment
    Wgz = dthe/dt;     %angular rate
    [Rn, Vb, Cbn, Wb] = odo_gyro_mech(Rn, Cbn, fir, fil, dthe,...
        lr, ll, d, dt);
    
    %% Kalman Predict
    %Error model
    [F,Q] = error_model(fir, fil, Vb, Cbn, dt, ...
        pos_noise, att_noise, whe_noise, bia_noise);
    %Covariance predict
    P = F*P*F'+Q;
    
    %% Kalman Update - "GPS"
    if (mod(t,gps_rate) == 0)
        %measurement vector
        v = zeros(4,1);
        rg_hat = [Rn; 0]+Cbn*lb;
        v(1:2,1) = rg_hat(1:2,1)-rg(1:2,1);
        vb = [Vb;0;0];
        wb = [0;0;Wgz];
        vg_hat = Cbn*(vb+skew(wb)*lb);
        v(3:4,1) = vg_hat(1:2,1)-vg(1:2,1);        
        %measurement matrix
        H = zeros(4,6);
        H(1:2,1:2) = eye(2);
        dpsi = skew(Cbn*lb);
        H(1:2,3) = dpsi(1:2,3);    
        dpsi = skew(vg_hat);
        H(3:4,3) =  dpsi(1:2,3);
        domega   = -Cbn*skew(lb);
        H(3:4,6) = domega(1:2,3);
        H(3,4)   = Cbn(1,1)*fir/2;
        H(3,5)   = Cbn(1,1)*fil/2;
        H(4,4)   = Cbn(2,1)*fir/2;
        H(4,5)   = Cbn(2,1)*fil/2;
        %measurement noise covariance
        R(1:2,1:2) = eye(2)*ngp^2;
        R(3:4,3:4) = eye(2)*ngv^2;
        %Update
        I = eye(6);
        K = (P*H')/(H*P*H'+ R);
        P = (I-K*H)*P*(I-K*H)' + K*R*K';
        x = K*v;
        %Correct Position
        Rn = Rn-x(1:2,1);
        %Correct Attitude
        En  = [1, -x(3,1), 0; x(3,1), 1, 0; 0, 0, 1];
        Cbn = En*Cbn;
        %Normalize Cbn matrix
        Cbn = dcmnormalize(Cbn);
        %Update errors estimates
        dlr = dlr+x(4,1);
        dll = dll+x(5,1);
        dw  = dw +x(6,1);
    end
    
    %% Kalman Update - Gyroscope
    if (mod(t,gyro_rate) == 0)
        %measurement vector
        v = Wgz-Wb;
        %measurement matrix
        H = zeros(1,6);
        H(1,4) = -fir/d;
        H(1,5) =  fil/d;
        H(1,6) =  1;
        %measurement noise covariance
        R = 1e-3;
        %Update
        I = eye(6);
        K = (P*H')/(H*P*H'+ R);
        P = (I-K*H)*P*(I-K*H)' + K*R*K';
        x = K*v;
        %Correct Position
        Rn = Rn-x(1:2,1);
        %Correct Attitude
        En  = [1, -x(3,1), 0; x(3,1), 1, 0; 0, 0, 1];
        Cbn = En*Cbn;
        %Normalize Cbn matrix
        Cbn = dcmnormalize(Cbn);
        %Update errors estimates
        dlr = dlr+x(4,1);
        dll = dll+x(5,1);
        dw  = dw +x(6,1);
    end
    
    %% Collect Logs
    Rn_(t,:) = Rn;
    Vb_(t,:) = Vb;
    [Euler_(t,1),~,~] = dcm_angle(Cbn');
    Vx_odo(t,1) = Vb;
    Wz_odo(t,1) = Wb;
    dlr_(t,:) = dlr;
    dll_(t,:) = dll;
    dw_(t,:)  = dw;
    
    %% Errors
    Cref = angle_dcm(Euler_ref(t,1), 0, 0);
    Cerr = Cref*Cbn;
    [Euler_err(t,1), ~, ~] = ...
        dcm_angle(Cerr);
    Rn_err(t,:) = Rn_ref(t,1:2)-Rn';
    Vb_err(t,:) = Vb_ref(t,1)-Vb;
    
    %% Animations
    if (t < 1000)
        animation_rate = 2;
    else
        animation_rate = 100;
    end
    if(mod(t,animation_rate) == 0)
        cnt_tail = cnt_tail+1;
        
        %% GPS measurement
        if (mod(t,gps_rate) == 0)
            set(hm, 'Xdata',rg(1,1), 'Ydata', rg(2,1));
            Pos_mes(cnt_tail,:) = rg(1:2,1);
        end
        
        %% Reference
        %trajectory
        set(ht, 'Xdata',Rn_ref(t,1), 'Ydata',Rn_ref(t,2));
        set(hu, 'Xdata',rg_ref(1,1), 'Ydata', rg_ref(2,1));
        Pos_ref(cnt_tail,:) = Rn_ref(t,1:2);
        set(plg,'Xdata',[Rn_ref(t,1) rg_ref(1,1)],...
            'Ydata',[Rn_ref(t,2) rg_ref(2,1)]);
        
        %body and body axes
        Cbnref = angle_dcm(Euler_ref(t,1),Euler_ref(t,2),Euler_ref(t,3));
        a1_ = Cbnref'*at1+Rn_ref(t,:)';
        a2_ = Cbnref'*at2+Rn_ref(t,:)';
        a3_ = Cbnref'*at3+Rn_ref(t,:)';
        s_ = Rn_ref(t,:);
        set(pat1,'Xdata',[s_(1) a1_(1)], 'Ydata',[s_(2) a1_(2)],...
            'Zdata',[s_(3) a1_(3)]);
        set(pat2,'Xdata',[s_(1) a2_(1)], 'Ydata',[s_(2) a2_(2)],...
            'Zdata',[s_(3) a2_(3)]);
        set(pat3,'Xdata',[s_(1) a3_(1)], 'Ydata',[s_(2) a3_(2)],...
            'Zdata',[s_(3) a3_(3)]);
        
        %% Estimated
        %body and body axes
        Pos_est(cnt_tail,:) = Rn_(t-1,1:2);
        a1_ = Cbn*ai1+[Rn_(t-1,:)';0];
        a2_ = Cbn*ai2+[Rn_(t-1,:)';0];
        a3_ = Cbn*ai3+[Rn_(t-1,:)';0];
        s_ = [Rn_(t-1,:),0];
        set(pai1,'Xdata',[s_(1) a1_(1)], 'Ydata',[s_(2) a1_(2)],...
            'Zdata',[s_(3) a1_(3)]);
        set(pai2,'Xdata',[s_(1) a2_(1)], 'Ydata',[s_(2) a2_(2)],...
            'Zdata',[s_(3) a2_(3)]);
        set(pai3,'Xdata',[s_(1) a3_(1)], 'Ydata',[s_(2) a3_(2)],...
            'Zdata',[s_(3) a3_(3)]);
        
        %% Cube
        Vert_ = Vert;
        for j=1:size(Vert,1)
            Vert_(j,:) = (Vert(j,:)*Cbn')+[Rn_(t-1,:),0];
        end
        set(patch_handle,'Vertices',Vert_);
        
        %% Tails
        if (cnt_tail > tail_length)
            set(tail_ref,...
                'Xdata',Pos_ref(cnt_tail-tail_length:cnt_tail,1),...
                'Ydata',Pos_ref(cnt_tail-tail_length:cnt_tail,2));
            set(tail_est,...
                'Xdata',Pos_est(cnt_tail-tail_length:cnt_tail,1),...
                'Ydata',Pos_est(cnt_tail-tail_length:cnt_tail,2));
            set(tail_mes,...
                'Xdata',Pos_mes(cnt_tail-tail_length:cnt_tail,1),...
                'Ydata',Pos_mes(cnt_tail-tail_length:cnt_tail,2));
        else
            set(tail_ref,...
                'Xdata',Pos_ref(1:cnt_tail,1),...
                'Ydata',Pos_ref(1:cnt_tail,2));
            set(tail_est,...
                'Xdata',Pos_est(1:cnt_tail,1),...
                'Ydata',Pos_est(1:cnt_tail,2));
            set(tail_mes,...
                'Xdata',Pos_mes(1:cnt_tail,1),...
                'Ydata',Pos_mes(1:cnt_tail,2));
        end
        
        set(th,'String',sprintf('Time %2.1f sec.',t*dt));
        drawnow;
    end
end

%% Plot Results
t = (1:Nsim)*dt;
figure('Name','Position');
subplot(2,1,1);
hold on; grid on; axis tight;
plot(t,Rn_ref(:,1:2),'--','LineWidth',2);
plot(t,Rn_,'LineWidth',2);
legend('X_{ref}','Y_{ref}','X_{est}','Y_{est}');
ylabel('Position, m');
xlabel('Time,sec');
subplot(2,1,2);
hold on; grid on; axis tight;
plot(t,Rn_err(:,1:2),'LineWidth',2);
ylabel('Position error, m');
legend('\delta X','\delta Y');
xlabel('Time,sec');

figure('Name','Velocity');
subplot(2,1,1);
hold on; grid on; axis tight;
plot(t,Vb_ref(:,1),'--','LineWidth',2);
plot(t,Vb_(:,1),'LineWidth',2);
legend('Vx_{ref}','Vx_{est}');
ylabel('Velocity, m/sec');
xlabel('Time,sec');
subplot(2,1,2);
hold on; grid on; axis tight;
plot(t,Vb_err(:,1),'LineWidth',2);
ylabel('Velocity error, m/sec');
legend('\delta Vx');
xlabel('Time,sec');

figure('Name','Attitude');
subplot(2,1,1);
hold on; grid on; axis tight;
plot(t,Euler_ref(:,1),'--','LineWidth',2);
plot(t,Euler_(:,1),'LineWidth',2);
legend('\Psi_{ref}','\Psi_{est}');
ylabel('Heading, rad');
xlabel('Time,sec');
subplot(2,1,2);
hold on; grid on; axis tight;
plot(t,Euler_err(:,1)*180/pi,'LineWidth',2);
ylabel('Heading error, deg');
legend('\delta \Psi');
xlabel('Time,sec');

figure('Name','Odometer');
subplot(3,1,1);
hold on; grid on; axis tight;
plot(t,Vb_ref(:,1),'--','LineWidth',2);
plot(t,Vx_odo,'LineWidth',2);
legend('Vbx_{ref}','Vbx_{est}');
subplot(3,1,2);
hold on; grid on; axis tight;
plot(t,dThe_ref(:,3)/dt,'--','LineWidth',2);
plot(t,Wz_odo,'LineWidth',2);
legend('Wbz_{ref}','Wbz_{est}');
subplot(3,1,3);
hold on; grid on; axis tight;
plot(t,ones(length(t),1)*err_lr,'--','LineWidth',2);
plot(t,ones(length(t),1)*err_ll,'--','LineWidth',2);
plot(t,-dlr_,'LineWidth',2);
plot(t,-dll_,'LineWidth',2);
legend('dlr_{ref}','dll_{ref}','dlr_{est}','dll_{est}');

figure('Name','Gyro');
hold on; grid on; axis tight;
plot(t,ones(length(t),1)*bthe,'--','LineWidth',2);
plot(t,dw_*dt,'LineWidth',2);
legend('bias_{ref}','bias_{est}');
ylabel('Gyro bias, rad/sec');
xlabel('Time,sec');
end

%% Odometer+gyro mechanization
function [Rn, Vb, Cbn, Wb] = odo_gyro_mech(Rn, Cbn, fir, fil, dthe,...
    lr, ll, d, dt)
%Odometer velocities
Vb = 1/2*(fir*lr+fil*ll);
Wb = 1/d*(fir*lr-fil*ll);
%Update Attitude
rot_norm=norm(dthe);
sr_a=1-(rot_norm^2/6)+(rot_norm^4/120);
sr_b=(1/2)-(rot_norm^2/24)+(rot_norm^4/720);
Cbb = [1-sr_b*dthe^2, -sr_a*dthe, 0;  sr_a*dthe,...
    1-sr_b*dthe^2, 0; 0, 0, 1];
Cbn=Cbn*Cbb;
%Update Position
Rn = Rn+Cbn(1:2,1:2)*[Vb;0]*dt;
end

%% Odometer+gyro Error Model
function [F,Q] = error_model(fir, fil, Vb,Cbn,dt,...
    pos_noise, att_noise, whe_noise, bia_noise)
%Continuous-time system parameters
An=zeros(3);
An(1,3) =  Cbn(2,1)*Vb;
An(2,3) = -Cbn(1,1)*Vb;
%discretize An;
Fnd=eye(3)+An*dt+An*An*dt*dt/2;
%Error model - error inputs
Nn = zeros(3,3);
Nn(1,1) =  Cbn(1,1)*fir/2;
Nn(1,2) =  Cbn(1,1)*fil/2;
Nn(2,1) =  Cbn(2,1)*fir/2;
Nn(2,2) =  Cbn(2,1)*fil/2;
Nn(3,3) = -1;
%discrete Q
R = diag([pos_noise, pos_noise, att_noise]);
Qn = Nn*R*Nn';
%Use trapezoidal rule
Qnd=dt/2*(Fnd*Qn+Qn*Fnd');
%System matrix and system covariance matrix
F = zeros(6);
Q = zeros(6);
F(1:3,1:3) = Fnd;
F(4:6,4:6) = diag([1.0, 1.0, 1.0]);
F(1:3,4:6) = Nn*dt;
Q(1:3,1:3) = Qnd;
Qwhe = diag([whe_noise, whe_noise, bia_noise]);
Q(4:6,4:6) = Qwhe;
Q(1:3,4:6)=Nn*Qwhe*dt/2;
Q(4:6,1:3)=Q(1:3,4:6)';
end

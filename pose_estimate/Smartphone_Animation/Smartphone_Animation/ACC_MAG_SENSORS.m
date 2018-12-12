%% CALIBRATION AND ATTITUDE ESTIMATION WITH ACC&MAG SENSORS

%% MAIN
clear all
close all
clc
format long

%% DATA: TYPE 2 (accelerometer, magnetometer and gyroscope)
filename = 'example_rotation_3axes.csv';                      % file 13  -> test 7 -> (myphone)
% filename = 'example_calibration.csv';           % file 20  -> test 1 -> (myphone)
% filename = 'example_rotation_z.csv';    % file 21  -> test 2 -> (myphone)

data = csvread(filename);         % data is a matrix of size [timex9] imported by filename
time = data(:,1);                 % time of the measure                                     [sec]
t = time(:,1)-time(1,1);          % elapsed time since the first measure                    [sec]

acc = zeros(size(time,1),3);      % accelerometer's measure              vect(Ax Ay Az)     [m sec^-2]
mag = zeros(size(time,1),3);      % magnetometer's  measure              vect(Bx By Bz)     [100uT]
gyr = zeros(size(time,1),3);      % gyroscope's     measure              vect(Gx Gy Gz)     [rad sec^-1]

mask_ACC = false(size(time,1),1);
mask_MAG = false(size(time,1),1);
mask_GYR = false(size(time,1),1);

for j=1:size(time,1)
    if (data(j,2)==3)
        acc(j,1:3) = data(j,3:5);
        mask_ACC(j,1) = true;
    elseif (data(j,2)==4)
        gyr(j,1:3) = data(j,3:5);
        mask_GYR(j,1) = true;
    elseif (data(j,10)==5)
        mag(j,1:3) = data(j,3:5)./100;
        mask_MAG(j,1) = true;
    end
    if (data(j,6)==3)
        acc(j,1:3) = data(j,7:9);
        mask_ACC(j,1) = true;
    elseif (data(j,6)==4)
        gyr(j,1:3) = data(j,7:9);
        mask_GYR(j,1) = true;
    elseif (data(j,6)==5)
        mag(j,1:3) = data(j,7:9)./100;
        mask_MAG(j,1) = true;
    end
    if (data(j,10)==3)
        acc(j,1:3) = data(j,11:13);
        mask_ACC(j,1) = true;
    elseif (data(j,10)==4)
        gyr(j,1:3) = data(j,11:13);
        mask_GYR(j,1) = true;
    elseif (data(j,10)==5)
        mag(j,1:3) = data(j,11:13)./100;
        mask_MAG(j,1) = true;
    end
end

mask_DATA = and(and(mask_ACC,mask_MAG),mask_GYR);
acc = acc(mask_DATA,:);
mag = mag(mask_DATA,:);
gyr = gyr(mask_DATA,:);
t = t(mask_DATA);

%% IGRF - DESCRIPTION
% HEIGTH       = Scalar distance, in meters, from the surface of the Earth.
% LATITUDE     = Scalar geodetic latitude, in degrees. North latitude is positive, south latitude is negative.
% LONGITUDE    = Scalar geodetic longitude, in degrees. East longitude is positive, west longitude is negative.
% DECIMAL_YEAR = Scalar year, in decimal format. This value is the desired year ...
%                 ...to include any fraction of the year that has already passed.
% MAG_FIELD_VECTOR          = Magnetic field vector, in nanotesla (nT). Z is the vertical component (+ve down).
% HOR_INTENSITY             = Horizontal intensity, in nanotesla (nT).
% DECLINATION               = Declination "Azimuth", in degrees (+ve east).
% INCLINATION               = Inclination, in degrees (+ve down).
% TOTAL_INTENSITY           = Total intensity, in nanotesla (nT).
% MAG_FIELD_SEC_VARIATION   = Secular variation in magnetic field vector, in nT/year...
%                           ...Z is the vertical component (+ve down).
% SEC_VARIATION_HORIZONTAL  = Secular variation in horizontal intensity, in nT/year.
% SEC_VARIATION_DECLINATION = Secular variation in declination, in minutes/year (+ve east).
% SEC_VARIATION_INCLINATION = Secular variation in inclination, in minutes/year (+ve down).
% SEC_VARIATION_TOTAL       = Secular variation in total intensity, in nT/year.

% syntax (igrf11.magm)
% [mag_field_vector, hor_intensity, declination, inclination, ...
%     total_intensity, mag_field_sec_variation, sec_variation_horizontal, ...
%     sec_variation_declination, sec_variation_inclination, sec_variation_total] = ...
%     igrf11magm(height, latitude, longitude, decimal_year)
% syntax (igrf)
% [BX, BY, BZ] = ...
%     igrf(datenum(year,month,day), latitude, longitude, height, 'geodetic')
% Note: 100 micro-Tesla are 1 Gauss

%% IGRF - INPUT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
height = 1;          % [km]
LAT    = 41.2;       % [deg]
LONG   = 12.89;      % [deg]   
year   = 2013;
month  = 12;
day    = 27;
hour   = 14;
minute = 30;
second = 00;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% WITH "igrf11magm.m" FUNCTION
% decimal_year = decyear(year,month,day);
% [XYZ, H, DEC, DIP, F] = igrf11magm(height, LAT, LONG, decimal_year);
%% WITH "igrf.m" FUNCTION
warning('off','MATLAB:dispatcher:InexactCaseMatch')
date = datenum(year,month,day,hour,minute,second);
[BX, BY, BZ]    = igrf(date, LAT, LONG, height, 'geodetic');    % [nT]
%------------------------------- CONV ------------------------------------%
BX = BX./1e5;     % positive towards the geograpich north       % [100uT]
BY = BY./1e5;     % positive towards the east                   % [100uT]
BZ = BZ./1e5;     % positive towards the interior of the Earth  % [100uT]
%-------------------------------------------------------------------------%
HOR_INTENSITY   = hypot(BX, BY);                                % [100uT]
TOTAL_INTENSITY = hypot(BZ, HOR_INTENSITY);                     % [100uT]
DECLINATION     = atan2(BY, BX);                                % [rad]
INCLINATION     = atan(BZ./hypot(BX, BY));                      % [rad]

%% SYSTEM  Y = H*X
H   = [  -2.*mag(:,1),  mag(:,2).^2,  -2.*mag(:,2),  mag(:,3).^2  -2.*mag(:,3)  ones(length(t),1)  ];
X = -1.*((H'*H)\H')*(mag(:,1).^2);

%% COMPUTATION OF BIAS AND SCALE FACTORS
% k1 = TOTAL_INTENSITY*(1+sfx) <- definition
% k2 = TOTAL_INTENSITY*(1+sfy) <- definition
% k3 = TOTAL_INTENSITY*(1+sfz) <- definition
dBx = X(1);                           % bias x-component smartphone
dBy = X(3)/X(2);                      % bias y-component smartphone
dBz = X(5)/X(4);                      % bias z-component smartphone
k1  = sqrt(dBx^2+dBy^2*X(2)+dBz^2*X(4)-X(6));
k2  = k1/sqrt(X(2));
k3  = k1/sqrt(X(4));
sfx = k1/TOTAL_INTENSITY-1;     % scale factor x-component smartphone
sfy = k2/TOTAL_INTENSITY-1;     % scale factor y-component smartphone
sfz = k3/TOTAL_INTENSITY-1;     % scale factor z-component smartphone

%% PRINT THE RESULTS 
clc
fprintf(' (bias error x-component)    dBx0 = %11.8f [100uT]\n',dBx);
fprintf(' (bias error y-component)    dBy0 = %11.8f [100uT]\n',dBy);
fprintf(' (bias error z-component)    dBz0 = %11.8f [100uT]\n',dBz);
fprintf(' (scale factor x-component)  sfx  = %11.8f \n',sfx);
fprintf(' (scale factor y-component)  sfy  = %11.8f \n',sfy);
fprintf(' (scale factor z-component)  sfz  = %11.8f \n',sfz);

%% CALIBRATION
disp(' ------------------------------------------------------------------ ');
string_cal = 'uncalibrated data';
cal = input(' Would you like to make the calibration? [y] or [n] >> ','s');
if strcmp(cal,'y')
    mag_notcal = mag;
    disp(' Choose the set of parameters with which you desire to make the calibration');
    disp(' 1. Current set');
    disp(' 2. Another set to be load from file "file_calibration"');
    disp(' 3. Another set to be insert from keyboard');
    choose_set = input(' >> ');
    if (choose_set==3)
        clc
        dBx = input(' (bias error x-component)    [100 uT] dBx0 = ');
        dBy = input(' (bias error y-component)    [100 uT] dBy0 = ');
        dBz = input(' (bias error z-component)    [100 uT] dBz0 = ');
        sfx = input(' (scale factor x-component)           sfx  = ');
        sfy = input(' (scale factor y-component)           sfy  = ');
        sfz = input(' (scale factor z-component)           sfz  = ');
    elseif (choose_set==2)
        clc
        file_calibration       = fopen('file_calibration.txt','r+t');
        calibration_parameters = fscanf(file_calibration,'%s %f %s %f %s %f %s %f %s %f %s %f',[5 inf]);
        dBx = calibration_parameters(5,1);
        dBy = calibration_parameters(5,2);
        dBz = calibration_parameters(5,3);
        sfx = calibration_parameters(5,4);
        sfy = calibration_parameters(5,5);
        sfz = calibration_parameters(5,6);
        fclose(file_calibration);
        fprintf(' (bias error x-component)    dBx0 = %11.8f [100uT]\n',dBx);
        fprintf(' (bias error y-component)    dBy0 = %11.8f [100uT]\n',dBy);
        fprintf(' (bias error z-component)    dBz0 = %11.8f [100uT]\n',dBz);
        fprintf(' (scale factor x-component)  sfx  = %11.8f \n',sfx);
        fprintf(' (scale factor y-component)  sfy  = %11.8f \n',sfy);
        fprintf(' (scale factor z-component)  sfz  = %11.8f \n',sfz);
    end
    string_cal = 'calibrated data';
    mag(:,1) = (mag(:,1)-dBx)./(1+sfx);
    mag(:,2) = (mag(:,2)-dBy)./(1+sfy);
    mag(:,3) = (mag(:,3)-dBz)./(1+sfz);
end

%% Storage bias and factor scale for the calibration for the next run
file_calibration = fopen('file_calibration.txt','w+t');
fprintf(file_calibration,['%4s %15.11f\n%4s %15.11f\n%4s %15.11f\n%4s %15.11f\n%4s %15.11f\n%4s %15.11f']...
    ,'dBx=',dBx,'dBy=',dBy,'dBz=',dBz,'sfx=',sfx,'sfy=',sfy,'sfz=',sfz);
fclose(file_calibration);

%% FIGURE

% DATA accelerometer
figure('Name','DATA accelerometer');
title(' DATA accelerometer smartphone');
hold on;
plot(t,acc(:,1),'b','LineWidth',1.2);
plot(t,acc(:,2),'r','LineWidth',1.2);
plot(t,acc(:,3),'g','LineWidth',1.2);
grid on;
axis on;
legend('Gx (width)','Gy (lenght)','Gz (depth)');
xlabel('time [sec]');
ylabel('G [m/s^2]');

% DATA magnetometer
figure('Name',['DATA magnetometer: ',string_cal]);
title(' DATA magnetometer smartphone');
hold on;
plot(t,mag(:,1),'b','LineWidth',1.2);
plot(t,mag(:,2),'r','LineWidth',1.2);
plot(t,mag(:,3),'g','LineWidth',1.2);
grid on;
axis on;
legend('B_X (width)','B_Y (length)','B_Z (depth)');
xlabel('time [sec]');
ylabel('B [100 uT]');

% Magnetic elipsoid
figure('Name',['Magnetic elipsoid: ',string_cal]);
title([' Magnetic elipsoid: ',string_cal]);
plot3(mag(:,1),mag(:,2),mag(:,3),'b-','LineWidth',1.2);
grid on;
axis equal;
xlabel('x-smartphone [100uT]');
ylabel('y-smartphone [100uT]');
zlabel('z-smartphone [100uT]');
view( [1,1,1]);

% Acceleration elipsoid
figure('Name','Acceleration elipsoid');
title(' Acceleration elipsoid ');
plot3(acc(:,1),acc(:,2),acc(:,3),'b-','LineWidth',1.2);
grid on;
axis equal;
xlabel('x-smartphone [m/s^2]');
ylabel('y-smartphone [m/s^2]');
zlabel('z-smartphone [m/s^2]');
view([1,1,1]);

if strcmp(cal,'y')
    % Magnetic elipsoid: calibrated and uncalibrated
    figure('Name','Magnetic elipsoid: comparison');
    plot3(mag(:,1),mag(:,2),mag(:,3),'b-','LineWidth',1.2);
    hold on;
    plot3(mag_notcal(:,1),mag_notcal(:,2),mag_notcal(:,3),'r-','LineWidth',1);
    grid on;
    axis equal;
    xlabel('x-smartphone [100uT]');
    ylabel('y-smartphone [100uT]');
    zlabel('z-smartphone [100uT]');
    legend(' calibrated','uncalibrated');
    view( [1,1,1]);
    % DATA magnetometer: difference among calibrate and uncalibrated
    figure('Name','DATA magnetometer: difference cal and uncal');
    title(' DATA magnetometer smartphone: difference cal and uncal');
    hold on;
    plot(t,mag(:,1)-mag_notcal(:,1),'b','LineWidth',1.2);
    plot(t,mag(:,2)-mag_notcal(:,2),'r','LineWidth',1.2);
    plot(t,mag(:,3)-mag_notcal(:,3),'g','LineWidth',1.2);
    grid on;
    axis on;
    legend('B_X_c_a_l-B_X_u_n_c_a_l (width)','B_Y_c_a_l-B_Y_u_n_c_a_l (length)'...
        ,'B_Z_c_a_l-B_Z_u_n_c_a_l (depth)');
    xlabel('time [sec]');
    ylabel('B [100 uT]');
end
    
%% DELETING INCOMPLETE DATAS AND REARRANGEMENT OF VECTORS
nonull = 0;
%time_  = zeros(size(t,1),1);
for j=1:size(t,1);
    if ((norm(acc(j,:))~=0)&&norm(mag(j,:))~=0)
        nonull = nonull + 1;
        acc(nonull,:) = acc(j,:);
        mag(nonull,:) = mag(j,:);
        time_(nonull) = t(j);
    end
end
% the vectors now have valid components until nonull.
% So mag it's valid if you consider mag(1:nonull,:), because all the valid initial 
% values and nonzero were rearranged and copied one of the first places 
% in the vector mag. You could find some components of the vector which in the 
% end they aren't zero, but they are values initially imported from the file.csv



%% STATEMENT OF VECTORS AND MATRIX FOR FOLLOWING ALGORITHM 
g_loc      = [0, 0, 1]';           % accelerations'   vector   - local coordinates normalized
b_loc      = zeros(3,nonull);      % magnetic field's vector   - local coordinates to normalize
l1_loc     = zeros(3,nonull);      % "TRIAD" vector 1 - local coordinates to normalize
l2_loc     = zeros(3,nonull);      % "TRIAD" vector 2 - local coordinates to normalize
l3_loc     = zeros(3,nonull);      % "TRIAD" vector 3 - local coordinates to normalize
g_body     = zeros(3,nonull);      % accelerations'   vector   - body  coordinates to normalize
b_body     = zeros(3,nonull);      % magnetic field's vector   - body  coordinates to normalize
b1_body    = zeros(3,nonull);      % "TRIAD" vector 1 - body coordinates to normalize
b2_body    = zeros(3,nonull);      % "TRIAD" vector 2 - body coordinates to normalize
b3_body    = zeros(3,nonull);      % "TRIAD" vector 3 - body coordinates to normalize
BODY       = zeros(3,3,nonull);    % matrix of columns b(i)_body (i=1,2,3) at the time t (it's into the nonull index)
LOCAL      = zeros(3,3,nonull);    % matrix of columns l(i)_loc  (i=1,2,3) at the time t (it's into the nonull index)
C          = zeros(3,3,nonull);    % ATTITUDE matrix defined as: B = C*L.
ang_eul321 = zeros(nonull,3);      % Eulero angles set321 (roll pitch yaw for different time)
ang_eul313 = zeros(nonull,3);      % Eulero angles set313 (precession nutation spin for different time)
mainrot    = zeros(nonull,4);      % main rotation's elements (rotation's axis and angle for different time)
quat       = zeros(nonull,4);      % quaternion parameters
 
%% TRIAD ALGORITHM
for iter=1:nonull
    date = datenum(year,month,day,hour,minute,second+time(iter));
    [BX, BY, BZ]    = igrf(date, LAT, LONG, height, 'geodetic');    % [nT]
%------------------------------- CONV ------------------------------------%
BX = BX./1e5;     % positive towards the geograpical north          % [100uT]
BY = BY./1e5;     % positive towards the east                       % [100uT]
BZ = BZ./1e5;     % positive towards the interior of the Earth      % [100uT]
%-------------------------------------------------------------------------%
    %% unit vectors and attitude matrix from TRIAD algorythm
    b_loc(:,iter)   = [BX, BY, BZ]'./norm([BX, BY, BZ]);  % unit vector for local magnetic field given from IGRF
    g_body(:,iter)  = acc(iter,:)'./norm(acc(iter,:));    % unit vector for body accelerations  given from smratphone data      
    b_body(:,iter)  = mag(iter,:)'./norm(mag(iter,:));    % unit vector for body magnetic field given from smratphone data 

    C(:,:,iter)     = triad(g_loc,b_loc(:,iter),g_body(:,iter),b_body(:,iter));  % attitude matrix 

    %% decoding of attitude matrix into different sets
    [phi, theta, psi] = ang_eulero321(C(:,:,iter));   % roll       ,  pitch        ,  yaw                 [decoding]
    [ra, i, om] = ang_eulero313(C(:,:,iter));         % RAAN       ,  inclination  ,  argument of perigee [decoding]
    [axis_E,angle] = mainrotation(C(:,:,iter));       % rotation unit vector,  rotation angle             [decoding]
    [q0, q1, q2, q3]   = quaternion(C(:,:,iter));     % quaternions                                       [decoding]
    ang_eul321(iter,1:3) = [phi theta psi];           % [assignment to the variables]
    ang_eul313(iter,1:3) = [ra i om];                 % [assignment to the variables]
    mainrot(iter,1:4)    = [axis_E angle];            % [assignment to the variables]
    quat(iter,1:4)    = [q0 q1 q2 q3];                % [assignment to the variables]
end
% -------------------------------- CONV --------------------------------- %
ang_eul321   = ang_eul321.*(180/pi);      % phi, theta, psi [deg]
ang_eul313   = ang_eul313.*(180/pi);      % ra,      i,  om [deg]
mainrot(:,4) = mainrot(:,4).*(180/pi);    % mainrot. angle  [deg]
% ----------------------------------------------------------------------- %

%% Figure
% Eulero's angle set(3-2-1)
figure('Name','Eulero''s angles set(3-2-1)');
subplot(3,1,1)
    axis([0 time_(nonull) -181 181])
    plot(time_,ang_eul321(:,1),'b-','LineWidth',1);
    legend(texlabel('phi (roll)'));
    set(gca,'YTick',[-180 -90 0 90 180],'YTickMode','manual')
    title('Eulero''s angles set(3-2-1)');
    ylabel(texlabel('phi [deg]'))
    grid on;
subplot(3,1,2)
    axis([0 time_(nonull) -91 91])
    plot(time_,ang_eul321(:,2),'r-','LineWidth',1);
    legend(texlabel('theta (pitch)'));
    set(gca,'YTick',[-90 -45 0 45 90],'YTickMode','manual')
    ylabel(texlabel('theta [deg]'))
    grid on;
subplot(3,1,3)
    axis([0 time_(nonull) -181 181])
    plot(time_,ang_eul321(:,3),'g-','LineWidth',1);
    legend(texlabel('psi (yaw)'));
    set(gca,'YTick',[-180 -90 0 90 180],'YTickMode','manual')
    ylabel(texlabel('psi [deg]'))
    xlabel('time [s]')
    grid on;

% Eulero's angle set(3-1-3)
figure('Name','Eulero''s angles set(3-1-3)');
subplot(3,1,1)
    axis([0 time_(nonull) -181 181])
    plot(time_,ang_eul313(:,1),'b-','LineWidth',1);
    legend(texlabel('precession'));
    set(gca,'YTick',[-180 -90 0 90 180],'YTickMode','manual')
    title('Eulero''s angles set(3-1-3)');
    ylabel(texlabel('RAAN [deg]'))
    grid on;
subplot(3,1,2)
    axis([0 time_(nonull) -1 181])
    plot(time_,ang_eul313(:,2),'r-','LineWidth',1);
    legend(texlabel('nutation'));
    set(gca,'YTick',[0 45 90 135 180],'YTickMode','manual')
    ylabel(texlabel('i [deg]'))
    grid on;
subplot(3,1,3)
    axis([0 time_(nonull) -181 181])
    plot(time_,ang_eul313(:,3),'g-','LineWidth',1);
    legend(texlabel('spin'));
    set(gca,'YTick',[-180 -90 0 90 180],'YTickMode','manual')
    ylabel(texlabel('omega [deg]'))
    xlabel('time [s]')
    grid on;

% Main rotation's elements
figure('Name','Main rotation''s elements');
subplot(4,1,1)
    axis([0 time_(nonull) -1.01 1.01])
    plot(time_,mainrot(:,1),'b-','LineWidth',1);
    legend('e1');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    title('Main rotation''s elements');
    ylabel('e1')
    grid on;
subplot(4,1,2)
    axis([0 time_(nonull) -1.01 1.01])
    plot(time_,mainrot(:,2),'r-','LineWidth',1);
    legend('e2');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    ylabel('e2')
    grid on;
subplot(4,1,3)
    axis([0 time_(nonull) -1.01 1.01])
    plot(time_,mainrot(:,3),'g-','LineWidth',1);
    legend('e3');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    ylabel('e3')
    grid on;
subplot(4,1,4)
    axis([0 time_(nonull) -181 181])
    plot(time_,mainrot(:,4),'k-','LineWidth',1);
    legend(texlabel('phi'));
    set(gca,'YTick',[-180 -90 0 90 180],'YTickMode','manual')
    xlabel('time [s]')
    ylabel(texlabel('phi [deg]'))
    grid on;

% Quaternions
figure('Name','Quaternions');
subplot(4,1,1)
    axis([0 time_(nonull) -1.01 1.01])
    plot(time_,quat(:,1),'b-','LineWidth',1);
    legend('q0');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    title('Quaternions');
    ylabel('q0')
    grid on;
subplot(4,1,2)
    axis([0 time_(nonull) -1.01 1.01])
    plot(time_,quat(:,2),'r-','LineWidth',1);
    legend('q1');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    ylabel('q1')
    grid on;
subplot(4,1,3)
    axis([0 time_(nonull) -1.01 1.01])
    plot(time_,quat(:,3),'g-','LineWidth',1);
    legend('q2');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    ylabel('q2')
    grid on;
subplot(4,1,4)
    axis([0 time_(nonull) -1.01 1.01])    
    plot(time_,quat(:,4),'k-','LineWidth',1);
    legend('q3');
    set(gca,'YTick',[-1 -0.5 0 0.5 1],'YTickMode','manual')
    ylabel('q3')
    xlabel('time [s]');
    grid on;
    
%% ANIMATION 3D (WORLD EDITOR 3D)

disp(' ');
disp(' If you want only to acquire frames and then play the movie out of Matlab');
disp(' insert "y", otherwise if you want to see the movie also in Matlab insert "n".');
disp(' Instead insert "stop" if you want to stop the process, without to run the animation.');
mov = input(' >> ','s');

if ~(strcmp(mov,'stop'))
    frame = struct('cdata',[],'colormap',[]);
    mainrot(:,4) = mainrot(:,4).*(pi/180);    % mainrot. angle  [rad]
    mondo = vrworld('smartphone.wrl');        % assignment the file.wrl to "mondo" variable
    open(mondo);                              % open the file into the workspace
    % view(mondo);                              % view of the file into new window
    % nodes(mondo,'-full');                     % display the nodes (renamed) and the fields in the command window

    %cel = vrnode(mondo,'cellulare');          % cel holds the nodes of cellulare transform 
    %mynodes = get(mondo,'nodes');             % display the nodes (renamed) in the command window
    %field(mondo,'cellulare');                 % display the fields in the command window

% set(address of the file: vrworld.node1.node2 etc. Between apex it's
% the property, that you want to change, are set and subsequently you puts
% the desired value
% mondo.cellulare.children.geometry.size=[Lx Ly Lz];

    reload(mondo);
    f = vrfigure(mondo); % vrfigure open the window for animation
    h = 1;               % counter for the images (Save it in orderly way)

    for t_time = 1:nonull
    
        if h==1
%             set(f,'NavMode','examine');
            disp(' ');
            disp(' Set the preferences into vrworld window and then insert "return" to continue');
            keyboard            % setting world view into viewer and then digit return in the command window
        end
    
        asse   = [mainrot(t_time,1) mainrot(t_time,2) mainrot(t_time,3)];   % rotation axis
        angolo = mainrot(t_time,4);                                         % rotation angle
        mondo.cellulare.rotation = [asse,angolo];      % equal to "set". This command is more direct
    
        vrdrawnow;              % draw the world
        pause(0.005);           % pause between picture and the other
    
        [Xi] = capture(f);      % capture snapshot image  
 
        if strcmp(mov,'n')
            fig = figure('Name','Frame');
            set(fig,'Position',[535 60 560 467]);
            image(Xi);
            frame(t_time) = getframe(fig);
        end
        h = h+1;
        
        % ".\ it" means that you start from current folder on which you run the script.
        % Subsequently to the slash found, you goes folder to folder.
        % Last slash indicates that the save file will be the one shown. In this case:
        % ".\"           current folder (or D:\....\SN Matlab)
        % "Video"\       folder Video into current folder
        % "smartphone\"  smartphone folder
        % "movie1.tiff"  save file
        name = ['.\Video\smartphone\movie',int2str(h),'.tiff']; 
    
        % imwrite saves the picture Xi into "name" file and it modifies it
        % with the properties shown by desired values.
        imwrite(Xi,name,'tiff','Resolution',300);
    end
    close(mondo);
end

if strcmp(mov,'n')
    fig = figure('Name','movie');
    movie(fig,frame,1,3,[0 0 0 0]);
end
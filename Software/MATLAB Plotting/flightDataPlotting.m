%% DATA INPUT 

clear

load("XYZControl.mat")


dt = 20e-3; 
Time = dt*(1:length(Ma));

uwbEn = Ma(:,60); 

global vertlines

plottingPIDonly =1;
if plottingPIDonly == 1
indexInput = find(uwbEn == 1);  
vertlines = find(diff(indexInput) > 1);
if isempty(vertlines)
vertlines = 1; 
end

else if plottingPIDonly == 2
indexInput = find(uwbEn == 0);  
vertlines = find(diff(indexInput) > 1);
if isempty(vertlines)
vertlines = 1; 
end


else 
    indexInput = 1:length(Ma);
% indexInput = 3730:3770;


    throttle = Ma(:,19);
%vertlines = find(throttle < 200);
vertlines = 1;
clearvars throttle
end 
end

t = Time(indexInput);

PositionError = { ...
    'X (m)', 'Y (m)', 'Z (m)'};% Position measurement X, Y, Z


%(SP = Setpoint, MEAS = Measurement)

% POSITION CONTROL (XY)
% Position Error = sqrt(Position_SP - Position_MEAS) %%% (m)
% Velocity Error = P(Position Error) - V_MEAS) %%% (m/s)
%Axis Acceleration SP = PID(Velocity Error)  %%% (m/s^2)
%Desired Angle = atan(Acceleration SP / Thurst)*180/pi %%%(deg)


% ATTITUDE CONTROL(phi, theta, yaw)
% Angle Error = (ANGLE_SP - ANGLE_MEAS) %%%(deg)
%Gyro Setpoint = PI(Angle Error) *CONST  %%%(deg*CONST) = (deg/s)
%Gyro Error = Gyro Setpoint - Gyro_Measurement
%Motor Commands = PID(Gyro Error) %%%(Non-Dimensional Values in range [-300 300])

% ALTITUDE CONTROL (Z)
% Position Error = sqrt(Position_SP - Position_MEAS) %%% (m)
% Velocity Error = P(Positon Error) - V_MEAS   %%% (m/s)
% Acceleration Error = PI(Velocity Error) - A_MEAS %%% (m/s^2)
% Throttle Command = PID(Acceleration Error) %%% (Non-Dimensional Values in range [-200 200])

% Kt = Relation Between Z Acceleration and Throttle Command = 9.81/Hover Throttle


VelocityX = { ... 
     '$V_{X}$ SP (m/s)', '$V_{X}$ Meas (m/s)', ...
    'Vel $P_{x}$', 'Vel $I_{x}$', 'Vel $D_{x}$'};
VelocityY = {
    '$V_{Y}$ SP (m/s)', '$V_{Y}$ Meas (m/s)', ...
    'Vel $P_{y}$', 'Vel $I_{y}$', 'Vel $D_{y}$'};
Roll = {
    'Roll SP $\phi$ [deg]', 'Roll Meas $\phi$ [deg]', ...
    'Roll P', 'Roll I', ...
    'Roll Gyro SP', 'Roll Gyro Meas', ...
    'Roll Gyro P', 'Roll Gyro I', 'Roll Gyro D'};
Pitch=  {
    'Pitch SP $\theta$ [deg]', 'Pitch Meas $\theta$ [deg]', ...
    'Pitch P', 'Pitch I', ...
    'Pitch Gyro SP', 'Pitch Gyro Meas', ...
    'Pitch Gyro P', 'Pitch Gyro I', 'Pitch Gyro D'};
Yaw = {
    'Yaw SP ${\psi}$ [rad]', 'Yaw Meas $\psi$ [rad]', ...
    'Yaw P', 'Yaw I', ...
    'Yaw Gyro SP', 'Yaw Gyro Meas', ...
    'Yaw Gyro P', 'Yaw Gyro I', 'Yaw Gyro D'};
Height = {
    '$V_{Z}$ SP (m/s)', '$V_{Z}$ Meas (m/s)', ...
    '$A_{Z}$ SP', '$A_{Z}$ Meas', ...
    'Z Accel P', 'Z Accel I', 'Z Accel D', ...
    'PID(Acceleration Error)/Kt', 'Kt', 'Throttle Correction', ... % PID output in units (m/2^2), Kt = 9.81/U_hover, PID output/Eta
     '$\ddot{Z}$', '9.81 - $\ddot{Z}$', 'Eta'}; %Throttle Correction*Kt*Eta (m/s^2), Thrust (m/s^2), Cos(phi)*Cos(theta)  


 %%% PID Output Accel from XY controllers, Measured Axis Acceleration
 %%% (Measured Accel not directly fedback)
XYAccel = { 
    'PID Accel X', 'Measured Accel X', ...
    'PID Accel Y', 'Measured Accel Y'}; 
ArmedEnable = {
    'Armed Status', 'UWB EN'};


% oldInd = indexInput; 
% len = length(oldInd);
% startI = floor(0.35*len);
% clearvars indexInput
% indexInput = startI:startI+400;
% vertlines = 1;

i = length(PositionError);
p = 1;
PError = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(VelocityX);
VelX = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(VelocityY);
VelY = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(Roll);
R = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(Pitch);
P = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(Yaw);
Y = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(Height);
Hei= [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(XYAccel);
XY_A = [Ma(indexInput,p:i)];

p = i + 1;
i = i+length(ArmedEnable);
ArmedEN = [Ma(indexInput,p:i)];
% Ensure the number of headers matches the data's column count


PositionErr = array2table(PError, 'VariableNames', (PositionError));
Vel_X = array2table(VelX, 'VariableNames', (VelocityX));
Vel_Y = array2table(VelY, 'VariableNames', (VelocityY));
R_ = array2table(R, 'VariableNames', (Roll));
P_ = array2table(P, 'VariableNames', (Pitch));
Y_ = array2table(Y, 'VariableNames', (Yaw));
Height_ = array2table(Hei, 'VariableNames', (Height));
XYaccel_ = array2table(XY_A, 'VariableNames', (XYAccel));

% si = measured distance from UWB anchors A,B,C,D,E
% rssi = measured RSSI during distance measurement
si = Ma(indexInput, 64:68); 
Rssi = Ma(indexInput, 69:73);


baropos = Ma(indexInput, 74); % Measured Barometer pressure used during Z error identification 


%% PLOTTING 

close all

% vertlines = Indicates when operator gives control to XYZ Position
% controllers

%Figure
numplot = 8;
ind = 1:numplot;

figure('Name','Roll Axis')

subplot(2,4,1)
plotTableDataWithLimits(PositionErr, 1,  [-1.2 1.2],vertlines,1 );%delta(X)

XSP = 0; 
yline(XSP)
yline(XSP+0.1, '--')
yline(XSP-0.1, '--')


legend('X (m)', 'Setpoint (m)', 'SP + 0.1m', 'SP - 0.1m')
title('X Positon Control Target = (+-) 0.1m of Setpoint')



%xlim([400 800])
subplot(2,4,2)
plotTableDataWithLimits(R_, [1:2], [-20 20],vertlines,7); %angle roll sp, meas
%xlim([400 800])

subplot(2,4,3)
plotTableDataWithLimits(R_, [3:4], [-5 5],vertlines,8); %angle roll PI
%xlim([400 800])

subplot(2,4,4)
plotTableDataWithLimits(R_, [5:6], [-60 60],vertlines,9);%gyro roll sp, meas
%xlim([400 800])

subplot(2,4,5)
plotTableDataWithLimits(R_, [7:9], [-40 40],vertlines,10);%gyro roll PID
%xlim([400 800])

subplot(2,4,6)
plotTableDataWithLimits(Vel_X, [1:2], [-2 2],vertlines,3);%vel X sp meas
%xlim([400 800])

subplot(2,4,7)
plotTableDataWithLimits(Vel_X, [3:5], [-5 5],vertlines,4);%vel X PID
%xlim([400 800])

subplot(2,4,8)
plotTableDataWithLimits(XYaccel_, [1:2], [-2 2],vertlines,21);%PIDAccel,measured (X)
%xlim([400 800])

figure('Name','Pitch Axis')
subplot(2,4,1)
plotTableDataWithLimits(PositionErr, 2, [-0.5 1.5],vertlines,1);%delta(Y)

YSP = 0.4; 
yline(YSP)
yline(YSP+0.1, '--')
yline(YSP-0.1, '--')
legend('Y (m)', 'Setpoint (m)', 'SP + 0.1m', 'SP - 0.1m')
title('Y Positon Control Target = (+-) 0.1m of Setpoint')


subplot(2,4,2)
plotTableDataWithLimits(P_, [1:2], [-20 20],vertlines,11);%angle pitch sp,meas

subplot(2,4,3)
plotTableDataWithLimits(P_, [3:4], [-5 5],vertlines,12);%angle pitch PI
subplot(2,4,4)
plotTableDataWithLimits(P_, [5:6], [-60 60],vertlines,13); %gyro pitch sp,meas
subplot(2,4,5)
plotTableDataWithLimits(P_, [7:9], [-60 60],vertlines,14);%gyro pitch PID
subplot(2,4,6)
plotTableDataWithLimits(Vel_Y, [1:2], [-2 2],vertlines,5);%vel Y sp meas
subplot(2,4,7)
plotTableDataWithLimits(Vel_Y, [3:5], [-5 5],vertlines,6);%vel Y PID
subplot(2,4,8)
plotTableDataWithLimits(XYaccel_, [3:4], [-2 2],vertlines,22);%PIDAccel,measured (Y)


figure('Name','Yaw Axis')
subplot(2,2,1)
plotTableDataWithLimits(Y_, [1:2], [0.5 1.5],vertlines,15);%angle yaw SP, meas
subplot(2,2,2)
plotTableDataWithLimits(Y_, [3:4], [-20 20],vertlines,16);% angle yaw PI
subplot(2,2,3)
plotTableDataWithLimits(Y_, [5:6], [-200 200],vertlines,17); %gyro yaw sp, meas
subplot(2,2,4)
plotTableDataWithLimits(Y_, [7:9], [-50 50],vertlines,18);%gyro yaw PID

figure('Name','Z Axis')
subplot(2,2,1)
plotTableDataWithLimits(Height_, [1:2], [-2 2],vertlines,19);%Vz sp, meas
subplot(2,2,2)
plotTableDataWithLimits(Height_, [3:4], [-5 5],vertlines,20);%Az SP, Meas
subplot(2,2,3)
plotTableDataWithLimits(PositionErr, 3, [0 2.5],vertlines,2);%delta(Z)
ZSP = Ma(indexInput,58);
hold on
plot(ZSP, 'k')
plot(ZSP+0.15, '--k')
plot(ZSP-0.15, '--k')
legend('Z (m)', 'Setpoint (m)', 'SP + 0.15m', 'SP - 0.15m')
title('Height Positon Target Control = (+-) 0.15m of Setpoint')
subplot(2,2,4)
plotTableDataWithLimits(Height_, [5:7], [-4 4],vertlines,2);%delta(Z)



% Kt = 9.81/(throttle required to hover) == Scalar between throttle units and acceleration(m/s/s)
figure(5)
subplot(2,3,1)
plotTableDataWithLimits(Height_, 8, [-200 200],vertlines,2) %Acceleration/Kt  = (unfiltered throttle correction)
subplot(2,3,2)
plotTableDataWithLimits(Height_, 9, [.01 .02],vertlines,2) %Kt 
subplot(2,3,3)
plotTableDataWithLimits(Height_, 10, [-200 200],vertlines,2) %throttle correction to motors
subplot(2,3,4)
plotTableDataWithLimits(Height_, 11, [-5 5],vertlines,2) %z dotdot = unfiltered throttle correction *Kt
subplot(2,3,5)
plotTableDataWithLimits(Height_, 12, [5 13],vertlines,2) % 9.81 - zdotdot
subplot(2,3,6)
plotTableDataWithLimits(Height_, 13, [0.9 1.1],vertlines,2) %total tilt angle = c(phi)*c(theta)



%PID Z acceleration output vs Z acceleration measured
figure  
plotTableDataWithLimits(Height_, [11,4], [-5 5],vertlines,2) %z dotdot 
% 
% figure 
% subplot(2,1,1)
% plotTableDataWithLimits(Vel_Y, [4:4], [-5 5],vertlines,6);%vel Y PID
% subplot(2,1,2)
% plot(Ma(indexInput,61))

% figure
% plot(Ma(indexInput,58))
% xline(vertlines)
% 
% 
% figure
% subplot(2,1,1)
% plot(0.4-PError(:,2))
% xline(vertlines)
% 
% subplot(2,1,2)
% plot(VelY(:,2))
% xline(vertlines)
% 
% figure
% subplot(2,1,1)
% plot(Ma(indexInput,61:62))
% legend('ZVelP', 'ZVelI')
% subplot(2,1,2)
% plot(Ma(indexInput,63))
% legend('position error')

%% UWB DATA   https://ciis.lcsr.jhu.edu/lib/exe/fetch.php?media=courses:446:2016:446-2016-08:algebraicmultilaterationnorrdine.pdf
load("UWB_anchor.mat");
sumanchor2 = sum(anchor.^2,2)'
A  = [ones(numbernodes,1), - 2*(anchor)];
pIA = pinv(A)  


vertlines = [vertlines; length(PError)];

%%
RoomVerticies = [-0.975, -1.47, 0;
    -0.975, -1.47, 2.3;
-0.975, 1.57, 2.3;
-0.975, 1.57, 0;
0.975, 1.57, 2.3;
0.975, 1.57, 0; 
0.975, -1.47, 2.3;
0.975, -1.47, 0; ];

edges = [1 2; 2 3; 3 4; 4 1; % Bottom face
         5 6; 6 8; 8 7; 7 5; % Top face
         3 5; 4 6]; % Vertical edges


for i1 = 1:length(vertlines)
   
    if i1 ==1
    beginInd = [1: vertlines(i1)];
    else if i1 == length(vertlines)
    break;
    else 
    beginInd = [vertlines(i1)+1: vertlines(i1+1)];
    end
    end
figure('Name',['Flight ',num2str(i1)])
hold on 
plot3(PError(beginInd,1), PError(beginInd,2), PError(beginInd,3), 'kx')
plot3(PError(beginInd(1),1), PError(beginInd(1),2), PError(beginInd(1),3), 'bo', 'MarkerSize', 10, 'LineWidth',1)

plot3(0,0.4, ZSP(beginInd) ,'rx', 'MarkerSize', 15, 'LineWidth',2)

legend('Flight Data', 'First Data Point', 'Control Setpoint', 'AutoUpdate','off')


plot3(anchor(:,1),anchor(:,2), anchor(:,3), 'kx', 'MarkerSize', 10)


labels = {'A', 'B', 'C', 'D', 'E'};
for i = 1:length(anchor)
    text(anchor(i,1), anchor(i,2), anchor(i,3),labels{i} ,'VerticalAlignment', 'top', 'HorizontalAlignment', 'left')
end


for i = 1:size(edges, 1)
    % Get the indices of the vertices for the current edge
    idx = edges(i, :);
    % Extract the vertices
    line(RoomVerticies(idx, 1), RoomVerticies(idx, 2), RoomVerticies(idx, 3), 'Color', 'b', 'LineWidth', 0.75);
end

ylim([-1.47, 1.57])
xlim([-1.1, 1.1])
zlim([0, 3])
Pind1 = 2081;
Pind2 = 2841; 

title(['Flight ',num2str(i1)])
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
grid on

% camPos = campos;    % Get camera position [x y z]
% camTarget = camtarget;  % Get camera target [x y z]
% camDir = camTarget - camPos;  % Compute the camera direction vector
% [az, el] = view;  % Get azimuth and elevation angles

camPos =   [  4.5830  -24.6384    8.0085];    % Get camera position [x y z]
camTarget = [  0    0.0500    1.5000];  % Get camera target [x y z]
camDir = camTarget - camPos;  % Compute the camera direction vector
az = 14.3867;
el = 14.5079;


campos(camPos);     % Set camera position
camtarget(camPos + camDir); % Maintain the same direction
view(az, el);       % Apply the same azimuth and elevation angles

end


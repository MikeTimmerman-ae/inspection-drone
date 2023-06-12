clc;
clear;
close all;

s = tf('s');

addpath('control');
addpath('navigation');
addpath('navigation/signals');
addpath('guidance');
addpath('models');
addpath('verification/model/test_inputs/')

%% Drone properties
global param;

param.m = 12.0248;          % kg

Ix = 1.12755;      % kgm^2
Iy = 0.61461;      % kgm^2
Iz = 1.51166;      % kgm^2
param.I = diag([Ix Iy Iz]);

param.g = 9.80665;           % m/s^2
d_cgtop = 0.8175 / sqrt(2);          % m
d_ptop = 0.8175 ;          % m

% coaxial considerations
k_loss = 1.219;
k_counter = 2 / k_loss - 1; 


% NS26x85
kF = 0.00037257456483943437;            % N/(rad/s)^2
kF2 = kF * (1 + k_counter);
kM = 1.1454812375861364e-05;             % Nm/(rad/s)^2
kM2 = kM * (1 - k_counter);


% MF2211
%R_prop = 0.2794;                     % m            
%kF = -0.00019205 / 1.225;            % {N/(rad/s)^2} / {kg/m^3}
%kM = 4.90694039e-06 / 1.225;             % {Nm/(rad/s)^2} / {kg/m^3}

% T-Motor Antigravity MN6007II KV160
% with NS26x85
TO_time = 0.2;              % s
max_ang_vel_motor_actual = 314;              % rad/s (above this motors die)


G_control_allocation = [-kF2 -kF2 -kF2 -kF2;
                        -kF2*d_ptop/2 -kF2*d_ptop/2 kF2*d_ptop/2 kF2*d_ptop/2;
                        kF2*d_ptop/2 -kF2*d_ptop/2 -kF2*d_ptop/2 kF2*d_ptop/2;
                        kM2 -kM2 kM2 -kM2];
G_control_allocation_inv = inv(G_control_allocation);

G_radu = [-kF, -kF*k_counter, -kF, -kF*k_counter, -kF, -kF*k_counter, -kF, -kF*k_counter;
           -kF*d_ptop/2, -kF*d_ptop/2*k_counter, -kF*d_ptop/2, -kF*d_ptop/2*k_counter, kF*d_ptop/2, kF*d_ptop/2*k_counter, kF*d_ptop/2, kF*d_ptop/2*k_counter;  
           kF*d_ptop/2, kF*d_ptop/2*k_counter, -kF*d_ptop/2, -kF*d_ptop/2*k_counter, -kF*d_ptop/2, -kF*d_ptop/2*k_counter, kF*d_ptop/2, kF*d_ptop/2*k_counter;
           kM, -kM*k_counter, -kM, kM*k_counter, kM, -kM*k_counter, -kM, kM*k_counter];

S_x = 147000e-6;            % m^2
S_y = 85000e-6;             % m^2
S_z = 209500e-6;            % m^2

C_D_drone = 0.8;


%% Navigation

param.dt = 0.001;

% IMU
gyro.wn = 190;                              % natural frequency [rad/s]
gyro.damping = 0.707;                       % damping ratio [-]
gyro.noise = 4.5e-3*pi/180/sqrt(gyro.wn);   % noise power [rad/s^2/Hz]
gyro.sat = 250*pi/180;                      % saturation level [rad/s]
gyro.noise = 4.5e-3*pi/180;                 % gyrometer noise [rad/s]
gyro.offset_stability = 10e-3*pi/180;       % gyrometer offset stability [rad/s/C]
gyro.sensitivity_temp = 0.045;              % gyrometer sensitivity/temp [%/C]

acc.wn = 190;                               % natural frequency [rad/s]
acc.damping = 0.707;                        % damping ratio [-]
acc.noise = 100e-6*param.g/sqrt(acc.wn);    % noise power [m/s^2/Hz]
acc.sat = 4*param.g;                        % saturation level m/s^2
acc.noise = 100e-6*param.g;                 % accelerometer noise [m/s^2/sqrt(Hz)]
acc.offset_stability = 0.15e-3*param.g;     % accelerometer offset stability [m/s^2/C]
acc.sensitivity_temp = 0.007;               % acceleromter sensitivity/temp [%/C]

% GPS
longitude = 1.791;
latitude = 53.885;


%% Control

% Feedback control (angular velocity loop)
Kp_p = 2.874;
Kp_q = 2.874;
Kp_r = 1.3536;

sat_p = 50;                        % max commandable p [deg/s]
sat_q = 50;                        % max commandable q [deg/s]
sat_r = 50;                        % max commandable r [deg/s]

% Feedback control (attitude loop)
Kp_roll = 5.6234;
Kp_pitch = 5.6234;
Kp_yaw = 5.6234;

sat_roll = 20;                      % max commandable roll angle [deg]
sat_pitch = 20;                     % max commandable pitch angle [deg]
sat_yaw = 180;                      % max commandable yaw angle [deg]

% Feedback control (velocity loop)
Kp_vx = -0.2723;
Kp_vy = 0.2723;
Kp_vz = 100;
Td = 1/3;
Ti = 5/3;

Dc = (Td*s+1)*(1+1/(Ti*s));

sat_Vx = 10;                     % max commandable velocity [m/s]
sat_Vy = 10;                     % max commandable velocity [m/s]
sat_Vz = 20;                     % max commandable velocity [m/s]

n = 51;
Wn = 0.07;
b = fir1(n,Wn,'low');

% Feedback control (position loop)
Kp_x = 1;
Kp_y = 1;
Kp_z = 1;
myDictionaryObj = Simulink.data.dictionary.open('uavPackageDeliveryDataDict.sldd');
dDataSectObj = getSection(myDictionaryObj,'Design Data');
innerLoopCmdsBus = getValue(getEntry(dDataSectObj, "innerLoopCmdsBus"));
UAVPathManagerBus = getValue(getEntry(dDataSectObj, "uavPathManagerBus"));
baseMission = getValue(getEntry(dDataSectObj, "baseMission"));
copybaseMission = getValue(getEntry(dDataSectObj, "baseMission"));
test_map = load("testing_map.mat", "-mat", "omap3D");


From = [0, 0, 1, 0, 0, 0];
To = [200, 200, 100, 0, 0, 0];


turbine_1_pos = [60, 45, 110, 0, 0, 0];



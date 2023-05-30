clc;
clear;
close all;

s = tf('s');

addpath('control');
addpath('navigation');
addpath('navigation/signals');
addpath('guidance');
addpath('models');

%% Drone properties
global param;
param.m = 11;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
param.I = diag([Ix Iy Iz]);

param.g = 9.81;           % m/s^2
d_cgtop = 0.4389087297;  % m

% NS26x85
% kF = -4.66925492e-4 / 1.225;            % N/(rad/s)^2
% kM = 1.45073741e-5 / 1.225;             % Nm/(rad/s)^2

% MF2211
R_prop = 0.2794;                     % m            
kF = -0.00019205 / 1.225;            % {N/(rad/s)^2} / {kg/m^3}
kM = 4.90694039e-06 / 1.225;             % {Nm/(rad/s)^2} / {kg/m^3}

% T-Motor Antigravity MN6007II KV160
% with MF2211
max_ang_vel_motor = 414;  %for T/W = 2      % rad/s 
max_ang_acc_motor =  5;   % (estimation)    18.7032;        % rad/s^2

k_loss = 1.219;

G = [kF kF kF kF;
    0 kF*d_cgtop 0 -kF*d_cgtop;
    -kF*d_cgtop 0 kF*d_cgtop 0;
    kM -kM kM -kM];

Ginv = inv(G);

G_radu = [1 1 1 1;
          0 d_cgtop 0 -d_cgtop;
          -d_cgtop 0 d_cgtop 0];

S_x = 85000e-6;             % m^2
S_y = 147000e-6;            % m^2
S_z = 209500e-6;            % m^2

C_D_drone = 0.5;


%% Navigation

param.dt = 0.01;

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

sat_roll = 15;                      % max commandable roll angle [deg]
sat_pitch = 15;                     % max commandable pitch angle [deg]
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
sat_Vz = 15;                     % max commandable velocity [m/s]

n = 51;
Wn = 0.07;
b = fir1(n,Wn,'low');

% Feedback control (position loop)
Kp_x = 1;
Kp_y = 1;
Kp_z = 1;

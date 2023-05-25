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
param.m = 10;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
param.I = diag([Ix Iy Iz]);

param.g = 9.81;           % m/s^2
d_cgtop = 0.4389087297;  % m

% NS26x85
% kF = -4.66925492e-4;            % N/(rad/s)^2
% kM = 1.45073741e-5;             % Nm/(rad/s)^2

% MF2211
kF = -0.00019205;            % N/(rad/s)^2
kM = 4.90694039e-06;             % Nm/(rad/s)^2

k_loss = 1.219;

G = [kF kF kF kF;
    0 kF*d_cgtop 0 -kF*d_cgtop;
    -kF*d_cgtop 0 kF*d_cgtop 0;
    kM -kM kM -kM];

Ginv = inv(G);

G_radu = [1 1 1 1;
          0 d_cgtop 0 -d_cgtop;
          -d_cgtop 0 d_cgtop 0];


%% Navigation

param.dt = 0.01;

% IMU
gyro.wn = 190;                              % natural frequency [rad/s]
gyro.damping = 0.707;                       % damping ratio [-]
gyro.noise = 4.5e-3*pi/180/sqrt(gyro.wn);    % noise power [rad/s^2/Hz]
gyro.sat = 250*pi/180;                      % saturation level [rad/s]

acc.wn = 190;                               % natural frequency [rad/s]
acc.damping = 0.707;                        % damping ratio [-]
acc.noise = 100e-6*param.g/sqrt(acc.wn);    % noise power [m/s^2/Hz]
acc.sat = 4*param.g;                        % saturation level m/s^2

% GPS
longitude = 1.791;
latitude = 53.885;


%% Control

% Feedback control
Kp_p = 7.1614;
Kp_q = 7.1614;
Kp_r = 3.3884;

sat_p = 200;                        % max commandable p [deg/s]
sat_q = 200;                        % max commandable q [deg/s]
sat_r = 200;                        % max commandable r [deg/s]

% Feedback control
Kp_roll = 10.84;
Kp_pitch = 10.84;
Kp_yaw = 10.84;

sat_roll = 45;                      % max commandable roll angle [deg]
sat_pitch = 45;                     % max commandable pitch angle [deg]
sat_yaw = 180;                      % max commandable yaw angle [deg]

% Feedback control
Kp_vx = -0.3281;
Kp_vy = 0.3281;
Kp_vz = 100;
Td = 1/3;
Ti = 5/3;

Dc = (Td*s+1)*(1+1/(Ti*s));

sat_Vx = 10;                     % max commandable velocity [m/s]
sat_Vy = 10;                     % max commandable velocity [m/s]
sat_Vz = 15;                     % max commandable velocity [m/s]

% Feedback control
Kp_x = 1;
Kp_y = 1;
Kp_z = 1;

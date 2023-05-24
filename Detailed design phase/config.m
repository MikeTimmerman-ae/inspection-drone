clc;
clear;
close all;

s=tf('s');

addpath('control');
addpath('navigation');
addpath('navigation/signals');
addpath('guidance');

%% Drone properties
global param;
param.m = 10;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
param.I = diag([Ix Iy Iz]);

param.g = 9.81;           % m/s^2
d_cgtop = 0.4389087297;  % m

kF = -4.66925492e-4;            % N/(rad/s)^2
kM = 1.45073741e-5;             % Nm/(rad/s)^2
k_loss = 1.219;

G = [kF kF kF kF;
    0 kF*d_cgtop 0 -kF*d_cgtop;
    -kF*d_cgtop 0 kF*d_cgtop 0;
    kM -kM kM -kM];

Ginv = inv(G);


%% Navigation

param.dt = 0.01;

% GPS
longitude = 1.791;
latitude = 53.885;


%% Control

% Feedback control
Kp_p = 7.1614;
Kp_q = 7.1614;
Kp_r = 3.3884;

% Feedback control
Kp_roll = 10.84;
Kp_pitch = 10.84;
Kp_yaw = 10.84;

% Feedback control
Kp_vx = -0.3281;
Kp_vy = 0.3281;
Kp_vz = 100;
Td = 1/3;
Ti = 5/3;

Dc = (Td*s+1)*(1+1/(Ti*s));

% Feedback control
Kp_x = 1;
Kp_y = 1;
Kp_z = 1;

clc;
clear;
close all;

s=tf('s');

%% Drone poperties
m = 10;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
I = diag([Ix Iy Iz]);

g = 9.81;           % m/s^2
lx = 0.4389087297;  % m
ly = 0.4389087297;  % m

kF = -9.17e-5;             % N/(rad/s)^2
kM = 3.24e-6;             % Nm/(rad/s)^2

G = [kF kF kF kF;
    0 kF*ly 0 -kF*ly;
    -kF*lx 0 kF*lx 0;
    kM -kM kM -kM];

Ginv = inv(G);

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

% Feedback control
Kp_x = 1;
Kp_y = 1;
Kp_z = 1;
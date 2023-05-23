clc;
clear;
close all;

s=tf('s');

%% Drone properties
global param;
param.m = 10;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
param.I = diag([Ix Iy Iz]);

param.g = 9.81;           % m/s^2
lx = 0.4389087297;  % m
ly = 0.4389087297;  % m

kF = -4.66925492e-4;            % N/(rad/s)^2
kM = 1.45073741e-5;             % Nm/(rad/s)^2

G = [kF kF kF kF;
    0 kF*ly 0 -kF*ly;
    -kF*lx 0 kF*lx 0;
    kM -kM kM -kM];

Ginv = inv(G);


%% Navigation

param.dt = 0.01;
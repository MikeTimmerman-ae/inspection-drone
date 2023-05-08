clc;
clear;
close all;

%% Obtain linearized model

dt = 0.01;

sim("plant.slx",[0,10]);

A = plant_Timed_Based_Linearization.a;
B = plant_Timed_Based_Linearization.b;
C = plant_Timed_Based_Linearization.c;
D = plant_Timed_Based_Linearization.d;

sys = ss(A,B,C,D, dt);

%% Linear frequency domain analysis

s=tf('s');

sys1 = tf(d2c(sys(10,1)));

Kp = -2;
alpha = 10;
Ti = 0.3;

Dc = alpha*(Ti*s+1)/(alpha*Ti*s+1);

margin(sys1);

sys_cl = feedback(Kp*sys1, 1);



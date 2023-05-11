clc;
clear;
close all;

s=tf('s');

%% Drone poperties
m = 0.468;          % kg

Ix = 4.856e-3;      % kgm^2
Iy = 4.856e-3;      % kgm^2
Iz = 8.801e-3;      % kgm^2
I = diag([Ix Iy Iz]);

g = 9.81;           % m/s^2
lx = 0.225;         % m
ly = 0.225;         % m

%% Obtain linearized model

sim("plant.slx",[0,10]);

% Continuous-time model
A = plant_Timed_Based_Linearization.a;
B = plant_Timed_Based_Linearization.b;
C = plant_Timed_Based_Linearization.c;
D = plant_Timed_Based_Linearization.d;

sys = ss(A,B,C,D);

%% Table with transfer functions

for i=1:12
    for j=1:4
        tf(sys(i,j)) 
    end
end


%% Linear feedback control angular verlocity (inner loop)

sim("plant_level_1.slx",[0,10]);

% Continuous-time model
A = plant_level_1_Timed_Based_Linearization.a;
B = plant_level_1_Timed_Based_Linearization.b;
C = plant_level_1_Timed_Based_Linearization.c;
D = plant_level_1_Timed_Based_Linearization.d;

sys_level_1 = ss(A,B,C,D);

% Feedback control
Kp_p = 0.4858;
Kp_q = 0.4858;
Kp_r = 0.881;

sys_p = tf(Kp_p*sys_level_1(1,1));
sys_p_cl = feedback(sys_p, 1);

sys_q = tf(Kp_q*sys_level_1(2,2));
sys_q_cl = feedback(sys_q, 1);

sys_r = tf(Kp_r*sys_level_1(3,3));
sys_r_cl = feedback(sys_r, 1);

% Control implementation
omega_ref = [1, 1, 1];            % deg/s

%% Linear feedback control attitude angles

sim("plant_level_2.slx",[0,10]);

% Continuous-time model
A = plant_level_2_Timed_Based_Linearization.a;
B = plant_level_2_Timed_Based_Linearization.b;
C = plant_level_2_Timed_Based_Linearization.c;
D = plant_level_2_Timed_Based_Linearization.d;

sys_level_2 = ss(A,B,C,D);

% Feedback control
Kp_roll = 14;
Kp_pitch = 14;
Kp_yaw = 12;

sys_roll = tf(Kp_roll*sys_level_2(1,1));
sys_roll_cl = feedback(sys_roll, 1);

sys_pitch = tf(Kp_pitch*sys_level_2(2,2));
sys_pitch_cl = feedback(sys_pitch, 1);

sys_yaw = tf(Kp_yaw*sys_level_2(3,3));
sys_yaw_cl = feedback(sys_yaw, 1);

% Control implementation

attitude_ref = [15, 10, 0];            % deg/s

%% Linear feedback control velocity angles

sim("plant_level_3.slx",[0,10]);

% Continuous-time model
A = plant_level_3_Timed_Based_Linearization.a;
B = plant_level_3_Timed_Based_Linearization.b;
C = plant_level_3_Timed_Based_Linearization.c;
D = plant_level_3_Timed_Based_Linearization.d;

sys_level_3 = ss(A,B,C,D);

% Feedback control
Kp_vx = -0.3308;
Kp_vy = 0.3308;
Kp_vz = 4.677;
Td = 1/3;
Ti = 5/3;

Dc = (Td*s+1)*(1+1/(Ti*s));

sys_vx = minreal(tf(Kp_vx*Dc*sys_level_3(1,2)));
sys_vx_cl = feedback(sys_vx, 1);

sys_vy = minreal(tf(Kp_vy*Dc*sys_level_3(2,1)));
sys_vy_cl = feedback(sys_vy, 1);

sys_vz = tf(Kp_vz*sys_level_3(3,3));
sys_vz_cl = feedback(sys_vz, 1);

% Control implementation

velocity_ref = [0.5, 0.5, 0];            % m/s

%% Linear feedback control position

sim("plant_level_4.slx",[0,10]);

% Continuous-time model
A = plant_level_4_Timed_Based_Linearization.a;
B = plant_level_4_Timed_Based_Linearization.b;
C = plant_level_4_Timed_Based_Linearization.c;
D = plant_level_4_Timed_Based_Linearization.d;

sys_level_4 = ss(A,B,C,D);

% Feedback control
Kp_x = 1;
Kp_y = 1;
Kp_z = 10;

sys_x = minreal(tf(Kp_x*sys_level_4(1,1)));
sys_x_cl = feedback(sys_x, 1);

sys_y = minreal(tf(Kp_y*sys_level_4(2,2)));
sys_y_cl = feedback(sys_vy, 1);

sys_z = tf(Kp_z*sys_level_4(3,3));
sys_z_cl = feedback(sys_z, 1);

% Control implementation

position_ref = [0, 3, 0];            % m

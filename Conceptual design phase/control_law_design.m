clc;
clear;
close all;

s=tf('s');

%% Drone properties
m = 12.0248;          % kg

Ix = 1.12755;      % kgm^2
Iy = 0.61461;      % kgm^2
Iz = 1.51166;      % kgm^2
I = diag([Ix Iy Iz]);

g = 9.80665;           % m/s^2
lx = 0.8175 / sqrt(2);  % m
ly = 0.8175 / sqrt(2);  % m

kF = -9.17e-5;             % N/(rad/s)^2
kM = 3.24e-6;             % Nm/(rad/s)^2

G = [kF kF kF kF;
    0 kF*ly 0 -kF*ly;
    -kF*lx 0 kF*lx 0;
    kM -kM kM -kM];

Ginv = inv(G);

%% Obtain linearized model

sim("plant_level_0.slx",[0,10]);

% Continuous-time model
A = plant_level_0_Timed_Based_Linearization.a;
B = plant_level_0_Timed_Based_Linearization.b;
C = plant_level_0_Timed_Based_Linearization.c;
D = plant_level_0_Timed_Based_Linearization.d;

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
Kp_p = 11.275;
Kp_q = 6.1660;
Kp_r = 15.136;

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
Kp_roll = 2.5734;
Kp_pitch = 2.5734;
Kp_yaw = 2.5734;

sys_roll = tf(Kp_roll*sys_level_2(1,1));
sys_roll_cl = feedback(sys_roll, 1);

sys_pitch = tf(Kp_pitch*sys_level_2(2,2));
sys_pitch_cl = feedback(sys_pitch, 1);

sys_yaw = tf(Kp_yaw*sys_level_2(3,3));
sys_yaw_cl = feedback(sys_yaw, 1);

% Control implementation

attitude_ref = [20, 20, 0];            % deg/s

%% Attitude feedback control position

sim("plant_level_5.slx",[0,10]);

% Continuous-time model
A = plant_level_5_Timed_Based_Linearization.a;
B = plant_level_5_Timed_Based_Linearization.b;
C = plant_level_5_Timed_Based_Linearization.c;
D = plant_level_5_Timed_Based_Linearization.d;

A_full = [A(1:6,1:6) zeros(6,3);
         -C(:,1:6) zeros(3,3)];
B_full = [B(1:6,:); zeros(3,3)];

sys_level_5 = ss(A_full, B_full, [eye(3) zeros(3,6)], zeros(3,3));

Q = diag([1 1 1 1 1 1 10 10 10]);
R = 1e-2*diag([1 1 1]);
K = lqr(A_full, B_full, Q, R);

sys_level_5_cl = ss(A_full-B_full*K, [zeros(6,3); eye(3)], [eye(3) zeros(3,6)], zeros(3));

%% plot attitude loop

t = 0:0.001:20;
roll_ref = 0.087266*[zeros(1,1000) ones(1,4001) zeros(1,15000)];
pitch_ref = 0.087266*[zeros(1,7000) ones(1,4001) zeros(1,9000)];
yaw_ref = 0.087266*[zeros(1,13000) ones(1,4001) zeros(1,3000)];

[y_PID_roll, ~, ~] = lsim(sys_roll_cl,roll_ref,t);
[y_PID_pitch, ~, ~] = lsim(sys_pitch_cl,pitch_ref,t);
[y_PID_yaw, ~, ~] = lsim(sys_yaw_cl,yaw_ref,t);

[y_FULL, ~, ~] = lsim(sys_level_5_cl,[roll_ref; pitch_ref; yaw_ref],t);

figure()
subplot(3,1,1)
plot(t, y_PID_roll, t, y_FULL(:,1), t, roll_ref);

subplot(3,1,2)
plot(t, y_PID_pitch, t, y_FULL(:,2), t, pitch_ref);

subplot(3,1,3)
plot(t, y_PID_yaw, t, y_FULL(:,3), t, yaw_ref);

%% Linear feedback control velocity angles

sim("plant_level_3.slx",[0,10]);

% Continuous-time model
A = plant_level_3_Timed_Based_Linearization.a;
B = plant_level_3_Timed_Based_Linearization.b;
C = plant_level_3_Timed_Based_Linearization.c;
D = plant_level_3_Timed_Based_Linearization.d;

sys_level_3 = ss(A,B,C,D);

% Feedback control
Kp_vx = -0.05623;
Kp_vy = 0.05623;
Kp_vz = 6.0256;
Td = 2/1;
Ti = 10/1;

Dc = (Td*s+1)*(1+1/(Ti*s));

sys_vx = minreal(tf(Kp_vx*Dc*sys_level_3(1,2)));
sys_vx_cl = feedback(sys_vx, 1);

sys_vy = minreal(tf(Kp_vy*Dc*sys_level_3(2,1)));
sys_vy_cl = feedback(sys_vy, 1);

sys_vz = tf(Kp_vz*sys_level_3(3,3));
sys_vz_cl = feedback(sys_vz, 1);

% Control implementation

velocity_ref = [0.5, 0.5, -0.5];            % m/s

%% Linear feedback control position

sim("plant_level_4.slx",[0,10]);

% Continuous-time model
A = plant_level_4_Timed_Based_Linearization.a;
B = plant_level_4_Timed_Based_Linearization.b;
C = plant_level_4_Timed_Based_Linearization.c;
D = plant_level_4_Timed_Based_Linearization.d;

sys_level_4 = ss(A,B,C,D);

% Feedback control
Kp_x = 0.83272;
Kp_y = 0.83272;
Kp_z = 0.5290;

sys_x = minreal(tf(Kp_x*sys_level_4(1,1)));
sys_x_cl = feedback(sys_x, 1);

sys_y = minreal(tf(Kp_y*sys_level_4(2,2)));
sys_y_cl = feedback(sys_vy, 1);

sys_z = tf(Kp_z*sys_level_4(3,3));
sys_z_cl = feedback(sys_z, 1);

% Control implementation

position_ref = [1, 0, 0];            % m

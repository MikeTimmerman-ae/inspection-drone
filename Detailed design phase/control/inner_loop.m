%% Create inner control loop using LQR+PI

% Drone poperties
m = 10;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
I = diag([Ix Iy Iz]);

g = 9.81;           % m/s^2

% State-space model inner loop

np = 6;
mp = 3;
pp = 3;
nref = 3;

sim('plant_level_inner', [0, 10]);

Ap = plant_level_inner_Timed_Based_Linearization.a;
Bp = plant_level_inner_Timed_Based_Linearization.b;
Cp = plant_level_inner_Timed_Based_Linearization.c;
Dp = plant_level_inner_Timed_Based_Linearization.d;

sys_p = ss(Ap(1:np, 1:np), Bp(1:np, :), Cp(:, 1:np), Dp);


% Reference model

tau_roll = 0.1;
tau_pitch = 0.1;
tau_yaw = 0.1;

A_ref = [-1/tau_roll 0 0;
         0 -1/tau_pitch 0;
         0 0 -1/tau_yaw];
B_ref = [1/tau_roll 0 0;
         0 1/tau_pitch 0;
         0 0 1/tau_yaw];
C_ref = eye(3);
D_ref = zeros(3);

sys_ref = ss(A_ref, B_ref, C_ref, D_ref);


% Open loop dynamics

A = [sys_p.A zeros(np,nref);
    zeros(nref,np) sys_ref.A];
B = [sys_p.B;
    zeros(mp)];
B_r = [zeros(nref);
      sys_ref.B];
C = [sys_p.C -sys_ref.C];
D = zeros(pp, mp);

sys = ss(A, B, C, D);


% Extended dynamics

A_tilde = [zeros(nref) sys.C;
            zeros(np+nref,nref) sys.A];
B_tilde = [zeros(nref, mp);
            sys.B];
B_tilde_r = [zeros(nref);
             B_r];
C_tilde = [zeros(nref) sys.C];

D_tilde = zeros(mp);

sys_ext = ss(A_tilde, B_tilde, C_tilde, D_tilde);


% Closed-loop

Q = diag([10 10 10 1 1 1 1 1 1 1 1 1]);
R = 1e-3*eye(3);

[K,S,P] = lqr(sys_ext,Q,R);

KI = K(:,1:3);
Kp_x = K(:,4:9);
Kp_xref = K(:,10:12);



%% Create inner control loop using LQR+PI

% Drone poperties
m = 10;          % kg

Ix = 0.287503;      % kgm^2
Iy = 0.287503;      % kgm^2
Iz = 0.135532;      % kgm^2
I = diag([Ix Iy Iz]);

g = 9.81;           % m/s^2

% State-space model inner loop

np = 12;
mp = 4;
pp = 4;
nref = 4;

sim('plant_level_full', [0, 10]);

Ap = plant_level_full_Timed_Based_Linearization.a;
Bp = plant_level_full_Timed_Based_Linearization.b;
Cp = plant_level_full_Timed_Based_Linearization.c;
Dp = plant_level_full_Timed_Based_Linearization.d;

sys_p = ss(Ap, Bp, Cp([1:3,9],:), zeros(nref, mp));


% Reference model

tau_x = 1;
tau_y = 1;
tau_z = 0.1;
tau_yaw = 1;

A_ref = [-1/tau_x 0 0 0;
         0 -1/tau_y 0 0;
         0 0 -1/tau_z 0;
         0 0 0 -1/tau_yaw];
B_ref = [1/tau_x 0 0 0;
         0 1/tau_y 0 0;
         0 0 1/tau_z 0;
         0 0 0 1/tau_yaw];
C_ref = eye(nref);
D_ref = zeros(nref);

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

Q = diag([ones(1,4) ones(1,4) ones(1,12)]);
R = 1e-3*eye(mp);

[K,S,P] = lqr(sys_ext,Q,R);

KI = K(:,1:nref);
Kp_x = K(:,nref+1:np+nref);
Kp_xref = K(:,np+nref+1:end);
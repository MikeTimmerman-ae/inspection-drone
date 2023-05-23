clc;
clear;
close all;

%% LQR+PI control

% State-space system
b = 13.36;
V = 125;
mu_b = 15.5;
Kx2 = 0.012;
Kz2 = 0.037;
Kxz = 0.002;

Cnb_dot = -0.15;
Cyb = -0.9896;
Cyp = -0.087;
Cyr = 0.43;
Clb = -0.0772;
Clp = -0.3444;
Clr = 0.28;
Cnb = 0.1638;
Cnp = -0.0108;
Cnr = -0.1930;

Cy_da = 0;
Cy_dr = 0.21;
Cl_da = -0.35;
Cl_dr = 0;
Cn_da = -0.11;
Cn_dr = -0.05;

P = b/V*[-2*mu_b 0 0;
        0 -4*mu_b*Kx2 4*mu_b*Kxz;
        Cnb_dot 4*mu_b*Kxz -4*mu_b*Kz2];
Q = [-Cyb -Cyp -(Cyr-4*mu_b);
    -Clb -Clp -Clr;
    -Cnb -Cnp -Cnr];
R = [-Cy_da -Cy_dr;
    -Cl_da -Cl_dr;
    -Cn_da -Cn_dr];

Ap = inv(P)*Q;
Bp = inv(P)*R;
Cp = [0 1 0;
      1 0 0];
Dp = zeros(2,2);

sys_p = ss(Ap, Bp, Cp, Dp);

% Reference model

tau_p = 0.01;
tau_b = 0.01;

A_ref = [-1/tau_p 0;
     0 -1/tau_b];
B_ref = [1/tau_p 0
     0 1/tau_b];
C_ref = [1 0;
     0 1];
D_ref = zeros(2,2);

sys_ref = ss(A_ref, B_ref, C_ref, D_ref);

% Open loop dynamics

A = [sys_p.A zeros(3,2);
    zeros(2,3) sys_ref.A];
B = [sys_p.B;
    zeros(2,2)];
B_r = [zeros(3,2);
      sys_ref.B];
C = [sys_p.C -sys_ref.C];
D = zeros(2,2);

sys = ss(A, B, C, D);


% Extended dynamics

A_tilde = [zeros(2,2) sys.C;
            zeros(5,2) sys.A];
B_tilde = [zeros(2,2);
            sys.B];
B_tilde_r = [zeros(2,2);
             B_r];
C_tilde = [zeros(2,2) sys.C];

D_tilde = zeros(2,2);

sys_ext = ss(A_tilde, B_tilde, C_tilde, D_tilde);


% Closed-loop

Q = eye(7);
R = 1e-6*eye(2);

[K,S,P] = lqr(sys_ext,Q,R);

KI = K(:,1:2);
Kp_x = K(:,3:5);
Kp_xref = K(:,6:7);


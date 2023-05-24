function x = stateTransition(x, u)
%STATETRANSITION Summary of this function goes here
    m = 10;
    g = [0; 0; 9.81];
    dt = 0.01;
    I = diag([0.287503 0.287503 0.135532]);
    
    x_prev = x;

%   States
    phi = x_prev(7);
    theta = x_prev(8);
    psi = x_prev(9);
    
    omega = x_prev(10:12);

%   Inputs
    F = [0; 0; u(1)];
    M = u(2:4);

%   State derivative
    x_dot = zeros(12,1);
    
    % Angular motion
    x_dot(7:9) = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                  0 cos(phi) -sin(phi);
                  0 sin(phi)*sec(theta) cos(phi)*sec(theta)]*omega;
    x_dot(10:12) = inv(I)*(M - cross(omega, I*omega));

    % Linear motion
    x_dot(1:3) = x_prev(4:6);        % p_e_dot = V_e
    
    Reb = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
        cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
        -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];

    x_dot(4:6) = Reb*F/m + g;

%   Next state
   x = x_prev + dt*x_dot;

end



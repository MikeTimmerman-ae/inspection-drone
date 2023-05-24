function y = gyrometer_model(x)
%IMU_MODEL Gyrometer measurements
%   Output angular velocity
    y = diag([zeros(1,9) ones(1,3)])*x;
end

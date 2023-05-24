function y = gyrometer_model(x)
%IMU_MODEL Gyrometer measurements
%   Output angular velocity
    y = [zeros(3,9) eye(3)]*x;
end

function y = accelerometer_model(x)
%IMU_MODEL Accelerometer measurements
    disp(x);
    phi = x(4);
    theta = x(5);
    
%   Output proper acceleration
    y = [sin(theta);
        -cos(theta)*sin(phi);
        cos(phi)*cos(theta)]*param.g;
end

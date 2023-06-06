function y = accelerometer_model(x)
%IMU_MODEL Accelerometer measurements
%     g = 9.81;
% 
%     phi = x(4);
%     theta = x(5);
% 
% %   Output proper acceleration
%     y = [sin(theta);
%         -cos(theta)*sin(phi);
%         cos(phi)*cos(theta)]*g;
    y = x(7:9);
end

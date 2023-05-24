function y = GPS_model(x)
%GPS_model GPS measurements
%   Specify output vector
    y = zeros(6,1);
    
    % Position measurement
    y(1:3) = [eye(3) zeros(3,9)]*x;

    % Velocity measurement
    y(4:6) = [zeros(3) eye(3) zeros(3,6)]*x;
end


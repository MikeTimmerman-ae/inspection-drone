function y = GPS_model(x)
%GPS_model GPS measurements
%   Specify output vector
    y = zeros(6,1);
    
    % Position measurement
    y(1:3) = diag([ones(1,3) zeros(1,9)])*x;

    % Velocity measurement
    y(4:6) = diag([zeros(1,3) ones(1,3) zeros(1,6)])*x;
end


% function waypoint_struct = generating_waypoints(From, To)
% 
% persistent outWPS
% 
% if isempty(outWPS)
%     outWPS = single(zeros(2,4));
% end


test = out.PoseOut;
hold on
plot3(test(:, 1), test(:, 2), -test(:, 3), "b");
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on



% axis([-100 100 -100 100])

inspectionStatesSpaced = inspection_generator((1/3)*pi, 120, 80);
disp(inspectionStatesSpaced)
b = [[0 0 0 0]; inspectionStatesSpaced];
disp(b)
plot3(b(:, 1), b(:, 2), -b(:, 3), "r")
% axis([-100 100 -100 100])
legend("Generated Path", "Simulated Path")
hold off
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')

    
    % plot3(test(:, 1), test(:, 2), -test(:, 3));
    % xlabel('x [m]')
    % ylabel('y [m]')
    % zlabel('z [m]')
    % grid on

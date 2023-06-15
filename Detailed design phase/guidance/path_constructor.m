
% function waypoint_struct = generating_waypoints(From, To)
% 
% persistent outWPS
% 
% if isempty(outWPS)
%     outWPS = single(zeros(2,4));
% end

% 
rng(100,"twister");
From = [0 0];
To = [800 800];

targets = [[1, 1];[300, 230];[700, 230];[700, 730];[300, 730];];
targets2 = [[0, 0, 0];[300, 230, 0];[700, 230, 0];[700, 730, 0];[300, 730, 0];];
% disp(size(targets, 1));
%%%%%%%%%%%
test_map = load("testing_map2.mat");
%whos -file testing_map.mat omap3D

Data = test_map.omap2D;
omap2 = test_map.omap2D;
% Consider unknown spaces to be unoccupied
omap2.FreeThreshold = omap2.OccupiedThreshold;
omap = occupancyMap(Data,1);
omap.FreeThreshold = Data.FreeThreshold;
omap.OccupiedThreshold = Data.OccupiedThreshold;
% disp(omap)
% show(omap)


% Define start and end position

waypoints_tot = [];

sum_tot = 15;
for i = 2:size(targets,1)
   
    [waypoints, sum_wp] = wp(targets(i-1, :), targets(i, :), omap);
    sum_tot = sum_tot + sum_wp;
    
    waypoints_tot = cat(1, waypoints_tot, waypoints(2:size(waypoints, 1), :));
    % states_tot = cat(1, states_tot, states(2:size(states, 1), :));
end
disp(sum_tot);
    % waypoints_tot = [waypoints_tot, waypoints];

% waypoints_tot = [[0, 0]; waypoints_tot];
waypoints_tot_new = [0, 0];
for i = 1:size(waypoints_tot, 1)
    if mod(i, 10) == 0
        % disp(waypoints_tot(i, :));
        waypoints_tot_new = cat(1, waypoints_tot_new, waypoints_tot(i, :));
    end
end
% disp(size(waypoints_tot_new, 1));
% disp(states_tot);
% disp(waypointsTest);
angle = (1/3)*pi;
nh = 120;
bl = 82;
[inspectionStates, sum] = inspection_generator(angle, nh, bl);
disp(sum);
inspection_struct = struct('mode',[],'position',[], 'params',[]);
inspection_struct = repmat(inspection_struct,size(inspectionStates, 1), 1);
inspection_struct(1).mode = uint8(1) ;
inspection_struct(1).position = single([0;0;-110]);
inspection_struct(1).params = single([0;0;0;0]);
for j = 2:size(inspectionStates, 1)
    inspection_struct(j).mode = uint8(6) ;
    inspection_struct(j).position = single([inspectionStates(j-1, 1);inspectionStates(j-1, 2);inspectionStates(j-1, 3)]);
    inspection_struct(j).params = single([0;0;0;inspectionStates(j-1, 4)]);
end

number_of_points = size(waypoints_tot_new, 1);

mode_vector = zeros(number_of_points, 1)+2;
% mode_vector(1, 1) =  1;
mode_vector(number_of_points, 1) = 4;




waypoint_struct = struct('mode',[],'position',[], 'params',[]);
waypoint_struct = repmat(waypoint_struct,number_of_points, 1);

waypoint_struct(1).mode = uint8(1) ;
waypoint_struct(1).position = single([0;0;-10]);
waypoint_struct(1).params = single([0;0;0;0]);
waypoint_struct(number_of_points).mode = uint8(7) ;
waypoint_struct(number_of_points).position = single([waypoints_tot_new(number_of_points,1);waypoints_tot_new(number_of_points,2);-15]);
waypoint_struct(number_of_points).params = single([-1;-1;-1;-1]);



for i = 2:number_of_points-1
    waypoint_struct(i).mode = uint8(mode_vector(i-1,1)) ;
    waypoint_struct(i).position = single([waypoints_tot_new(i-1,1);waypoints_tot_new(i-1,2);-15]);
    
    % u = [waypoints_tot(i, 1) - waypoints_tot(i-1, 1); waypoints_tot(i, 2) - waypoints_tot(i-1, 2)];
    % v = [waypoints_tot(i, 1) - waypoints_tot(i-1, 1); 0];
    % dotUV = dot(u, v);
    % normU = norm(u);
    % normV = norm(v);
    % 
    % theta = acos(dotUV/(normU * normV));
    % 
    % disp(theta)
    waypoint_struct(i).params = single([0;0;0;0]);
end

% display(waypoint_struct);

assignin('base',"waypoint_struct", waypoint_struct);
assignin('base',"inspection_struct", inspection_struct);
% waypoint_struct= outWPS;

% show(omap)
% hold on
% scatter(targets(1,1),targets(1,2),30,".r")
% scatter(targets(size(targets, 1),1),targets(size(targets, 1),2),30,".g")

% Plot the waypoints
z_zeros = zeros(size(waypoints_tot_new(:,1)));
z_zeros = z_zeros(:) +15;
hold on
% plot3(waypoints_tot_new(:,1),waypoints_tot_new(:,2), z_zeros, "r")
inspectionStates = [[0 0 0 0]; inspectionStates];
plot3(inspectionStates(:,1),inspectionStates(:,2), -inspectionStates(:,3), "r")
% scatter(targets(1,1),targets(1,2),30,".r")
% scatter(targets(size(targets, 1),1),targets(size(targets, 1),2),30,".g")
% plot(states_tot(:,1),states_tot(:,2), "-y")

% view([-31 63])
test_ = out.PoseOut;
plot3(test_(:, 1), test_(:, 2), -test_(:, 3), "b");
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis([-100 100 -100 100])
legend("Generated Path", "Simulated Path")
grid on
hold off


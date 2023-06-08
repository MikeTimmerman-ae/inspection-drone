
function waypoint_struct = generating_waypoints(From, To)

persistent outWPS

if isempty(outWPS)
    outWPS = single(zeros(2,4));
end

% 
rng(100,"twister");
From = [0 0 0];
To = [500 500 0];

targets = [[0, 0, 0];[300, 230, 0];[700, 230,  0];[700, 730,  0];[300, 730,  0];];
disp(size(targets, 1));
%%%%%%%%%%%
test_map = load("testing_map2.mat");
%whos -file testing_map.mat omap3D

omap = test_map.omap2D;

% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
%show(omap)


% Define start and end position

% From = [From(1:3) (eul2quat(From(4:6)))];
% %display(From)
% To = [To(1:3) (eul2quat(To(4:6)))];

% 
% [waypoints, nWayPoints, states] = wp(From, To, omap);
% [waypoints1, nWayPoints1] = wp(To, From, omap);
waypoints_tot = [];
states_tot = [];

for i = 2:size(targets,1)
    % disp(targets(i-1, :))
    % disp(i)
    [waypoints, nWayPoints, states] = wp(targets(i-1, :), targets(i, :), omap);
    % disp(waypoints)
    
    waypoints_tot = cat(1, waypoints_tot, waypoints(2:size(waypoints, 1), :));
    states_tot = cat(1, states_tot, states(2:size(states, 1), :));
end
    % waypoints_tot = [waypoints_tot, waypoints];
disp(waypoints_tot);
disp(states_tot);





number_of_points = size(waypoints_tot, 1);

mode_vector = zeros(number_of_points, 1)+2;
mode_vector(1, 1) =  1;
mode_vector(number_of_points, 1) = 4;




waypoint_struct = struct('mode',[],'position',[], 'params',[]);
waypoint_struct = repmat(waypoint_struct,number_of_points, 1);

waypoint_struct(1).mode = uint8(1) ;
waypoint_struct(1).position = single([0;0;-15]);
waypoint_struct(1).params = single([0;0;0;0]);
waypoint_struct(number_of_points).mode = uint8(7) ;
waypoint_struct(number_of_points).position = single([waypoints_tot(number_of_points,1);waypoints_tot(number_of_points,2);-15]);
waypoint_struct(number_of_points).params = single([-1;-1;-1;-1]);


% for j = 2:number_of_points
%     a = states_tot(j-1, 1) - states_tot(j, 1);
%     o = states_tot(j-1, 2) - states_tot(j, 2);
%     ang = atan(o/a);
%     disp(ang);
% 
% end

for i = 2:number_of_points-1
    waypoint_struct(i).mode = uint8(mode_vector(i,1)) ;
    waypoint_struct(i).position = single([waypoints_tot(i,1);waypoints_tot(i,2);-15]);
    u = [waypoints_tot(i, 1) - waypoints_tot(i-1, 1); waypoints_tot(i, 2) - waypoints_tot(i-1, 2)];
    v = [waypoints_tot(i, 1) - waypoints_tot(i-1, 1); 0];
    dotUV = dot(u, v);
    normU = norm(u);
    normV = norm(v);

    theta = acos(dotUV/(normU * normV));
    disp(theta)
    waypoint_struct(i).params = single([0;0;0;0]);
end

display(waypoint_struct);

assignin('base',"waypoint_struct", waypoint_struct);

waypoint_struct= outWPS;

show(omap)
hold on
scatter(targets(1,1),targets(1,2),30,".r")
scatter(targets(size(targets, 1),1),targets(size(targets, 1),2),30,".g")

% Plot the waypoints

plot(waypoints_tot(:,1),waypoints_tot(:,2), "-g")
plot(states_tot(:,1),states_tot(:,2), "-y")

view([-31 63])
legend("Start Position","Goal Position","Planned Path", "Optimize Path")
hold off


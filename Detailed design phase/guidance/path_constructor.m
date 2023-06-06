
function waypoint_struct = generating_waypoints(From, To)

persistent outWPS

if isempty(outWPS)
    outWPS = single(zeros(2,4));
end

% 
rng(100,"twister");
From = [0 0 0];
To = [500 500 0];

targets = [[300, 240, 0];[700, 240,  0];[700, 730,  0];[300, 730,  0];];
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


[waypoints, nWayPoints, states] = wp(From, To, omap);
[waypoints1, nWayPoints1] = wp(To, From, omap);
for i = 1:sizeof(targets,1)
    [waypoints, nWayPoints, states] = wp(From, To, omap);

    waypoints = cat(1, waypoints, waypoints1(2:size(waypoints1, 1), :));

disp(waypoints);
disp(states);


waypoints_tot = cat(1, waypoints, waypoints1(2:size(waypoints1, 1), :));


number_of_points = nWayPoints;

mode_vector = zeros(number_of_points, 1)+2;
mode_vector(1, 1) =  1;
mode_vector(number_of_points, 1) = 4;




waypoint_struct = struct('mode',[],'position',[], 'params',[]);
waypoint_struct = repmat(waypoint_struct,number_of_points, 1);

waypoint_struct(1).mode = uint8(1) ;
waypoint_struct(1).position = single([0;0;-15]);
waypoint_struct(1).params = single([0;0;0;0]);
waypoint_struct(number_of_points).mode = uint8(7) ;
waypoint_struct(number_of_points).position = single([waypoints(number_of_points,1);waypoints(number_of_points,2);-15]);
waypoint_struct(number_of_points).params = single([-1;-1;-1;-1]);


for i = 2:number_of_points-1
    waypoint_struct(i).mode = uint8(mode_vector(i,1)) ;
    waypoint_struct(i).position = single([waypoints(i,1);waypoints(i,2);-15]);
    waypoint_struct(i).params = single([0;0;0;0]);
end

display(waypoint_struct);

assignin('base',"waypoint_struct", waypoint_struct);

waypoint_struct= outWPS;

show(omap)
hold on
scatter(From(1),From(2),30,".r")
scatter(To(1),To(2),30,".g")

% Plot the waypoints

plot(waypoints(:,1),waypoints(:,2), "-g")



view([-31 63])
legend("Start Position","Goal Position","Planned Path")
hold off


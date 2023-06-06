
function waypoint_struct = generating_waypoints(From, To)

persistent outWPS

if isempty(outWPS)
    outWPS = single(zeros(2,4));
end

% 
rng(100,"twister");
% omap3D =  occupancyMap3D(1);
% mapWidth = 500;
% mapLength = 500;
% 
% width = 20;                 % The largest integer in the sample intervals for obtaining width, length and height                                                     
% length = 20;                % can be changed as necessary to create different occupancy maps.
% height = 200;
% turbinepoints = [0 -25 -50 -75;0 -25 -50 -75;-15 -15 -20 0];
From = [0 0 15 0 0 0];
To = [500 500 20 0 0 0];
To2 = [300 300 20 0 0 0];

% posx = [50, 450, 50, 450] ;
% posy = [50, 50, 450, 450;];
% 
% for j = 1:4
%     xpos1 = posx(1,j);
%     ypos1 = posy(1,j);
% 
%     [xObstacle,yObstacle,zObstacle] = meshgrid(xpos1:xpos1+width,ypos1:ypos1+length,0:height);
%     xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
%     setOccupancy(omap3D,xyzObstacles,1);
% end
% 
% [xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,-1);
% xyzGround = [xGround(:) yGround(:) zGround(:)];
% setOccupancy(omap3D,xyzGround,1)


%%%%%%%%%%%
test_map = load("testing_map.mat");
%whos -file testing_map.mat omap3D

omap = test_map.omap3D;

% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
%show(omap)


% Define start and end position

From = [From(1:3) (eul2quat(From(4:6)))];
%display(From)
To = [To(1:3) (eul2quat(To(4:6)))];

nSamples = 20;
[states, initialStates, waypoints] = wp(From, To, omap, nSamples);
[states1, initialStates1, waypoints1] = wp(To, From, omap, nSamples);

disp(waypoints);

states_tot = cat(1, states, states1(2:size(states, 1), :));
initialStates_tot = cat(1, initialStates, initialStates1(2:size(initialStates, 1), :));
waypoints_tot = cat(1, waypoints, waypoints1(2:size(waypoints, 1), :));


number_of_points = size(states_tot,1);

mode_vector = zeros(number_of_points, 1)+2;
mode_vector(1, 1) =  1;
mode_vector(number_of_points, 1) = 4;




waypoint_struct = struct('mode',[],'position',[], 'params',[]);
waypoint_struct = repmat(waypoint_struct,number_of_points, 1);
waypoint_struct(1).mode = uint8(1) ;
waypoint_struct(1).position = single([0;0;-states_tot(1,3)]);
waypoint_struct(1).params = single([0;0;0;0]);


for i = 2:number_of_points
    waypoint_struct(i).mode = uint8(mode_vector(i,1)) ;
    waypoint_struct(i).position = single([states_tot(i,1);states_tot(i,2);-states_tot(i,3)]);
    waypoint_struct(i).params = single([0;0;0;0]);
end

display(waypoint_struct);
%waypoint_struct(1, 1).mode = uint8(mode_vector);
%waypoint_struct(2, 1).position = single(test_points);
%waypoint_struct(3, 1).params = single(zeros(5, 4));


assignin('base',"waypoint_struct", waypoint_struct);

waypoint_struct= outWPS;

show(omap)
hold on
scatter3(From(1),From(2),From(3),30,".r")
scatter3(To(1),To(2),To(3),30,".g")

% Plot the waypoints
plot3(waypoints_tot(:,1),waypoints_tot(:,2),waypoints_tot(:,3),"-g")


% Plot the initial trajectory
plot3(initialStates_tot(:,1),initialStates_tot(:,2),initialStates_tot(:,3),"-y")

% Plot the final valid trajectory
plot3(states_tot(:,1),states_tot(:,2),states_tot(:,3),"-c")
view([-31 63])
legend("","Start Position","Goal Position","Planned Path", "Initial Trajectory","Valid Trajectory")
hold off


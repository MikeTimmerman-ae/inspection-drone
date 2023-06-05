function waypoint_struct = generating_waypoints(From, To)

persistent outWPS

if isempty(outWPS)
    outWPS = single(zeros(2,4));
end

rng(100,"twister");
omap3D =  occupancyMap3D(1);
mapWidth = 500;
mapLength = 500;

width = 20;                 % The largest integer in the sample intervals for obtaining width, length and height                                                     
length = 20;                % can be changed as necessary to create different occupancy maps.
height = 200;

posx = [50, 450, 50, 450] ;
posy = [50, 50, 450, 450;];

for j = 1:4
    xpos1 = posx(1,j);
    ypos1 = posy(1,j);
    
    [xObstacle,yObstacle,zObstacle] = meshgrid(xpos1:xpos1+width,ypos1:ypos1+length,0:height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    setOccupancy(omap3D,xyzObstacles,1);
end

[xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,-1);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(omap3D,xyzGround,1)


%%%%%%%%%%%
test_map = load("testing_map.mat");

omap = test_map.omap3D;

% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
%show(omap)


% Define start and end position

From = [From(1:3) (eul2quat(From(4:6)))];
%display(From)
To = [To(1:3) (eul2quat(To(4:6)))];

startPose = From;  % [x y z qw qx qy qz]
goalPose1 = To;


%valid = all(isStateValid(sv,[startPose; goalPose1; goalPose2]));

% Plot the start and goal poses
%hold on
%scatter3(startPose(1),startPose(2),startPose(3),100,".r")
%scatter3(goalPose1(1),goalPose1(2),goalPose1(3),100,".g")
%scatter3(goalPose2(1),goalPose2(2),goalPose2(3),100,".y") 
%view([160 63])
%legend("","Start Position","Goal 1 Position", "Goal 2 Position")
%hold off

ss = stateSpaceSE3([-20 500;
                    -20 500;
                    -20 500;
                    inf inf;
                    inf inf;
                    inf inf;
                    inf inf]);

sv = validatorOccupancyMap3D(ss,Map=omap);
sv.ValidationDistance = 0.1;


planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance = 50;
planner.GoalBias = 0.8;
planner.MaxIterations = 1000;
planner.ContinueAfterGoalReached = true;
planner.MaxNumTreeNodes = 20000;

[pthObj,solnInfo] = plan(planner,startPose,goalPose1);
if (~solnInfo.IsPathFound)
    disp("No Path Found by the RRT, terminating example")
    return
end


waypoints = pthObj.States;
nWayPoints = pthObj.NumStates;

% Calculate the distance between waypoints
distance = zeros(1,nWayPoints);
for i = 2:nWayPoints
    distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
end

% Assume a UAV speed of 6 (originally 3) m/s and calculate time taken to reach each waypoint
UAVspeed = 6;
timepoints = cumsum(distance/UAVspeed);
nSamples = 100;

% Compute states along the trajectory
initialStates = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100)';

states = initialStates;
valid = all(isStateValid(sv,states));

while(~valid)
    % Check the validity of the states
    validity = isStateValid(sv,states);

    % Map the states to the corresponding waypoint segments
    segmentIndices = exampleHelperMapStatesToPathSegments(waypoints,states);

    % Get the segments for the invalid states
    % Use unique, because multiple states in the same segment might be invalid
    invalidSegments = unique(segmentIndices(~validity));

    % Add intermediate waypoints on the invalid segments
    for i = 1:size(invalidSegments)
        midpoint = zeros(7);
        segment = invalidSegments(i);
        
        % Take the midpoint of the position to get the intermediate position
        midpoint(1:3) = (waypoints(segment,1:3) + waypoints(segment+1,1:3))/2;
        
        % Spherically interpolate the quaternions to get the intermediate quaternion
        midpoint(4:7) = slerp(quaternion(waypoints(segment,4:7)),quaternion(waypoints(segment+1,4:7)),.5).compact;
        waypoints = [waypoints(1:segment,:); midpoint; waypoints(segment+1:end,:)];

    end

    nWayPoints = size(waypoints,1);
    distance = zeros(1,nWayPoints);
    for i = 2:nWayPoints
        distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
    end
    sum(distance)
    % Calculate the time taken to reach each waypoint
    timepoints = cumsum(distance/UAVspeed);
    nSamples = 100;
    states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=5000)';    
    
    % Check if the new trajectory is valid
    valid = all(isStateValid(sv,states));
   
end



number_of_points = size(states,1);

mode_vector = zeros(number_of_points, 1)+2;
mode_vector(1, 1) =  1;
mode_vector(number_of_points, 1) = 4;




waypoint_struct = struct('mode',[],'position',[], 'params',[]);
waypoint_struct = repmat(waypoint_struct,number_of_points, 1);


for i = 1:number_of_points
    waypoint_struct(i).mode = uint8(mode_vector(i,1)) ;
    waypoint_struct(i).position = single(states(i,1:3));
    waypoint_struct(i).params = single([0,0,0,0]);
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
plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")

% Plot the initial trajectory
plot3(initialStates(:,1),initialStates(:,2),initialStates(:,3),"-y")

% Plot the final valid trajectory
plot3(states(:,1),states(:,2),states(:,3),"-c")
view([-31 63])
legend("","Start Position","Goal Position","Planned Path","Initial Trajectory","Valid Trajectory")
hold off
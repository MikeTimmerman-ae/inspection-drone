
function waypoint_struct = generating_waypoints(From, To)

persistent outWPS

if isempty(outWPS)
    outWPS = single(zeros(2,4));
end
 
rng(100,"twister");

From = [0 0 15 0 0 0];
To = [500 500 20 0 0 0];
To2 = [300 300 20 0 0 0];

targets = [[0, 0, 0, 0, 0, 0];
 [300, 240, 130, 0, 0, 0];
 [700, 240, 130, 0, 0, 0];
 [700, 730, 130, 0, 0, 0];
 [300, 730, 130, 0, 0, 0];
 [0, 0, 0, 0, 0, 0];
 ]

test_map = load("testing_map.mat");
omap = test_map.omap3D;
omap.FreeThreshold = omap.OccupiedThreshold;

% Transform input to quaternions
From = [From(1:3) (eul2quat(From(4:6)))];
To = [To(1:3) (eul2quat(To(4:6)))];

trans_targets(:,:) = [targets(:,1:3) (eul2quat(targets(:,4:6)))];
nSamples = 20;


ss = stateSpaceSE3([-20 220;
                    -20 220;
                    -10 100;
                    inf inf;
                    inf inf;
                    inf inf;
                    inf inf]);

sv = validatorOccupancyMap3D(ss,Map=omap);
sv.ValidationDistance = 0.1;
planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance = 100;
planner.GoalBias = 0.8;
planner.MaxIterations = 1000;
planner.ContinueAfterGoalReached = true;
planner.MaxNumTreeNodes = 10000;



waypoints = []


for i = 2:size(targets,1)
    startPose = trans_targets(i-1,:)
    goalPose = trans_targets(i, :)
    [pthObj,solnInfo] = plan(planner,startPose,goalPose);
    size(waypoints)
    size(pthObj.States)
    if (~solnInfo.IsPathFound)
        disp("No Path Found by the RRT, terminating example")
        return
    end
    
   waypoints = [waypoints; pthObj.States]
   
end


%states_tot = cat(1, states, states1(2:size(states, 1), :));
%waypoints_tot = cat(1, waypoints, waypoints1(2:size(waypoints, 1), :));


show(omap)
hold on
scatter3(startPose(1),startPose(2),startPose(3),30,".r")
scatter3(goalPose(1),goalPose(2),goalPose(3),30,".g")
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),"-c")
view([-31 63])
legend("","Start Position","Goal Position","Planned Path","Initial Trajectory","Valid Trajectory")
hold off




% 
% [states, initialStates, waypoints] = wp(From, To, omap, nSamples);
% [states1, initialStates1, waypoints1] = wp(To, From, omap, nSamples);
% 
% disp(waypoints);
% 
% states_tot = cat(1, states, states1(2:size(states, 1), :));
% initialStates_tot = cat(1, initialStates, initialStates1(2:size(initialStates, 1), :));
% waypoints_tot = cat(1, waypoints, waypoints1(2:size(waypoints, 1), :));
% 
% 
% number_of_points = size(states_tot,1);
% 
% mode_vector = zeros(number_of_points, 1)+2;
% mode_vector(1, 1) =  1;
% mode_vector(number_of_points, 1) = 4;
% 
% 
% 
% 
% waypoint_struct = struct('mode',[],'position',[], 'params',[]);
% waypoint_struct = repmat(waypoint_struct,number_of_points, 1);
% waypoint_struct(1).mode = uint8(1) ;
% waypoint_struct(1).position = single([0;0;-states_tot(1,3)]);
% waypoint_struct(1).params = single([0;0;0;0]);
% 
% 
% for i = 2:number_of_points
%     waypoint_struct(i).mode = uint8(mode_vector(i,1)) ;
%     waypoint_struct(i).position = single([states_tot(i,1);states_tot(i,2);-states_tot(i,3)]);
%     waypoint_struct(i).params = single([0;0;0;0]);
% end
% 
% display(waypoint_struct);
% %waypoint_struct(1, 1).mode = uint8(mode_vector);
% %waypoint_struct(2, 1).position = single(test_points);
% %waypoint_struct(3, 1).params = single(zeros(5, 4));
% 
% 
% assignin('base',"waypoint_struct", waypoint_struct);
% 
% waypoint_struct= outWPS;
% 
% show(omap)
% hold on
% scatter3(From(1),From(2),From(3),30,".r")
% scatter3(To(1),To(2),To(3),30,".g")
% 
% % Plot the waypoints
% plot3(waypoints_tot(:,1),waypoints_tot(:,2),waypoints_tot(:,3),"-g")
% 
% 
% % Plot the initial trajectory
% plot3(initialStates_tot(:,1),initialStates_tot(:,2),initialStates_tot(:,3),"-y")
% 
% % Plot the final valid trajectory
% plot3(states_tot(:,1),states_tot(:,2),states_tot(:,3),"-c")
% view([-31 63])
% legend("","Start Position","Goal Position","Planned Path", "Initial Trajectory","Valid Trajectory")
% hold off


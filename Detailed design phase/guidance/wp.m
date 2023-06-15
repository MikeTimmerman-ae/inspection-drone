function [waypointsSpaced, sum] = wp(From ,To, omap)
    
    
    % 
    % ss = stateSpaceSE2;
    % 
    % ss.StateBounds = [omap.XWorldLimits; omap.YWorldLimits; [-pi pi]];
    
    
    
    planner = plannerAStarGrid(omap);
    % planner.MaxConnectionDistance = 50;
    % planner.GoalBias = 0.8;
    % planner.MaxIterations = 1000;
    % planner.ContinueAfterGoalReached = true;
    % planner.MaxNumTreeNodes = 20000;
    
    
    pthObj = plan(planner,From,To, "world");
    % 
    % if (~solnInfo.IsPathFound)
    %     disp("No Path Found by the RRT, terminating example")
    %     return
    % 
    % end
    
    % disp(pthObj);
    waypoints = pthObj;
   
    nWayPoints = size(waypoints, 1);
    
    % disp(nWayPoints);
    % Calculate the distance between waypoints
    % distance = zeros(1,nWayPoints);
    % for i = 2:nWayPoints
    %     distance(i) = norm(waypoints(i,1:2) - waypoints(i-1,1:2));
    % end

    % UAVspeed = 3;
    % timepoints = cumsum(distance/UAVspeed);
    % nSamples = 50;
    waypointsSpaced = [];
    % disp(waypoints(1,:));
    waypointsSpaced(1 ,:) = waypoints(1,1:2);
    k = 2;
    sum = 0;
    for i = 2:nWayPoints
        % n = int((norm(inspectionStates(i, (1:3)) -  inspectionStates(i-1, (1:3)))) /10);
        sum = sum + norm(waypoints(i, (1:2)) -  waypoints(i-1, (1:2)));
        if norm(waypoints(i, (1:2)) -  waypoints(i-1, (1:2))) > 80
            n = 8;
        elseif norm(waypoints(i, (1:2)) -  waypoints(i-1, (1:2))) > 50
            n = 5;
        elseif norm(waypoints(i, (1:2)) -  waypoints(i-1, (1:2))) > 30
            n = 3;
        else
            n = 2;
        end
        x = linspace(waypoints(i-1, 1), waypoints(i, 1), n);
        y = linspace(waypoints(i-1, 2), waypoints(i, 2), n);
        % z = linspace(waypoints(i-1, 3), waypoints(i, 3), n);
        
        for j = 1:n-1
            waypointsSpaced(k,:) = [x(j+1); y(j+1)];
            % disp(k)
            % disp(((i-2)*(n)) + j + 1);
            k = k + 1;
            
            
        end
      
    end
 
    % disp(waypointsSpaced);
    % disp(timepoints)
    
    % % Compute states along the trajectory
    % initialStates = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100)';
    % 
    % states = initialStates;
    % valid = all(isStateValid(sv,states));
    % 
    % while(~valid)
    %     % Check the validity of the states
    %     validity = isStateValid(sv,states);
    % 
    %     % Map the states to the corresponding waypoint segments
    %     segmentIndices = helper_wp(waypoints,states);
    % 
    %     % Get the segments for the invalid states
    %     % Use unique, because multiple states in the same segment might be invalid
    %     invalidSegments = unique(segmentIndices(~validity));
    % 
    %     % Add intermediate waypoints on the invalid segments
    %     for i = 1:size(invalidSegments)
    %         midpoint = zeros(1, 3);
    %         segment = invalidSegments(i);
    % 
    %         % Take the midpoint of the position to get the intermediate position
    %         midpoint(1:2) = (waypoints(segment,1:2) + waypoints(segment+1,1:2))/2;
    % 
    %         % Spherically interpolate the quaternions to get the intermediate quaternion
    %         midpoint(3) = (waypoints(segment,3) + waypoints(segment+1,3))/2;
    %         % disp(midpoint)
    %         % disp(waypoints)
    %         waypoints = [waypoints(1:segment,:); midpoint; waypoints(segment+1:end,:)];
    % 
    %     end
    %     % disp(waypoints)
    % 
    %     nWayPoints = size(waypoints,1);
    %     distance = zeros(1,nWayPoints);
    %     for i = 2:nWayPoints
    %         distance(i) = norm(waypoints(i,1:2) - waypoints(i-1,1:2));
    %     end
    %     sum(distance)
    % 
    %     % Calculate the time taken to reach each waypoint
    %     timepoints = cumsum(distance/UAVspeed);
    %     % disp(timepoints)
    % 
    %     % states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=5000)'; 
    %     states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100)';
    % 
    %     % Check if the new trajectory is valid
    %     valid = all(isStateValid(sv,states));
    % 
    % end
    
    
end
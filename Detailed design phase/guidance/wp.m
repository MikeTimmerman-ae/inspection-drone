function [waypoints, nWayPoints, states] = wp(From ,To, omap)
    
    
    
    ss = stateSpaceSE2;

    ss.StateBounds = [omap.XWorldLimits; omap.YWorldLimits; [-pi pi]];
    sv = validatorOccupancyMap(ss);
    sv.Map = omap;
    sv.ValidationDistance = 0.1;
    
    
    planner = plannerRRTStar(ss,sv);
    planner.MaxConnectionDistance = 50;
    planner.GoalBias = 0.8;
    planner.MaxIterations = 1000;
    planner.ContinueAfterGoalReached = true;
    planner.MaxNumTreeNodes = 20000;
    
    
    [pthObj,solnInfo] = plan(planner,From,To);
    
    if (~solnInfo.IsPathFound)
        disp("No Path Found by the RRT, terminating example")
        return
    
    end
    
    
    waypoints = pthObj.States;
   
    nWayPoints = pthObj.NumStates;
    
    
    % Calculate the distance between waypoints
    distance = zeros(1,nWayPoints);
    for i = 2:nWayPoints
        distance(i) = norm(waypoints(i,1:2) - waypoints(i-1,1:2));
    end
    UAVspeed = 3;
    timepoints = cumsum(distance/UAVspeed);
    nSamples = 20;

    disp(timepoints)
    
    % Compute states along the trajectory
    initialStates = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100)';
    
    states = initialStates;
    valid = all(isStateValid(sv,states));
    
    while(~valid)
        % Check the validity of the states
        validity = isStateValid(sv,states);

        % Map the states to the corresponding waypoint segments
        segmentIndices = helper_wp(waypoints,states);

        % Get the segments for the invalid states
        % Use unique, because multiple states in the same segment might be invalid
        invalidSegments = unique(segmentIndices(~validity));

        % Add intermediate waypoints on the invalid segments
        for i = 1:size(invalidSegments)
            midpoint = zeros(1, 3);
            segment = invalidSegments(i);

            % Take the midpoint of the position to get the intermediate position
            midpoint(1:2) = (waypoints(segment,1:2) + waypoints(segment+1,1:2))/2;

            % Spherically interpolate the quaternions to get the intermediate quaternion
            midpoint(3) = (waypoints(segment,3) + waypoints(segment+1,3))/2;
            disp(midpoint)
            disp(waypoints)
            waypoints = [waypoints(1:segment,:); midpoint; waypoints(segment+1:end,:)];

        end
        disp(waypoints)
     
        nWayPoints = size(waypoints,1);
        distance = zeros(1,nWayPoints);
        for i = 2:nWayPoints
            distance(i) = norm(waypoints(i,1:2) - waypoints(i-1,1:2));
        end
        sum(distance)
       
        % Calculate the time taken to reach each waypoint
        timepoints = cumsum(distance/UAVspeed);
        disp(timepoints)

        % states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=5000)'; 
        states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100)';

        % Check if the new trajectory is valid
        valid = all(isStateValid(sv,states));

    end
    
    
end
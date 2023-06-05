function [states, initialStates, waypoints] = wp(From ,To, omap)
    
    
    
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
        distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
    end
    
    % Assume a UAV speed of 6 (originally 3) m/s and calculate time taken to reach each waypoint
    UAVspeed = 6;
    timepoints = cumsum(distance/UAVspeed);
    nSamples = 20;
    
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
        nSamples = 20;
        states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=5000)';    
    
        % Check if the new trajectory is valid
        valid = all(isStateValid(sv,states));
    
    end
end
function [states, initialStates, waypoints] = wp(From ,To, omap, nSamples)
    
    
    
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
    disp(waypoints)
    nWayPoints = pthObj.NumStates;
    
    
    % Calculate the distance between waypoints
    distance = zeros(1,nWayPoints);
    for i = 2:nWayPoints
        distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
    end
    
    % Assume a UAV speed of 6 (originally 3) m/s and calculate time taken to reach each waypoint
    UAVspeed = 6;
    timepoints = cumsum(distance/UAVspeed);
    disp(timepoints);
    
    
    % Compute states along the trajectory
    initialStates = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=0.1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=100)';
    
    states = initialStates;
    valid = all(isStateValid(sv,states));
   
    % nStates = size(states,1);
    % disp(nStates);
    % A = [];
    % i = 1;
    % while i < nStates-1
    %     disp(norm(states(i+1,1:3) - states(i,1:3)));
    %     disp(i);
    % 
    %     j = 1;
    %     while norm(states(i+j,1:3) - states(i,1:3)) <= 2
    %         disp(j);
    %         A = [A, i+j];
    %         if j+1 + i <= 999
    %             j = j+1;
    %         else
    %             break
    %         end
    % 
    %     end
    %     i = i + j;
    % 
    % end
    % disp(A);
    % states(A, :) = [];
    % disp(states);
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

        states = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=1,MaxSegmentTime=20,TimeAllocation=true,TimeWeight=5000)';    
        % nStates = size(states,1);
        % disp(nStates);
        % for i = 1:nStates+1
        %     if norm(states(i+1,1:3) - states(i,1:3)) <= 1
        %         states(i, :) = [];
        %     end
        % end
        % disp(states);
        % Check if the new trajectory is valid
        valid = all(isStateValid(sv,states));

    end
end
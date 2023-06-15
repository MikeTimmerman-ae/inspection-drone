function [waypointsSpaced, sum] = wp2(From ,To, omap)
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
    
  
    waypointsSpaced = [];
    waypointsSpaced(1 ,:) = waypoints(1,1:2);
    k = 2;
    sum = 0;
    for i = 2:nWayPoints
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
        
        
        for j = 1:n-1
            waypointsSpaced(k,:) = [x(j+1); y(j+1)];
            
            k = k + 1;
            
            
        end
        
    end
    % disp(waypointsSpaced);



    
    
    
end

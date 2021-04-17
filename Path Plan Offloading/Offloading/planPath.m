function [pathWaypoints, pathFound, tPathPlan] = planPath(offload, startXY, goalXY, plannerRRT, plannerA, mapinfl, mapinfl2, a)

interDist = 0.01;

startGrid = world2grid(mapinfl2,startXY(1:2));
goalGrid = world2grid(mapinfl2,goalXY(1:2));

if offload == 1
    % returns if inflated obstacle in pose, deeming movement unsafe
    if getOccupancy(mapinfl, startXY(1:2)) || getOccupancy(mapinfl, goalXY(1:2))
        pathWaypoints = 0;
        pathFound = 0;
        tPathPlan = 0;
        return;
    end

    tic;
    [path, solnInfo] = plan(plannerRRT, startXY, goalXY);
    tPathPlan = toc;
    tPathPlan = tPathPlan/a; % increase speed by factor of a
    pathFound = solnInfo.IsPathFound;
    
    if pathFound % Interpolate if solution found
        pathLen = pathLength(path);
        interSize = ceil(pathLen/interDist);
        interpolate(path, interSize);
    else
        pathWaypoints = 0;
        return;
    end
    
    pathWaypoints = path.States;
else
    % returns if twice inflated obstacle in pose, deeming movement unsafe
    if getOccupancy(mapinfl2, startXY(1:2)) || getOccupancy(mapinfl2, goalXY(1:2))
        pathWaypoints = 0;
        pathFound = 0;
        tPathPlan = 0;
        return;
    end
    
    tic;
    [path, solnInfo] = plan(plannerA, startGrid, goalGrid);
    tPathPlan = toc;
    pause(tPathPlan*9) % adds realism
    tPathPlan = tPathPlan*10; % slow by factor of 10
    pathFound = abs(isinf(solnInfo.PathCost)-1); 
    
    if ~pathFound
        pathWaypoints = 0;
        return;
    end
    
    pathWaypoints = grid2world(mapinfl2,path);
end

end


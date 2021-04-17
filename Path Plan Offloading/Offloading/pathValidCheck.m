function replan = pathValidCheck(offload, stateValidator, pathWaypoints, mapinfl, mapinfl2, robotCurrentPose)

% Initialise flag low
replan = false;
stateValidator.Map = mapinfl;

%Ensures doen't look at points in path already passed
distances = pdist2(pathWaypoints(:,1:2), robotCurrentPose(1:2));
minD = min(distances);
[row, ~] = find(distances == minD);

% Check path states for obstacles
if offload == 1
    statesValid = isStateValid(stateValidator, pathWaypoints(row:end,:));
    replan =~ all(statesValid);
else  
    for k=row:height(pathWaypoints)
        if getOccupancy(mapinfl2, [pathWaypoints(k,1), pathWaypoints(k,2)]) == 1
            replan=true;
            break
        end
    end
    
end


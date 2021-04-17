clear;


% Map Parameters
mapSize = 20;
cells = mapSize^2;
resolution = 1;

% Parameters of Simple car
carLength = 0.5;
psimax = deg2rad(30);
MinTurningRadius = carLength/tan(psimax);
maxAng = v/MinTurningRadius;


controller = controllerPurePursuit;
controller.LookaheadDistance = 0.5*MinTurningRadius;

% Number of Data Points
dataLength = 100;

start = randi(mapSize,dataLength,2);
goal = randi(mapSize,dataLength,2);

% additional restricition of 25 because will be inflating the obstacles.
%max, obstacles occupy half the map.
% obsLimit = round(cells/(2*25));
% obstacles = randi(obsLimit,dataLength, 1);

% Measurement Parameters
obsDensityRatioMAP = zeros(dataLength,1);
obsDensityRatioSQ = zeros(dataLength,1);
distEuclidean = zeros(dataLength,1);
time2GoalAStar = zeros(dataLength, 1);
time2GoalRRT = zeros(dataLength, 1);
feasAStar = zeros(dataLength,1);
feasRRT = zeros(dataLength,1);
incomAStar = zeros(dataLength,1);
incomRRT = zeros(dataLength,1);
colAStar = zeros(dataLength,1);
colRRT = zeros(dataLength,1);

% Obstacle Positions
obstacles = [1,11; 1,12; 1,13; 2,11; 2,12; 2,13; 3,11; 3,12; 3,13; 4,11; 4,12; 4,13; ...
    20,10; 19,10; 18,10; 17,10; 16,10; 20,11; 19,11; 18,11; 17,11; 16,11; ...
    20,9; 19,9; 18,9; 17,9; 16,9; 20,8; 19,8; 18,8; 17,8; 16,8; ...
    15,8; 15,9; 15,10; 15,11];


for i = 1:dataLength
    
    obsPos = randi(mapSize, 1, 1);
    %     obsPos = randi(mapSize, obstacles(i), 2); % Random map
    
    map = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
    %     setOccupancy(map, obsPos, 1);
    setOccupancy(map, obstacles, 1);
    setOccupancy(map, [10,obsPos], 1);
    
    mapinfl = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
    %     setOccupancy(mapinfl, obsPos, 1);
    setOccupancy(mapinfl, obstacles, 1);
    setOccupancy(mapinfl, [10,obsPos], 1);
    inflate(mapinfl,1);
    
    mapinfl2 = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
    %     setOccupancy(mapinfl2, obsPos, 1);
    setOccupancy(mapinfl2, obstacles, 1);
    setOccupancy(mapinfl2, [10,obsPos], 1);
    inflate(mapinfl2,1);
    inflate(mapinfl2,1);
    
    while getOccupancy(mapinfl2, start(i,:), "grid") == 1
        start(i,:) = randi(mapSize,1,2);
    end
    while getOccupancy(mapinfl2, goal(i,:), "grid") == 1
        goal(i,:) = randi(mapSize,1,2);
    end
    while goal(i,:) == start(i,:)
        start(i,:) = randi(mapSize,1,2);
        while getOccupancy(mapinfl2, start(i,:), "grid") == 1
            start(i,:) = randi(mapSize,1,2);
        end
    end
    
    bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    
    % Point towards goal to improves success
    theta = atan2(goal(i,2)-start(i,2), goal(i,1)-start(i,1));
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% A* %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    plannerA = plannerAStarGrid(mapinfl2);
    
    clear pathAStar solnInfoAStar;
    [pthAStar, solnInfoAStar] = plan(plannerA, start(i,:), goal(i,:));
    
    %If no path found = 1
    feasAStar(i) = isinf(solnInfoAStar.PathCost);
    
    
    startXY = grid2world(mapinfl, start(i,:));
    goalXY = grid2world(mapinfl, goal(i,:));
    
    startWorld = [startXY, theta];
    goalWorld = [goalXY, theta];
    
    robotCurrentPose =  [startWorld, 0];
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(:));
    
    % Initialise Contoller
    release(controller);
    v=1.25;
    controller.DesiredLinearVelocity = v;
    controller.MaxAngularVelocity = v/MinTurningRadius;
    goalRadius = 0.2;
    
    % Discrete controller rate
    sampleTime = 0.1;
    tspan = [0 sampleTime];
    
    count = 1;
    
    
    % Follow A* if path found
    if ~isempty(pthAStar)
        % only pass waypoints to controller if path exists
        pathAStar = grid2world(mapinfl, pthAStar);
        controller.Waypoints = pathAStar(:,1:2);
        
        % Simulate
        tic;
        while distanceToGoal > goalRadius
            % Action inputs computation
            [v, w] = controller(robotCurrentPose(1:3));
            psiPlus1 = atan((w*carLength)./v);
            psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
            
            % Simulate movement of car
            [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
            robotCurrentPose = pose(end,:);
            distanceToGoal = norm(robotCurrentPose(1:2) - goalXY(1:2));
            
            % Check if crash
            flag = checkConstraints(map, robotCurrentPose, bounds, carLength);
            
            % If crash leave simulation
            if flag
                break;
            end
            
            % Abort if car has not converged within 50seconds
            count = count + 1;
            if count > 500
                disp('A* did not complete')
                disp(i);
                incomAStar(i) = 1;
                break;
            end
            waitfor(vizRate);
        end
        time2GoalAStar(i) = toc;
        
        if flag
            colAStar(i) = 1;
            time2GoalAStar(i) = 0;
        end
    end
    
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%% RRT %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ss = stateSpaceDubins(bounds);
    ss.MinTurningRadius = MinTurningRadius;
    stateValidator = validatorOccupancyMap(ss);
    stateValidator.ValidationDistance = 0.2;
    stateValidator.Map = mapinfl;
    plannerRRT = plannerRRTStar(ss, stateValidator);
    plannerRRT.MaxConnectionDistance = 2.0;
    plannerRRT.MaxIterations = 1500;
    
    
    clear pathRRT solnInfo;
    [pathRRT, solnInfo] = plan(plannerRRT, startWorld, goalWorld);
    %If no path found = 0
    feasRRT(i) = solnInfo.IsPathFound;
    
    interDist = 0.01;
    
    % Initialise Contoller
    release(controller);
    v=2;
    controller.DesiredLinearVelocity = v;
    controller.MaxAngularVelocity = v/MinTurningRadius;
    
    robotCurrentPose =  [startWorld, 0];
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(:));
    count = 1;
    
    % Follow RRT if path found
    if ~isempty(pathRRT.States)
        % only pass waypoints to controller if path exists
        pathLen = pathLength(pathRRT);
        interSize = ceil(pathLen/interDist);
        interpolate(pathRRT, interSize);
        
        controller.Waypoints = pathRRT.States(:,1:2);
        
        % Simulate
        tic;
        while distanceToGoal > goalRadius
            % Action inputs computation
            [v, w] = controller(robotCurrentPose(1:3));
            psiPlus1 = atan((w*carLength)./v);
            psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
            
            % Simulate movement of car
            [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
            robotCurrentPose = pose(end,:);
            distanceToGoal = norm(robotCurrentPose(1:2) - goalXY(1:2));
            
            % Check if crash
            flag = checkConstraints(map, robotCurrentPose, bounds, carLength);
            
            % If crash leave simulation
            if flag
                break;
            end
            
            % Abort if have not converged to goal within 50seconds
            count = count + 1;
            if count > 500
                disp('RRT did not complete')
                disp(i);
                incomRRT(i) = 1;
                break;
            end
            waitfor(vizRate);
        end
        time2GoalRRT(i) = toc;
        
        if flag
            colRRT(i) = 1;
            time2GoalRRT(i) = 0;
        end
    end
    
    
    distEuclidean(i) = norm(goalXY-startXY);
    
    % Measure the independent variables
    %Grid locations used becasue thats the format of matMap
    limitsRow = sort([start(i,1), goal(i,1)]);
    limitsCol = sort([start(i,2), goal(i,2)]);
    
    matMap = occupancyMatrix(mapinfl);
    szMap = size(matMap);
    
    square = matMap(limitsRow(1):limitsRow(2), limitsCol(1):limitsCol(2));
    szSQ = size(square);
    
    obsDensityRatioMAP(i) = sum(matMap, 'all')/(szMap(1)*szMap(2));
    obsDensityRatioSQ(i) = sum(square, 'all')/(szSQ(1)*szSQ(2));
    
end

% Clean data for errors 

timediff = time2GoalRRT-time2GoalAStar;

errors = colRRT + colAStar + feasAStar + ~feasRRT + incomAStar + incomRRT;
idx = find(errors == 0);
tA = time2GoalAStar(idx);
tRRT = time2GoalRRT(idx);
obs = obsDensityRatioMAP(idx) + obsDensityRatioSQ(idx);
dist = distEuclidean(idx);
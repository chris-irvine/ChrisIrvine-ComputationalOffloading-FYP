clear;

data = struct();

% Map Parameters
mapSize = 20;
cells = mapSize^2;
resolution = 1;

% Parameters of Simple car
carLength = 0.5;
psimax = deg2rad(30);
MinTurningRadius = carLength/tan(psimax);

% Number of Data Points
loops = 5;
dataLength = 100;

start = randi(mapSize,dataLength,2);
goal = randi(mapSize,dataLength,2);

% additional restricition of 25 because will be inflating the obstacles.
%max, obstacles occupy half the map.
obsLimit = round(cells/(2*25));
obstacles = randi(obsLimit,dataLength, 1);

% Initialise parameters to reduce time
obsDensityRatioMAP = zeros(dataLength,1);
obsDensityRatioSQ2MAP = zeros(dataLength,1);
obsDensityRatioSQ = zeros(dataLength,1);
distEuclidean = zeros(dataLength,1);
timeAStar = zeros(dataLength, 1);
timeRRT = zeros(dataLength,loops);
timeRRtAvg= zeros(dataLength, 1);
timeAStarFail = zeros(dataLength, 1);
timeRRTFailAvg = zeros(dataLength,1);
timeInterpolate = zeros(dataLength,loops);
feasAStar = zeros(dataLength,1);
feasRRT = zeros(dataLength,loops);


for i = 1:dataLength
    
    obsPos = randi(mapSize, obstacles(i), 2);     

    map = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
    setOccupancy(map, obsPos, 1);
    
    mapinfl = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
    setOccupancy(mapinfl, obsPos, 1);
    inflate(mapinfl,1);

    mapinfl2 = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
    setOccupancy(mapinfl2, obsPos, 1);
    inflate(mapinfl2,1);
    inflate(mapinfl2,1);
    
    % Check validiy of the start and goal positions
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
    
    
    %%%%%%%%%%%%%% A* %%%%%%%%%%%%%%
    
    plannerA = plannerAStarGrid(mapinfl2);

    timeLoopsA = zeros(1,loops);
    for j = 1:loops
        clear pathAStar solnInfoAStar;
        tic
        [pthAStar, solnInfoAStar] = plan(plannerA, start(i,:), goal(i,:));
        timeLoopsA(j) = toc;
    end
    % If no path found = 1
    feasAStar(i) = isinf(solnInfoAStar.PathCost);
    
    
    %%%%%%%%%%%%%% RRT %%%%%%%%%%%%%%
    
    bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    ss = stateSpaceDubins(bounds);
    ss.MinTurningRadius = MinTurningRadius;
    stateValidator = validatorOccupancyMap(ss);
    stateValidator.ValidationDistance = 0.2;
    stateValidator.Map = mapinfl;
    plannerRRT = plannerRRTStar(ss, stateValidator);
    plannerRRT.MaxConnectionDistance = 2.0;
    plannerRRT.MaxIterations = 1500;
    
    startXY = grid2world(mapinfl, start(i,:));
    goalXY = grid2world(mapinfl, goal(i,:));

    % Point in direction of goal
    theta = atan2(goalXY(2) - startXY(2), goalXY(1) - startXY(1));
    
    startRRT = [startXY, theta];
    goalRRT = [goalXY, theta];
    
    interDist = 0.01;

    for j=1:loops
        clear pathRRT solnInfo;
        tic
        [pathRRT, solnInfo] = plan(plannerRRT, startRRT, goalRRT);
        timeRRT(i,j) = toc;

        %If no path found = 0
        feasRRT(i,j) = solnInfo.IsPathFound;
        
        if solnInfo.IsPathFound % Interpolate if soultion is found
            pathLen = pathLength(pathRRT);
            interSize = ceil(pathLen/interDist);
            
            tic
            interpolate(pathRRT,interSize);
            timeInterpolate(i,j) = toc;
        end
    end
    
    % Measure the independent variables, sort lowest and highest point to
    % avoid error
    limitsRow = sort([start(i,1), goal(i,1)]);
    limitsCol = sort([start(i,2), goal(i,2)]);
    
    matMap = occupancyMatrix(mapinfl);
    szMap = size(matMap);
    
    square = matMap(limitsRow(1):limitsRow(2), limitsCol(1):limitsCol(2));
    szSQ = size(square);
    
    if feasAStar(i) == 0 || sum(feasRRT(i,:)) ~= 0
        timeAStar(i) = sum(timeLoopsA)/loops;
    else
        timeAStarFail(i) = sum(timeLoopsA)/loops;
    end
    
    % Only average succesful loops
    if sum(feasRRT(i,:)) ~= loops
        indicies = find(feasRRT(i,:)==1);
        timeRRtAvg(i) = sum(timeRRT(i,indicies))/loops;
        timeRRTFailAvg(i) = sum(timeRRT(i,:))/loops;
    else
        timeRRtAvg(i) = sum(timeRRT(i,:))/loops;
    end
    
    obsDensityRatioMAP(i) = sum(matMap, 'all')/(szMap(1)*szMap(2));
    obsDensityRatioSQ(i) = sum(square, 'all')/(szSQ(1)*szSQ(2));
    

end

% Clean data for any failures

indAStar = find(feasAStar == 1);
successTimeAStar = timeAStar;
successTimeAStar(indAStar) = [];
successDistAStar = distEuclidean;
successDistAStar(indAStar) = [];
successObsDensityRatioMAPA = obsDensityRatioMAP;
successObsDensityRatioMAPA(indAStar) = [];
successObsDensityRatioSQA = obsDensityRatioSQ;
successObsDensityRatioSQA(indAStar) = [];
successObsAStar = successObsDensityRatioSQA + successObsDensityRatioMAPA;

indRRT = find(timeRRtAvg == 0);
successTimeRRT = timeRRtAvg;
successTimeRRT(indRRT) = [];
successDistRRT = distEuclidean;
successDistRRT(indRRT) = [];
successObsDensityRatioMAPRRT = obsDensityRatioMAP;
successObsDensityRatioMAPRRT(indRRT) = [];
successObsDensityRatioSQRRT = obsDensityRatioSQ;
successObsDensityRatioSQRRT(indRRT) = [];
successObsRRT = successObsDensityRatioSQRRT + successObsDensityRatioMAPRRT;


figure;
scatter3(successObsRRT, successDistRRT, successTimeRRT, 'filled');
xlabel('Obstacle Density')
ylabel('Distance (m)')
 zlabel('Time (s)');
title('RRT Computation Time');

figure;
scatter3(successObsAStar, successDistAStar, successTimeAStar, 'filled');
xlabel('Obstacle Density')
ylabel('Distance (m)')
 zlabel('Time (s)');
title('A* Computation Time');


cftool;


clear;

% Map Parameters
mapSize = 20;
resolution = 1;
numObstacles = 2;

% Parameters of Simple car
carLength = 0.5;
psiMax = deg2rad(30);
MinTurningRadius = carLength/tan(psiMax);


mapOccupancy = zeros(mapSize);

rng('default')
map = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
rng('default')
mapInfl = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
inflate(mapInfl, 1);
% rng('default')
% mapInfl2 = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
% inflate(mapInfl2,1);
% inflate(mapInfl2,1);



%%%%%%%%%%%%%%%%%%%%%%%%%% RRT %%%%%%%%%%%%%%%%%%%%%%%%%%

rng('default')

% RRT Path Plan
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = MinTurningRadius;
stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = mapInfl;
stateValidator.ValidationDistance = 0.2;

RRT = plannerRRTStar(ss, stateValidator);
RRT.MaxConnectionDistance = 2.0;
RRT.MaxIterations = 30000;

start = [0.5, 0.5, 0];
goal = [18.5, 18.5, 0];

goal = [10.5, 10.5, 0];
% start = [15.5, 14.5, -0.1];
% start = [15.5, 15.5, -0.1];

initPose = [start, 0];

pathRRT = plan(RRT, start, goal);
interDist = 0.01;
pathLen = pathLength(pathRRT);
interSize = ceil(pathLen/interDist);
interpolate(pathRRT, interSize);

% Initialise Controller for RRT
v = 2;
controller = controllerPurePursuit;
controller.Waypoints = pathRRT.States(:,1:2);
controller.LookaheadDistance = 0.5*MinTurningRadius;
controller.DesiredLinearVelocity = v;
controller.MaxAngularVelocity = v/MinTurningRadius;

robotCurrentPose = initPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));

traceRRT = initPose;

% Set Sample Time
sampleTime = 0.1;
tspan = [0 sampleTime];

goalRadius = 0.2;

k = 1;

while distanceToGoal > goalRadius
    k = k+1;
    
    % Action inputs computation
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    
    % Simulate movement of car
    [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
    robotCurrentPose = pose(end,:);
    
    traceRRT = [traceRRT; robotCurrentPose];
    
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));
    
    % Abort if doesn't converge within 30 seconds
    if k > 30/sampleTime
        break
    end
    
end

closestPurePursuitRRT = [];
errorRRT = [];

% Measure Error of RRT
for i = 1:pathRRT.NumStates
    distances = pdist2(traceRRT(:,1:2), pathRRT.States(i,1:2));
    minD = min(distances);
    [row, column] = find(distances == minD);
    closestPurePursuitRRT = [closestPurePursuitRRT; traceRRT(row,1:2)];
    errorRRT = [errorRRT; distances(row(1))];
end

averageErrorRRT = sum(errorRRT)/length(errorRRT);
maxErrorRRT = max(errorRRT);


figure;
show(map);
hold all;
plot(pathRRT.States(:,1), pathRRT.States(:,2),'r:','LineWidth',2);
hold all;
plot(traceRRT(:,1), traceRRT(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);
legend('Planned Path', 'Robot Trajectoy','location','northwest');
title('RRT')


%%%%%%%%%%%%%%%%%%%%%%%%%% A* %%%%%%%%%%%%%%%%%%%%%%%%%%

startGrid = world2grid(map, start(1:2));
goalGrid = world2grid(map, goal(1:2));

plannerA = plannerAStarGrid(mapInfl);
% plannerA = plannerAStarGrid(mapInfl2);

pathAStar = plan(plannerA, startGrid, goalGrid);

pathWaypoints = grid2world(map, pathAStar);

% Initialise Controller for A*
v = 2;
% v=1.25;
release(controller);
controller.Waypoints = pathWaypoints(:,1:2);
controller.LookaheadDistance = 0.5*MinTurningRadius;
controller.DesiredLinearVelocity = v;
controller.MaxAngularVelocity = v/MinTurningRadius;

robotCurrentPose = initPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));

traceAStar = initPose;

j = 1;

while distanceToGoal > goalRadius
    
    j = j+1;
    
    % Action inputs computation
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    
    % Simulate movement of car
    [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
    robotCurrentPose = pose(end,:);
    
    traceAStar = [traceAStar; robotCurrentPose];
    
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));
    
    % Abort if doesn't converge within 30 seconds
    if j > 30/sampleTime
        break
    end
    
end

% Interpolate A*
xIn = 1:length(pathWaypoints);
xInq = 1:0.1:length(pathWaypoints);
pathAStarInpl = interp1(xIn, pathWaypoints, xInq);

figure;
show(map);
hold all;
plot(pathAStarInpl(:,1), pathAStarInpl(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);
hold all;
plot(pathWaypoints(:,1), pathWaypoints(:,2),'r:','LineWidth',2);
legend('Interpolated Path', 'Orgional A* Path','location','northwest');

closestPurePursuitAStar = [];
errorAStar = [];

% Measure Error of A*
for i = 1:length(pathAStarInpl)
    distances = pdist2(traceAStar(:,1:2), pathAStarInpl(i,1:2));
    minD = min(distances);
    [row, column] = find(distances == minD);
    closestPurePursuitAStar = [closestPurePursuitAStar; traceAStar(row,1:2)];
    errorAStar = [errorAStar; distances(row(1))];
end

averageErrorAStar = sum(errorAStar)/length(errorAStar);
maxErrorAStar = max(errorAStar);

figure;
show(map);
hold all;
plot(pathWaypoints(:,1), pathWaypoints(:,2),'r:','LineWidth',2);
hold all;
plot(traceAStar(:,1), traceAStar(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);
legend('Planned Path', 'Robot Trajectoy','location','northwest');
title('A*')


averageErrorRRT
maxErrorRRT
averageErrorAStar
maxErrorAStar


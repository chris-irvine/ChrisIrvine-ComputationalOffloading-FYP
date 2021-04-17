clear;

% Map Parameters
mapSize = 20;
resolution = 1;
numObstacles = 2;

% Parameters of Simple car
carLength = 0.5;
psiMax = deg2rad(30);
MinTurningRadius = carLength/tan(psiMax);


map = binaryOccupancyMap(mapSize, mapSize, resolution);
% setOccupancy(map, [19,19; 20,19; 19,20], 1); % uncomment to block goal

% Ensures repeatability
rng('default')

% Generate RRT parameters
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = MinTurningRadius;
stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.2; 

RRT = plannerRRT(ss, stateValidator);
RRT.MaxConnectionDistance = 2; % User Defined
RRT.MaxIterations = 1500; % User Defined

start = [0.5, 0.5, 0];
goal = [19.5, 19.5, 0];

tic
[pathRRT, solnInfo] = plan(RRT, start, goal);
toc

figure;
show(map)
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
hold on;
plot(pathRRT.States(:,1),pathRRT.States(:,2),'r-','LineWidth',2);
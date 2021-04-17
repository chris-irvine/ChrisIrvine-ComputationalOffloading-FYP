clear;

% Map Parameters
mapSize = 20;
resolution = 1;
numObstacles = 2;

% Parameters of Simple car
carLength = 0.5;
psiMax = deg2rad(30);
MinTurningRadius = carLength/tan(psiMax);
v = 2;


% Keep seed constant for repeatability
rng('default')
map = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
rng('default')
mapInfl = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
inflate(mapInfl, 1);

rng('default')

% RRT Path Plan
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 1;
stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = mapInfl;
stateValidator.ValidationDistance = 0.2;

RRT = plannerRRTStar(ss, stateValidator);
RRT.MaxConnectionDistance = 2.0;
RRT.MaxIterations = 30000;

start = [0.5, 0.5, 0];
goal = [18.5, 18.5, 0];

path = plan(RRT, start, goal);
interDist = 0.01;
pathLen = pathLength(path);
interSize = ceil(pathLen/interDist);
interpolate(path, interSize);

% Initialise Controller
controller = controllerPurePursuit;
controller.LookaheadDistance = 0.75*MinTurningRadius;
controller.Waypoints = path.States(:,1:2);
controller.DesiredLinearVelocity = v;
controller.MaxAngularVelocity = v/MinTurningRadius;

initPose = [start, 0];

robotCurrentPose = initPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));

trace1 = initPose(1:2);

% Set Sample Time
sampleTime = 0.005;

goalRadius = 0.2;

while distanceToGoal > goalRadius
    % Action inputs computation
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    
    % Euler Forward Method
    dx = v*cos(robotCurrentPose(3));
    dy = v*sin(robotCurrentPose(3));
    dtheta = v*tan(robotCurrentPose(4))/carLength;
    dpsi = psiDot;
    diff = [dx, dy, dtheta, dpsi];
    robotCurrentPose = robotCurrentPose + diff*sampleTime;
    
    trace1 = [trace1; robotCurrentPose(1:2)];
    
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));
    
end


% Release for new input
release(controller);

initPose = [start, 0];
robotCurrentPose = initPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));

trace2 = initPose(1:2);

% Set Sample Time
sampleTime = 0.1;

goalRadius = 0.2;

while distanceToGoal > goalRadius
    % Action inputs computation
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    
    % Euler Forward Method
    dx = v*cos(robotCurrentPose(3));
    dy = v*sin(robotCurrentPose(3));
    dtheta = v*tan(robotCurrentPose(4))/carLength;
    dpsi = psiDot;
    diff = [dx, dy, dtheta, dpsi];
    robotCurrentPose = robotCurrentPose + diff*sampleTime;
    
    trace2 = [trace2; robotCurrentPose(1:2)];
    
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));
    
end


figure;
show(map);
hold all;
plot(path.States(:,1), path.States(:,2),'r:','LineWidth',2);
hold all;
plot(trace1(:,1), trace1(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);
hold all;
plot(trace2(:,1), trace2(:,2), 'lineWidth',2, 'color', [0 0.4470 0.7410]);
legend('Planned Path', 'Sample Time = 0.005', 'Sample Time = 0.1', 'Location','northwest');


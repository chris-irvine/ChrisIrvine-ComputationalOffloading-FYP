clear;

% Map Parameters
mapSize = 20;
resolution = 1;
numObstacles = 2;

% Parameters of Simple car
carLength = 0.5;
psiMax = deg2rad(30);
MinTurningRadius = carLength/tan(psiMax);


% Keep seed constant for repeatability
rng('default')
map = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
rng('default')
mapInfl = mapClutter(numObstacles,{'Box','Plus'},'MapSize',[mapSize mapSize], 'MapResolution', resolution);
inflate(mapInfl, 1);

% Keep seed constant for repeatability
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

path = plan(RRT, start, goal);
interDist = 0.01;
pathLen = pathLength(path);
interSize = ceil(pathLen/interDist);
interpolate(path, interSize);

initPose = [start, 0];


robotCurrentPose = initPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));

traceSimple = initPose;

% Discrete controller rate
sampleTime = 0.1;
tspan = [0 sampleTime];

goalRadius = 0.2;

%%%%%%%%%%%%%%%%%%%%%%%%%% Simple Closed Loop %%%%%%%%%%%%%%%%%%%%%%%%%%

% Selection of intermediate waypoints to converge to
step = round(interSize*interDist);

for j = (1+step):step:path.NumStates
    
    % Ensure last waypoint is goal
    if j + step >= path.NumStates
        j = path.NumStates;
    end
    
    robotGoal = path.States(j,1:2);
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    while distanceToGoal > goalRadius

        % Action inputs computation
        v = 5*norm(robotGoal - robotCurrentPose(1:2));
        C = atan2((robotGoal(2)-robotCurrentPose(2)), (robotGoal(1)-robotCurrentPose(1)));
        w = 2*wrapToPi(C-robotCurrentPose(3));
        psiPlus1 = atan((w*carLength)./v);
        % Saturate to max steering angle
        if abs(psiPlus1) > psiMax
            psiPlus1 = sign(psiPlus1)*psiMax;
        end
        psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
        
        % Simulate movement of car
        [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
        robotCurrentPose = pose(end,:);
        
        traceSimple = [traceSimple; robotCurrentPose];
        
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
        
    end
       
end


figure;
show(map);
hold all;
plot(path.States(:,1), path.States(:,2),'r:','LineWidth',2);
hold all;
plot(traceSimple(:,1), traceSimple(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);


%%%%%%%%%%%%%%%%%%%%%%%%%% Pure Pursuit %%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialise Pure Pursuit Controller
v = 2;
controller = controllerPurePursuit;
controller.Waypoints = path.States(:,1:2);
controller.LookaheadDistance = 0.5*MinTurningRadius;
controller.DesiredLinearVelocity = v;
controller.MaxAngularVelocity = v/MinTurningRadius;

robotCurrentPose = initPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));

tracePure = initPose;

while distanceToGoal > goalRadius
    
    % Action inputs computation
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    
    % Simulate movement of car
    [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
    robotCurrentPose = pose(end,:);
    
    tracePure = [tracePure; robotCurrentPose];
    
    distanceToGoal = norm(robotCurrentPose(1:2) - goal(1:2));
    
end

figure;
show(map);
hold all;
plot(path.States(:,1), path.States(:,2),'r:','LineWidth',2);
hold all;
plot(tracePure(:,1), tracePure(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);


%%%%%%%%%%%%%%%%%%%%%%%%%% Simple Open Loop %%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate time of arrival
timeOfArival = 5;
stepSize = timeOfArival/path.NumStates;
t = linspace(0, timeOfArival, path.NumStates);
sampleTimeOpen = t(2);

waypoints = [path.States(:,1:2), zeros(path.NumStates, 1)];
theta = [path.States(:,3), zeros(path.NumStates, 2)];
orientation = quaternion(theta, 'euler','ZYX','frame');

traj = waypointTrajectory(waypoints, 'TimeOfArrival', t, 'Orientation', orientation);

[position,orient,vel,acc,angvel] = lookupPose(traj,t);

v = sqrt(vel(:,1).^2 + vel(:,2).^2); % x and y velocity conversion
w = angvel(:,3);

psiPlus1 = atan((w*carLength)./v);
for l = 1:length(psiPlus1)
    % Saturate to max steering angle
    if abs(psiPlus1(l)) > psiMax
        psiPlus1(l) = sign(psiPlus1(l))*psiMax;
    end
end

psi = 0;
psiDot = zeros(length(w),1);
for n = 1:length(psiPlus1)
    psiDot(n) = (psiPlus1(n) - psi)/sampleTimeOpen;
    psi = psiPlus1(n);
end

robotCurrentPose = [path.States(1,:), 0];

traceOpen = robotCurrentPose;

tSpanOpen = [0 sampleTimeOpen];

for k = 1:length(v)
    
    [t, pose] = ode45(@(t,y) ackermann(y, carLength, v(k), psiDot(k)), tSpanOpen, robotCurrentPose);
    robotCurrentPose = pose(end,:);
    
    traceOpen = [traceOpen; robotCurrentPose];
end

figure;
show(map);
hold all;
plot(path.States(:,1), path.States(:,2),'r:','LineWidth',2);
hold all;
plot(traceOpen(:,1), traceOpen(:,2), 'lineWidth',2, 'color', [0.4940 0.1840 0.5560]);


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

initPose = [start, 0];

robotCurrentPose = initPose;
sampleTime = 0.1;


%%%%%%%%%%%%%%%%%%%%%%%%%% Simple %%%%%%%%%%%%%%%%%%%%%%%%%%

step = round(interSize*interDist);
robotGoal = path.States(step,1:2);

tSimple = zeros(10,1);
for i = 1:10
    tic;
    v = 2*norm(robotGoal - robotCurrentPose(1:2));
    C = atan2((robotGoal(2)-robotCurrentPose(2)), (robotGoal(1)-robotCurrentPose(1)));
    w = 20*wrapToPi(C-robotCurrentPose(3));
    
    psiPlus1 = atan((w*carLength)./v);
    if abs(psiPlus1) > psiMax
        psiPlus1 = sign(psiPlus1)*psiMax;
    end
    tSimple(i) = toc;
end

tSimpleAvg = sum(tSimple)/length(tSimple);


%%%%%%%%%%%%%%%%%%%%%%%%%% Pure Pursuit %%%%%%%%%%%%%%%%%%%%%%%%%%

v = 2;
controller = controllerPurePursuit;
controller.Waypoints = path.States(:,1:2);
controller.LookaheadDistance = 0.75*MinTurningRadius;
controller.DesiredLinearVelocity = v;
controller.MaxAngularVelocity = v/MinTurningRadius;


tPure = zeros(10,1);
for i = 1:10
    tic;
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    tPure(i) = toc;
end
tPureAvg = sum(tPure)/length(tPure);


%%%%%%%%%%%%%%%%%%%%%%%%%% Open Loop %%%%%%%%%%%%%%%%%%%%%%%%%%


timeOfArival = 10;
stepSize = timeOfArival/path.NumStates;
t = linspace(0, timeOfArival, path.NumStates);
sampleTimeOpen = t(2);

tic;
waypoints = [path.States(:,1:2), zeros(path.NumStates, 1)];

theta = [path.States(:,3), zeros(path.NumStates, 2)];
orientation = quaternion(theta, 'euler','ZYX','frame');

traj = waypointTrajectory(waypoints, 'TimeOfArrival', t, 'Orientation', orientation);

[position,orient,vel,acc,angvel] = lookupPose(traj,t);

v = sqrt(vel(:,1).^2 + vel(:,2).^2); % x and y velocity conversion
w = angvel(:,3);

psiPlus1 = atan((w*carLength)./v);
for l = 1:length(psiPlus1)
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
tOpen = toc;






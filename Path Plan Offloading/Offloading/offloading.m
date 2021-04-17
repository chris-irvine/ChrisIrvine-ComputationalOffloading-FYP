clear;

% Map Parameters
mapSize = 20;
resolution = 1;
tNet = 0.01;

% Parameters of Simple car
carLength = 0.5;
psiMax = deg2rad(30);
MinTurningRadius = carLength/tan(psiMax);

% Initialise Parameters to reduce computation time
time = 0;
trace= zeros(400,4);
tRRTPdct = zeros(40,1);
tAStarPdct = zeros(40,1);
tPathPlanM = zeros(40,1);
time2goalRRTPdct = zeros(40,1);
time2goalAStarPdct = zeros(40,1);
cPdct = zeros(40,1);
offDec = zeros(40,1);

% Generate workload asscoaited parameter a
rng('shuffle');
Y = 5*randi(5, 1,40);
x = randi(3, 1,40);
X = [0, cumsum(x)];
Y = [Y, Y(end)];
xb = X(1):0.1:(X(end)-0.1);
repeat = x/0.1;
yb = repelem(Y(1:end-1),repeat) + rand(1,(X(end)/0.1));


% Draw Map
obstacles = [1,8; 1,9; 1,10; 1,11; 1,12; 1,13; 2,8; 2,9; 2,10; 2,11; 2,12; 2,13; ...
             20,10; 19,10; 18,10; 17,10; 16,10; 20,11; 19,11; 18,11; 17,11; 16,11; ...
             20,9; 19,9; 18,9; 17,9; 16,9; 20,8; 19,8; 18,8; 17,8; 16,8; ...
             15,8; 15,9; 15,10; 15,11];
         
map = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
setOccupancy(map, obstacles, 1);
matMap = occupancyMatrix(map);

% can't use mapinfl = map as inflates map as well
mapinfl = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
setOccupancy(mapinfl, obstacles, 1);
inflate(mapinfl, 1);
matMapinfl = occupancyMatrix(mapinfl);

mapinfl2 = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
setOccupancy(mapinfl2, obstacles, 1);
inflate(mapinfl2, 1);
inflate(mapinfl2, 1);
matMapinfl2 = occupancyMatrix(mapinfl2);

% Path Plan parameters for RRT
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = MinTurningRadius;
stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = mapinfl;
stateValidator.ValidationDistance = 0.2;

% Initialise RRT Planner
plannerRRT = plannerRRT(ss, stateValidator);
plannerRRT.MaxConnectionDistance = 2.0;
plannerRRT.MaxIterations = 30000;

% Initialise A* Planner
plannerA = plannerAStarGrid(mapinfl2);

start = [0.5, 0.5];
startXY = [start, 0];
goal = [19, 19];
goalXY = [19, 19, 0];

initPose = [startXY, 0];

a = yb(1);

% Offload Decision
[offload, tAStar, tRRT, time2goalRRT, time2goalAStar, c] = offloadDecision(start, goal, mapinfl, a, initPose);

% Path Plan
[pathWaypoints, pathFound, tPathPlan] = planPath(offload, startXY, goalXY, plannerRRT, plannerA, mapinfl, mapinfl2,a);

% Assign measurement Parameters
tRRTPdct(1) = tRRT;
tAStarPdct(1) = tAStar;
tPathPlanM(1) = tPathPlan;
offDec(1) = offload;
time2goalRRTPdct(1) = time2goalRRT;
time2goalAStarPdct(1) = time2goalAStar;
cPdct(1) = c;

% Abort if no Path Feasible
if pathFound == 0
    disp('No Feasible Path Found');
    return;
end

% Initialise Controller Depending of Offload Decision
controller = controllerPurePursuit;
controller.LookaheadDistance = 0.5*MinTurningRadius;
controller.Waypoints = pathWaypoints(:,1:2);
if offload == 1
    v=2;
    controller.DesiredLinearVelocity = v;
    controller.MaxAngularVelocity = v/MinTurningRadius;
else
    v=1.25;
    controller.DesiredLinearVelocity = v;
    controller.MaxAngularVelocity = v/MinTurningRadius;
end



initPose = [startXY, 0];
goalRadius = 0.5;

% Sample Time and Visibility Rate
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
tspan = [0 sampleTime];

robotCurrentPose = initPose;
trace(1,:) = robotCurrentPose;
distanceToGoal = norm(robotCurrentPose(1:2) - goal(:));

%Initialise Car
car = CarBot(robotCurrentPose);
car.L = carLength;

% Flag for Goal change
nonrepeat = 0;

% Draw Figure
figure;
plotAnimation(map, pathWaypoints(:,1:2),trace(1,:), offload);
hold all;
car.drawCarBot;
legend('Planned Route', 'Traced Path', 'Car Centre Rear Axis', 'Location', 'northeastoutside');
initialiseText(offload,tRRT,tAStar, a, time2goalRRT, time2goalAStar, c);

% Initialise DYnamic Obstacle 
x = 10;
py = 1;
sgn = 1;
rate = 1;
obsSpeed = 6; % low is high and high is low

i = 1;
j = 1;

reset(vizRate);

while distanceToGoal > goalRadius || time < 40
    
    i = i+1;
    
    % Control and Inflate Dynamic obstacle position
    if rem(rate, obsSpeed) == 0
        if py >= 20 || (sgn == -1 && py <= 1)
            sgn = sgn*(-1);
        end
        ny = py + 1*sgn;
        setOccupancy(map, matMap);
        setOccupancy(map, [x ny], 1);
        setOccupancy(mapinfl, matMapinfl);
        pos = singleInfl(x,ny);
        setOccupancy(mapinfl, pos, 1);
        setOccupancy(mapinfl2, matMapinfl2);
        pos2 = singleInfl(pos(:,1),pos(:,2));
        setOccupancy(mapinfl2, pos2, 1);    
        py = ny;
    end
    rate = rate + 1;
    
    % Check future plan is safe
    replan = pathValidCheck(offload, stateValidator, pathWaypoints, mapinfl, mapinfl2, robotCurrentPose);
    
    % Dynamic Goal state change
    if robotCurrentPose(1) >= 15 && nonrepeat == 0
        goalXY = [4.5,4.5,0];
        replan = 1;
        nonrepeat = 1;
    end
    
    % Initiate replan if flag is high
    if replan
        % Take Current state of environment and robot and update planners 
        startXY = robotCurrentPose(1:3);
        stateValidator.Map = mapinfl;
        plannerRRT = plannerRRTStar(ss, stateValidator);
        plannerRRT.MaxConnectionDistance = 2.0;
        plannerRRT.MaxIterations = 30000;
        
        plannerA = plannerAStarGrid(mapinfl2);
        
        % Take measurement of a based on time
        a = yb(1+int16(time/0.1));
        
        % Offload Decision
        [offload, tAStar, tRRT, time2goalRRT, time2goalAStar, c] = offloadDecision(startXY(1:2), goalXY(1:2), mapinfl, a, robotCurrentPose);
        
        % Path Plan
        [pathWaypoints, pathFound, tPathPlan] = planPath(offload, startXY, goalXY, plannerRRT, plannerA, mapinfl, mapinfl2, a);
        
        % Input Measurement Parameters
        j = j + 1;
        tRRTPdct(j) = tRRT;
        tAStarPdct(j) = tAStar;
        tPathPlanM(j) = tPathPlan;
        offDec(j) = offload;
        time2goalRRTPdct(j) = time2goalRRT;
        time2goalAStarPdct(j) = time2goalAStar;
        cPdct(j) = c;
        
        % Raise flag if no path found
        if pathFound == 0
            disp('Abort No Feasible Safe Path Found');
            flag = 1;
            break;
        end
        
        % Update Controller
        release(controller);
        controller.Waypoints = pathWaypoints(:,1:2);
        
        if offload == 1
            v=2;
            controller.DesiredLinearVelocity = v;
            controller.MaxAngularVelocity = v/MinTurningRadius;
        else
            v=1.25;
            controller.DesiredLinearVelocity = v;
            controller.MaxAngularVelocity = v/MinTurningRadius;
        end

        replan = 0;
        
    end
    
    % Action inputs computation
    [v, w] = controller(robotCurrentPose(1:3));
    psiPlus1 = atan((w*carLength)./v);
    psiDot = (psiPlus1 - robotCurrentPose(4))/sampleTime;
    
    % Simulate Movement of car
    [t, pose] = ode45(@(t,y) ackermann(y, carLength, v, psiDot), tspan, robotCurrentPose);
    robotCurrentPose = pose(end,:);
    
    distanceToGoal = norm(robotCurrentPose(1:2) - goalXY(1:2));
    
    car.Pose = robotCurrentPose;
    trace(i,:) = robotCurrentPose;
    
    % Update Figure
    plotAnimation(map,pathWaypoints(:,1:2),trace(1:i,:), offload);
    car.drawCarBot;
    legend('Planned Route', 'Traced Path', 'Car Centre Rear Axis', 'Location', 'northeastoutside');
    initialiseText(offload,tRRT,tAStar, a, time2goalRRT, time2goalAStar, c);
    
    % Check for collision or breach of bounds
    flag = checkConstraints(map, robotCurrentPose, bounds, carLength, car);
    
    % Abort if collision flag raised
    if flag
        break;
    end
    
    % Update time
    time = time + 0.1;
    
    waitfor(vizRate);
end

% Display if couldn't converge
if time >= 40
    disp('Failed to Converge to Goal within time limit')
end

% Process Measurements for analysis
tRemotePrdct = (tRRTPdct(1:j,:));
tLocalPrdct =  tAStarPdct(1:j,:);
tPathPlan =  tPathPlanM(1:j,:);
LHS = tRemotePrdct + tNet;
b = time2goalAStarPdct(1:j,:)./(time2goalRRTPdct(1:j,:)*2500);
cM = cPdct(1:j,:);
RHS = tLocalPrdct + b + cM;

figure;
plot(tLocalPrdct, 'g-o');
hold on;
plot(tRemotePrdct, 'r-o');
xlim([0.75,length(tRemotePrdct)+0.25]);
title('Comparison of Predicted Remote and Local Computation Time to Plan Path');
xlabel('Scenario');
ylabel('Time (s)');
legend('Local', 'Remote','Location', 'northeastoutside');
ax = gca;
ax.XTick = unique(round(ax.XTick));

figure;
plot(RHS, 'g-o');
hold on;
plot(LHS, 'r-o');
xlim([0.75,length(tRemotePrdct)+0.25]);
title('Comparison of Predicted Remote and Local Computation Time Full Model');
xlabel('Scenario');
ylabel('Time (s)');
legend( 'tAStarLocal + b + c', 'tNet + tRRTRemote', 'Location', 'northeastoutside');
ax = gca;
ax.XTick = unique(round(ax.XTick));

tdiff = tLocalPrdct-tRemotePrdct;
figure;
plot(tdiff, '-o');
title('Difference between Local and Remote Predicted Computation Time');
legend('tLocal - tRemote');
ax = gca;
ax.XTick = unique(round(ax.XTick));
xlim([0.75,length(tRemotePrdct)+0.25]);
xlabel('Scenario');
ylabel('Time (s)');

% Allign computation time with prediction time based on offload decision
predict = zeros(length(tPathPlan),1);
for k = 1:length(tPathPlan)
    if offDec(k) == 1
        predict(k) = tRemotePrdct(k);
    else
        predict(k) = tLocalPrdct(k);
    end
end

error = predict - tPathPlan;
figure;
plot(error, '-o');
title('Error between Predicted Computation Time and Actual Computation Time');
legend('tPredict - tPathPlan');
ax = gca;
ax.XTick = unique(round(ax.XTick));
xlabel('Scenario');
ylabel('Time (s)');
xlim([0.75,length(tRemotePrdct)+0.25])
ax = gca;
ax.XTick = unique(round(ax.XTick));

figure;
plot(xb,yb);
ylim([0,30]);
xlim([0,ceil(time)]);
title('Resources available in Edge Server');
xlabel('Time(s)');
ylabel('a');

figure;
stem(offDec)
xlim([0.75,length(tRemotePrdct)+0.25])
ylim([0,1.5]);
ax = gca;
ax.XTick = unique(round(ax.XTick));
ax = gca;
ax.YTick = unique(round(ax.YTick));
xlabel('Scenario');
ylabel('Offload Decision');
title('Offload Decision Results');


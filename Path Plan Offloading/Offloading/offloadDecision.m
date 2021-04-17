function [offload, tAStar, tRRT, time2goalRRT, time2goalAStar, c] = offloadDecision(start, goal, mapinfl, a, robotCurrentPose)

startGrid = world2grid(mapinfl,start);
goalGrid = world2grid(mapinfl,goal);

% Generate Independent value
dist = norm(start - goal);

matMap = occupancyMatrix(mapinfl);
szMap = size(matMap);

limitsRow = sort([startGrid(1), goalGrid(1)]);
limitsCol = sort([startGrid(2), goalGrid(2)]);
%round up lower so as to avoid a 0 value
upperLimitRow = ceil(limitsRow(2));
lowerLimitRow = ceil(limitsRow(1));
upperLimitCol = ceil(limitsCol(2));
lowerLimitCol = ceil(limitsCol(1));
square = matMap(lowerLimitRow:upperLimitRow, lowerLimitCol:upperLimitCol);
szSQ = size(square);

obsDensityRatioMAP = sum(matMap, 'all')/(szMap(1)*szMap(2));
obsDensityRatioSQ = sum(square, 'all')/(szSQ(1)*szSQ(2));
obs = obsDensityRatioMAP + obsDensityRatioSQ;

% Take direction from robot position to goal
theta = atan2(goal(2) - start(2), goal(1) - start(1));

% Wrap robot orientation to within pi and -pi
robotDir = wrapToPi(robotCurrentPose(3));

% if change in direction is more than 100 degrees prioritise RRT
if abs(robotDir-theta) > 1.7453
    c = 0.001;
else
    c = 0;
end


tNet = 0.01;

tAStar = (0.00005216*obs + 0.000004956*dist + 0.001069)*10;
tRRT = (0.009445*obs + 0.001336*dist + 0.002401)/a;

time2goalRRT = 2.074*obs + 0.5408*dist + 1.528;
time2goalAStar = 1.072*obs + 0.825*dist + 0.09728;
b = time2goalAStar/(time2goalRRT*2500);

% Offload decision high or low
offload = (tNet + tRRT)< tAStar + b + c;

end


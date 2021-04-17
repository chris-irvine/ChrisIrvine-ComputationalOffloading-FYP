clear;

data = struct();

% Characterise Resting Parameters
maps = [20, 40];
loops = 10;
denistyObst = [0.1 0.25];
resolution = 1;

field = 1;

for i = 1:loops
    
    for a = 1:length(maps)
        mapSize = maps(a);
        cells = mapSize^2;
        
        for b = 1:length(denistyObst)
            
            data(field).MapSize = mapSize;
            
            obsD = denistyObst(b);
            obstacles = round(cells*obsD);
            
            data(field).Obstacles = obstacles;
            
            
            columnObstacle = randi(mapSize, obstacles, 1);
            rowObstacle = randi(mapSize, obstacles, 1);
            
            map = binaryOccupancyMap(mapSize, mapSize, resolution, "grid");
            setOccupancy(map, [rowObstacle columnObstacle], 1);
            
            matMap = occupancyMatrix(map);
            data(field).actualDensity = length(find(matMap))/cells;
            
            % Uncomment to show maps
            %         figure;
            %         show(map);
            
            
            % Initialise and check validitiy of start and goal state
            startColumn = randi(mapSize);
            startRow = randi(mapSize);
            goalColumn = randi(mapSize);
            goalRow = randi(mapSize);
            
             while getOccupancy(map, [startRow startColumn], "grid") == 1
                 startColumn = randi(mapSize);
                 startRow = randi(mapSize);
             end
             
             while getOccupancy(map, [goalRow goalColumn], "grid") == 1
                 startColumn = randi(mapSize);
                 goalRow = randi(mapSize);
             end
             
             while startRow == goalRow && startColumn == goalColumn
                 startColumn = randi(mapSize);
                 startRow = randi(mapSize);
                 while getOccupancy(map, start(i,:), "grid") == 1
                     startColumn = randi(mapSize);
                     startRow = randi(mapSize);
                 end
             end
            
            
            
            %%%%%%%%%%%%%% A* %%%%%%%%%%%%%%
            
            plannerA = plannerAStarGrid(map);
            
            startAStar = [startRow, startColumn]; %grid locations
            goalAStar =  [goalRow, goalColumn];
            tic;
            [pthAStar, solnInfoAStar] = plan(plannerA, startAStar, goalAStar);
            data(field).timeAStar = toc;
            
            distAStar = 0;
            for k = 1:(size(pthAStar,1) - 1)
                X = pthAStar(k,1) - pthAStar(k+1,1);
                Y = pthAStar(k,2) - pthAStar(k+1,2);
                distAStar = distAStar + sqrt(X^2 + Y^2);
            end
            
            data(field).distAStar = distAStar;

            
            %%%%%%%%%%%%%% RRT %%%%%%%%%%%%%%
            
            % Duplicate and tranfsorm to match Grid
            map2 = binaryOccupancyMap(mapSize + 0.5, mapSize + 0.5, resolution * 2);
            RRTRowObstacle = [rowObstacle; rowObstacle + 0.5; rowObstacle; rowObstacle + 0.5];
            RRTColumnObstacle = [columnObstacle; columnObstacle; columnObstacle + 0.5; columnObstacle + 0.5];
            setOccupancy(map2, [RRTRowObstacle RRTColumnObstacle], 1);
            
            bounds = [map2.XWorldLimits + [0.5 0]; map2.YWorldLimits + [0.5 0]; [-pi pi]];
            ss = stateSpaceDubins(bounds);
            stateValidator = validatorOccupancyMap(ss);
            stateValidator.Map = map2;
            stateValidator.ValidationDistance = 0.2;
            
            % transform to match grid
            startX = startColumn;
            startY = mapSize - startRow + 1;
            goalX = goalColumn;
            goalY = mapSize - goalRow + 1;
            
            startXY = [startX, startY, 0];
            goalXY = [goalX, goalY, 0];
            
            RRT = plannerRRT(ss, stateValidator);
            RRT.MaxConnectionDistance = 2.0;
            RRT.MaxIterations = 30000;
            
            
            tic;
            [pthRRT, solnInfoRRT] = plan(RRT, startXY, goalXY);
            data(field).timeRRT = toc;
            if solnInfoRRT.IsPathFound % only interpolate if solution to avoid error
                pathLen = pathLength(pthRRT);
                data(field).distRRT = pathLen;
                interDist = 0.01;
                interSize = ceil(pathLen/interDist);
                interpolate(pthRRT, interSize);
                RRTpathMetrics = pathmetrics(pthRRT);
                data(field).smoothnessRRT = smoothness(RRTpathMetrics);
            else
                disp('RRT No solution')
                data(field).distRRT = 0;
                data(field).smoothnessRRT = 0;
            end            
            
            
            %%%%%%%%%%%%%% RRT* %%%%%%%%%%%%%%
            
            RRTStar = plannerRRT(ss, stateValidator);
            RRTStar.MaxConnectionDistance = 2.0;
            RRTStar.MaxIterations = 30000;
            
            tic;
            [pthRRTStar, solnInfoRRTStar] = plan(RRTStar, startXY, goalXY);
            data(field).timeRRTStar = toc;
            if solnInfoRRTStar.IsPathFound % only interpolate if solution to avoid error
                pathLen = pathLength(pthRRTStar);
                data(field).distRRTStar = pathLen;
                interDist = 0.01;
                interSize = ceil(pathLen/interDist);
                interpolate(pthRRTStar, interSize);
                RRTStarpathMetrics = pathmetrics(pthRRTStar);
                data(field).smoothnessRRTStar = smoothness(RRTStarpathMetrics);
            else
                disp('RRT* No solution')
                data(field).distRRTStar = 0;
                data(field).smoothnessRRTStar = 0;
            end
            
            
            % Uncomment to show solutions
%             figure;
%             subplot(1,3,1);
%             show(plannerA);
%             grid on;
%             grid minor;
%             
%             subplot(1,3,2);
%             show(map2);
%             xlim([0.5 (mapSize + 0.5)]);
%             ylim([0.5 (mapSize + 0.5)]);
%             hold on;
%             plot(solnInfoRRT.TreeData(:,1),solnInfoRRT.TreeData(:,2), '.-'); % tree expansion
%             plot(pthRRT.States(:,1),pthRRT.States(:,2),'r-','LineWidth',2); % draw path
%             grid on;
%             grid minor;
%             title('RRT')
%             
%             subplot(1,3,3);
%             show(map2);
%             xlim([0.5 (mapSize + 0.5)]);
%             ylim([0.5 (mapSize + 0.5)]);
%             hold on;
%             plot(solnInfoRRTStar.TreeData(:,1),solnInfoRRTStar.TreeData(:,2), '.-'); % tree expansion
%             plot(pthRRTStar.States(:,1),pthRRTStar.States(:,2),'r-','LineWidth',2); % draw path
%             grid on;
%             grid minor;
%             title('RRT*')
            
            
            field = field + 1; % Update position in struct
        end
    end
    
end

% Format data acuired
dataTable = struct2table(data);
rrtT = dataTable.timeRRT;
rrtST = dataTable.timeRRTStar;
rrtD = dataTable.distRRT;
rrtSD = dataTable.distRRTStar;
AStarT = dataTable.timeAStar;
AStarD = dataTable.distAStar;

figure;
plot(rrtT);
hold on;
plot(rrtST);
hold on;
plot(AStarT);
legend( 'RRT', 'RRT*', 'A*', 'Location', 'northeastoutside');
title('Comparison of Time to Plan Path for A*, RRT and RRT*')
xlabel('Instance');
ylabel('Time(s)');


figure;
plot(rrtD);
hold on;
plot(rrtSD);
hold on;
plot(AStarD);
legend( 'RRT', 'RRT*', 'A*', 'Location', 'northeastoutside');
title('Comparison of Path Length for A*, RRT and RRT*');
xlabel('Instance');
ylabel('Distance(m)');



function [rowObstacle, columnObstacle, startColumn, startRow, goalColumn, goalRow] = MazeGenerator(mapSize, gap, type)

    if type == 'spiral'
        [rowObstacle, columnObstacle, startColumn, startRow, goalColumn, goalRow] = spiral(mapSize, gap);
    elseif type == 'zigzag'
        [rowObstacle, columnObstacle, startColumn, startRow, goalColumn, goalRow] = zigzag(mapSize, gap);
    else
        rowObstacle = 0;
        columnObstacle = 0;
    end
end

function [rowVector, colVector, startColumn, startRow, goalColumn, goalRow] = spiral(mapSize, gap)
    
    rowVector = [];
    colVector =[];
    
    %1
    row = (gap+1)*ones(1,mapSize-gap);
    col = 1:mapSize-gap;
    rowVector = [rowVector; transpose(row)];
    colVector = [colVector; transpose(col)];
    
    %Right Verical
    for i = 1:(mapSize/2-gap)/(gap+1)
        col = (mapSize-(gap*i +(i-1)))*ones(1, mapSize-(gap*(i*2)+((i-1)*2)));
        row = (gap*i+1 +(i-1)):(mapSize-(gap*i +(i-1)));
        rowVector = [rowVector; transpose(row)];
        colVector = [colVector; transpose(col)];
    end
    
    %Bottom Horizontal
    for i = 1:(mapSize/2-gap)/(gap+1)
        row = (mapSize-(gap*i +(i-1)))*ones(1, mapSize-(gap*(i*2)+((i-1)*2)));
        col = (gap*i+1 +(i-1)):(mapSize-(gap*i +(i-1)));
        rowVector = [rowVector; transpose(row)];
        colVector = [colVector; transpose(col)];
    end
    
    %Left Verical
    for i = 1:(mapSize/2-gap)/(gap+1)
        col = (gap*i+1 +(i-1))*ones(1,(mapSize -(gap*(i*2+1) +(i*2-1))));
        row = (gap*(i+1)+1 +i):(mapSize - (gap*i +(i-1)));
        rowVector = [rowVector; transpose(row)];
        colVector = [colVector; transpose(col)];
    end
    
    %Top Horizontal
    for i = 1:(mapSize/2-gap)/(gap+1)
        row = (gap*(i+1)+1 +i)*ones(1,(mapSize-(gap*(i*2+1) + (i*2-1))));
        col = (gap*i+1 +(i-1)):(mapSize - (gap*(i+1) +i));
        rowVector = [rowVector; transpose(row)];
        colVector = [colVector; transpose(col)];
    end
    
    startColumn = 1;
    startRow = 1;
    goalColumn = mapSize/2 +2;
    goalRow = mapSize/2 +2;

end


function [rowVector, colVector, startColumn, startRow, goalColumn, goalRow] = zigzag(mapSize, gap)
    rowVector = [];
    colVector =[];
    
    % Alternate positionng of obstacles from left and right
    k=1;
    for i = 2:gap+1:mapSize
        if rem(k,2) == 0
            row = 1:mapSize-ceil(gap/2);
        else
            row = 1+ceil(gap/2):mapSize;
        end
        k=k+1;
        col = i*ones(1, mapSize-ceil(gap/2));
        rowVector = [rowVector; transpose(row)];
        colVector = [colVector; transpose(col)];
    end
    
    goalRow = mapSize;
    goalColumn = mapSize;
    
    %Move starting position based on gap from left or right of map
    if rem(gap, 2) ==0
        startColumn = 1;
        startRow = 1;
    else
        startColumn = mapSize;
        startRow = 1;
    end

end
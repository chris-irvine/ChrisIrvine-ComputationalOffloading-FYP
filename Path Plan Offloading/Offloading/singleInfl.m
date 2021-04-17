function pos = singleInfl(x,y)

% Inflate around specified position or positions
pos = [x-1,y+1; x,y+1; x+1,y+1;...
       x-1,y;   x,y;   x+1,y;...
       x-1,y-1; x,y-1; x+1,y-1];

end


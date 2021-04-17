function flag = checkConstraints(map, robotCurrentPose, bounds, L, car)

theta = robotCurrentPose(3);

corners = [robotCurrentPose(1:2)+[-sin(theta), cos(theta)]*L/2; ...
               robotCurrentPose(1:2)+[sin(theta), -cos(theta)]*L/2; ...
               robotCurrentPose(1:2)+[cos(theta)*L+sin(theta)*L/2, sin(theta)*L - cos(theta)*L/2]; ...
               robotCurrentPose(1:2)+[cos(theta)*L-sin(theta)*L/2, sin(theta)*L + cos(theta)*L/2]];

wheels = [car.HFw1.XData', car.HFw1.YData';...
         car.HFw2.XData', car.HFw2.YData';...
         car.HRw1.XData', car.HRw1.YData';...
         car.HRw2.XData', car.HRw2.YData'];

% Produce Middle of axle
frontAxleMid = [(corners(3,1)+corners(4,1))/2, (corners(3,2)+corners(4,2))/2];

%Combine all colission check points and see if an return wihtin obstacle
checkPos = [corners; frontAxleMid; robotCurrentPose(1:2); wheels];
collision = sum(getOccupancy(map, checkPos));

% Check if wheels have breached bounds
if any(wheels(:,1)<=bounds(1,1)) || any(wheels(:,1)>=bounds(1,2)) || any(wheels(:,2)<=bounds(2,1)) || any(wheels(:,2)>=bounds(2,2))
    disp('Car out of Bounds');
    flag = 1;
elseif collision >= 1 % Check if a collision
    disp('Collision');
    flag = 1;
else
    flag = 0; % No constraint breach return flag low
end

end


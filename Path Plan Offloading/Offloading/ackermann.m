function  dpose = ackermann(robotCurrentPose, L, v, w)

dx = v*cos(robotCurrentPose(3));
dy = v*sin(robotCurrentPose(3));
dtheta = v*tan(robotCurrentPose(4))/L;
dpsi = w;

dpose = [dx; dy; dtheta; dpsi];
 
end


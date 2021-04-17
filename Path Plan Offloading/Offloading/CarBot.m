classdef CarBot < handle
%EXAMPLEHELPERCARBOT Create a simulated car-like robot object 
% Altered by Chris Irvine

% Copyright 2015 The MathWorks, Inc.    

    properties(SetAccess = public)
        
        %Pose Current pose of the robot
        Pose
        
        %L wheelbase
        L = 0.2
        
        %HFw1 Graphics handle for front wheel 1
        HFw1
        
        %HFw2 Graphics handle for front wheel 2
        HFw2
        
        %HRw1 Graphics handle for rear wheel 1
        HRw1
        
        %HRw2 Graphics handle for rear wheel 2
        HRw2
        
    end
    
    properties(Access = private)  
        %HCenter Graphics handle for center of rear axle of the car robot
        HCenter
        
        %HTrajectory Graphics handle for the trajectory of the car
        HTrajectory
        
        %HChassis Graphics handle for chassis
        HChassis

    end
    
    
    methods
        function obj = CarBot(currentPose)
            %Constructor
            
            obj.Pose = currentPose;                    
        
        end


        function drawCarBot(obj)
            %drawCarBot Routine to draw the car-like robot
            r = obj.L/6; 
            p = obj.Pose;
            x = p(1);
            y = p(2);
            theta = p(3);
            phi = p(4);

            chassis = [x  + obj.L/2*cos(theta-pi/2), y + obj.L/2*sin(theta-pi/2); %back axle          
                   x + obj.L/2*cos(theta+pi/2), y + obj.L/2*sin(theta+pi/2);
                   x, y; % centre axle
                   x + obj.L*cos(theta), y + obj.L*sin(theta);
                   x + obj.L*cos(theta) + obj.L/2*cos(theta-pi/2), y + obj.L*sin(theta) + obj.L/2*sin(theta-pi/2); %front axle         
                   x + obj.L*cos(theta) + obj.L/2*cos(theta+pi/2), y + obj.L*sin(theta) + obj.L/2*sin(theta+pi/2);
                   x + obj.L*cos(theta), y + obj.L*sin(theta) %connect up last drawing
                   ];
            fw1 = [ x + obj.L*cos(theta) + obj.L/2*cos(theta-pi/2) + r*cos(theta+phi), y + obj.L*sin(theta) + obj.L/2*sin(theta-pi/2) + r*sin(theta+phi);
                      x + obj.L*cos(theta) + obj.L/2*cos(theta-pi/2) - r*cos(theta+phi), y + obj.L*sin(theta) + obj.L/2*sin(theta-pi/2) - r*sin(theta+phi)];         
            fw2 = [ x + obj.L*cos(theta) + obj.L/2*cos(theta+pi/2) + r*cos(theta+phi), y + obj.L*sin(theta) + obj.L/2*sin(theta+pi/2) + r*sin(theta+phi);
                      x + obj.L*cos(theta) + obj.L/2*cos(theta+pi/2) - r*cos(theta+phi), y + obj.L*sin(theta) + obj.L/2*sin(theta+pi/2) - r*sin(theta+phi)];

            rw1 = [ x  + obj.L/2*cos(theta-pi/2) + r*cos(theta), y  + obj.L/2*sin(theta-pi/2) + r*sin(theta);
                      x  + obj.L/2*cos(theta-pi/2) - r*cos(theta), y  + obj.L/2*sin(theta-pi/2) - r*sin(theta)];         
            rw2 = [ x  + obj.L/2*cos(theta+pi/2) + r*cos(theta), y  + obj.L/2*sin(theta+pi/2) + r*sin(theta);
                      x  + obj.L/2*cos(theta+pi/2) - r*cos(theta), y  + obj.L/2*sin(theta+pi/2) - r*sin(theta)];

            
             obj.HCenter = plot(obj.Pose(1), obj.Pose(2),'o');
             obj.HChassis = plot(chassis(:,1),chassis(:,2),'b'); % chassis
             obj.HFw1 = plot(fw1(:,1),fw1(:,2),'linewidth',3, 'color',[0.3010 0.7450 0.9330]); % front wheel 1
             obj.HFw2 = plot(fw2(:,1),fw2(:,2),'linewidth',3, 'color',[0.3010 0.7450 0.9330]); % front wheel 2
             obj.HRw1 = plot(rw1(:,1),rw1(:,2),'linewidth',3, 'color','b');         % rear wheel 1
             obj.HRw2 = plot(rw2(:,1),rw2(:,2),'linewidth',3, 'color','b');         % rear wheel 2
             
        end 
        
    end
    
end
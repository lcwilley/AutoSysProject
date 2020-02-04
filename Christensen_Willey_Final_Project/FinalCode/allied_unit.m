classdef allied_unit < ground_unit
    properties
        % Defined in ground_unit
        % X % Unit state
        % Xe % Estimated state
        % Xd % Desired state
        
        % Plotting variables
        % plotHandles % Unit plot handles
        % w % Plotting width
        % h % Plotting height
        
        % Control variables
        % alph % Motion noise
        % v % Linear velocity
        % om % Angular velocity
        % v_limit % Linear velocity limit
        % om_limit % Angular velocity limit
        % dt % Simulation time step
        
        myGPS % GPS class for measurement update
        myEKF % EKF class for kalman filter state estimation
    end
    methods
        function self = allied_unit(P,Pall)
        % An allied unit object that contains the position and animation
        % data. Includes functions to return the agent's position and a GPS
        % estimation of the agent's position.
        
            % Call the superclass constructor
            self@ground_unit(P,Pall);
            
            self.myGPS = GPS(self.dt);
            self.myEKF = EKF_GPS(self.X,eye(3),self.alph,...
                [0.4 0; 0 0.4],self.dt);
        end
        
        function self = updateEstimate(self)
        % Updates the agent's position using a Kalman Filter and a GPS
        % measurement
            self.myEKF.predict(self.u);
            GPSest = self.myGPS.getGPS(self.X);
            self.myEKF.correct(GPSest);
            self.Xe = self.myEKF.mu;
        end
        
        function self = animateOne(self)
        % Animates the agent on the current figure.
            
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Determine plotting points
            box_points = [-self.h/2,-self.h/2,self.h/2,self.h/2;
                          -self.w/2,self.w/2,self.w/2,-self.w/2];
            line1_points = [-self.h/2,self.h/2;-self.w/2,self.w/2];
            line2_points = [self.h/2,-self.h/2;-self.w/2,self.w/2];
            dot_points = [self.h/3;0];
            
            R = [cos(th), -sin(th); sin(th), cos(th)];
            rot_box = [x;y]+R*box_points;
            rot_line1 = [x;y]+R*line1_points;
            rot_line2 = [x;y]+R*line2_points;
            rot_dot = [x;y]+R*dot_points;
            
            % Plot the allied unit
            if isempty(self.plotHandles)
                self.plotHandles = gobjects(1,4);
                self.plotHandles(1) = fill(rot_box(1,:),rot_box(2,:),...
                    [0.2039,0.3647,0.6627]);
                hold on;
                self.plotHandles(2) = fill(rot_line1(1,:),rot_line1(2,:),'k');
                self.plotHandles(3) = fill(rot_line2(1,:),rot_line2(2,:),'k');
                self.plotHandles(4) = scatter(rot_dot(1),rot_dot(2),15,'ko','filled');
            else
                self.plotHandles(1).XData = rot_box(1,:);
                self.plotHandles(1).YData = rot_box(2,:);
                self.plotHandles(2).XData = rot_line1(1,:);
                self.plotHandles(2).YData = rot_line1(2,:);
                self.plotHandles(3).XData = rot_line2(1,:);
                self.plotHandles(3).YData = rot_line2(2,:);
                self.plotHandles(4).XData = rot_dot(1);
                self.plotHandles(4).YData = rot_dot(2);
            end
        end
    end
end
classdef allied_unit < ground_unit
    properties
        % State variables
        Xd % Desired enemy squad state
        
        % Noise variables
        alph % Motion noise
        Q % Measurement noise
        
        % Plotting variables
        box_points % Points to plot the UAV body
        sym_points % Points to plot the UAV symbol
        
        % Controller variables
        u % commanded force input
        integrator % Current value of the integrator
        error % Current error (distance and bearing)
        error_1 % Previous error (distance and bearing)
        kp % Position gain
        ki % Integrator gain
        limit % Force limit--assumed symmetric
        linear_ctrl % PID controller acting on the distance
        angular_ctrl % PID controller acting on the heading
        dt % Time step
        % For velocity-only commands, we set limits as well
        v_limit % Commanded linear velocity limit
        om_limit % Commanded angular velocity limit
        
        % enemy squad dynamics properties
        m % mass
        J % moment of inertia
        drag % drag coefficient
        % For velocity-only commands
        v % Linear velocity
        om % Angular velocity
    end
    methods
        function self = allied_unit(P,Ps)
        % An allied unit object that contains the position and animation
        % data. Includes functions to return the agent's position and a GPS
        % estimation of the agent's position.
            
            % Store the agent's initial position
            self.X = [P.x0;
                      P.y0;
                      P.th0];
                  
            % Animate the agent
            self.w = Ps.w;
            self.h = Ps.h;
            self.animate();
            
%             %%% Control Variables %%%
%             % Set initial desired position
%             self.Xd = [P.xd;
%                        P.yd];
%             % Initialize command limits
%             self.v_limit = P.v_limit;
%             self.om_limit = P.om_limit;
%             % Store the time step size
%             self.dt = dt;
%             % The following initialized unused force control parameters
% %             self.integrator = 0.0;
% %             self.linear_ctrl = PIDControl(P.kp,P.ki,P.kd,P.limit,P.beta,dt);
% %             self.angular_ctrl = PIDControl(P.kp,P.ki,P.kd,P.limit,P.beta,dt);
        end
        
        function self = moveAgent(self)
            % Moves the agent based on the control policy
            
            
        end
        
        function self = animate(self)
            % Animates the agent on the current figure.
            
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Determine plotting points
            box_points = [-self.w/2,self.w/2,self.w/2,-self.w/2;
                          -self.h/2,-self.h/2,self.h/2,self.h/2];
            line1_points = [-self.w/2,self.w/2;-self.h/2,self.h/2];
            line2_points = [-self.w/2,self.w/2;self.h/2,-self.h/2];
            dot_points = [0;self.h/3];
            
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
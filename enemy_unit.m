classdef enemy_unit < ground_unit
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
        function self = enemy_unit(P,Ps,dt)
        % An enemy unit object that contains the position and animation
        % data.
            
            % Store the agent's initial position
            self.X = [P.x0;
                      P.y0;
                      P.th0];     
             
            % Animate the agent
            self.w = Ps.h;
            self.h = Ps.h;
            self.animate();
            
            %%% Control Variables %%%
            % Set initial desired position
            self.Xd = [P.xd;
                       P.yd];
            % Initialize command limits
            self.v_limit = P.v_limit;
            self.om_limit = P.om_limit;
            % Store the time step size
            self.dt = dt;
            % The following initialized unused force control parameters
%             self.integrator = 0.0;
%             self.linear_ctrl = PIDControl(P.kp,P.ki,P.kd,P.limit,P.beta,dt);
%             self.angular_ctrl = PIDControl(P.kp,P.ki,P.kd,P.limit,P.beta,dt);
            
        end
        
        function self = moveAgent(self)
            % Moves the agent based on the control policy
            
            % temporarily (or maybe permanently) replaced by move_to_target           
        end
        
        function self = move_to_target(self)
            % Using the previously defined target position, moves the UAV
            % toward the goal position, updating both the state and the
            % state estimation.
            
            % Determine the required forces or velocities
%             self.calculateForce();
            self.calculateVelocity();
            
            % Update the dynamics based on the commanded input
            self.updateDynamics();
            
            % Update the estimated position
%             self.updateStateEstimate();
        end
        
        function self = calculateVelocity(self)
            % Calculates the required angular and linear velocity to reach
            % the goal location
            
            % Calculate the x, y, and theta error between the UAV's
            % estimated position and the desired location
            raw_err = self.Xd - self.X(1:2);
            raw_err(3) = atan2(self.Xd(2)-self.X(2),...
                self.Xd(1)-self.X(1))-self.X(3);
            
            % Convert error into range and bearing values
            self.error(1) = sqrt(raw_err(1)^2+raw_err(2)^2);
            % Ignore angular error when near the desired position
            if self.error(1) > 0.01
                self.error(2) = wrap_angle(raw_err(3));
            else
                self.error(2) = 0;
            end
            
            % Calculate the linear and angular velocity commands,
            % saturating the output when necessary
            if abs(self.error(1)/self.dt) < self.v_limit
                self.v = self.error(1) / self.dt;
            else
                self.v = self.v_limit;
            end
            if abs(self.error(2)/self.dt) < self.om_limit
                self.om = self.error(2) / self.dt;
            else
                self.om = sign(self.error(2))*self.om_limit;
            end
            
            % Add motion noise
            self.v = self.v;% + self.alph(1)*rand();
            self.om = self.om;% + self.alph(2)*rand();
        end
        
        function self = updateDynamics(self)
            % Updates the UAV state based on the current state and the
            % commanded input velocities
            
            self.X(1) = self.X(1) + self.v*cos(self.X(3))*self.dt;
            self.X(2) = self.X(2) + self.v*sin(self.X(3))*self.dt;
            self.X(3) = self.X(3) + self.om*self.dt;
        end
        
        function self = setTarget(self,xt,yt)
            % Public function that allows for modification of desired x and
            % y target positions. As part of the project, this should
            % eventually be overwritten with an algorithm that determines
            % the goal position based on the estimated enemy positions.
            self.Xd = [xt; yt];
        end
        
        function self = animate(self)
            % Animates the agent on the current figure.
            
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Determine plotting points
            box_points = [-self.w/2,0,self.w/2,0; 0,self.h/2,0,-self.h/2];
            line1_points = [-self.w/4,self.w/4; -self.h/4,self.h/4];
            line2_points = [-self.w/4,self.w/4; self.h/4,-self.h/4];
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
                    [0.8431,0.1961,0.1608]);
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
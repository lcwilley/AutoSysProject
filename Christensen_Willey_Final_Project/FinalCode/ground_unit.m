classdef ground_unit < handle
    properties
        % State estimates
        X % Unit state
        Xe % Estimated state
        Xd % Desired state
        
        % Plotting variables
        plotHandles % Unit plot handles
        w % Plotting width
        h % Plotting height
        
        % Control variables
        alph % Motion noise
        v % Linear velocity
        om % Angular velocity
        v_limit % Linear velocity limit
        om_limit % Angular velocity limit
        u % Commanded linear and angular velocity
        dt % Simulation time step
    end
    methods
        function self = ground_unit(P,Pall)
        % A ground unit superclass to help in the creation of enemy and
        % allied units
            
            % Store the agent's initial position
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            self.Xe = self.X;
            self.Xd = [];
            self.dt = Pall.dt;
            
            % Set the agent's dynamic properties
            self.alph = P.alph;
            self.v = 0;
            self.om = 0;
            self.v_limit = P.v_limit;
            self.om_limit = P.om_limit;
                  
            % Animate the agent
            self.w = Pall.w;
            self.h = Pall.h;
            self.animate();
        end
        
        function self = setTarget(self,target)
        % Public function that allows for modification of desired x and y
        % target positions. This assumes the target is passed as a (set of)
        % column vectors.
            
            for i = 1:length(self)
                self(i).Xd = target(:,i);
            end
        end
        
        function self = moveAgent(self)
        % Moves the ground unit toward the goal position
        
            for i = 1:length(self)
                if ~isempty(self(i).Xd)
                    % Determine the required velocities
                    self(i).getCommand();
                    % Update the dynamics based on the commanded input
                    self(i).updateDynamics();
                    % Update the agent's estimated position
                    self(i).updateEstimate();
                end
            end
        end
        
        function self = getCommand(self)
        % Calculate the x, y, and theta error between the unit's current
        % location and desired location
        
            raw_err = self.Xd - self.Xe(1:2);
            raw_err(3) = atan2(self.Xd(2)-self.Xe(2),...
                self.Xd(1)-self.Xe(1))-self.Xe(3);
            
            % Convert error into range and bearing values
            error(1) = sqrt(raw_err(1)^2+raw_err(2)^2);
            % Ignore angular error when near the desired position
            if error(1) > 0.01
                error(2) = wrap_angle(raw_err(3));
            else
                error(2) = 0;
            end
            
            % Calculate the linear and angular velocity commands,
            % saturating the output when necessary
            % Error is a distance. Error over time step is a speed in
            % either m/s or rad/s. 
            if abs(error(1)/self.dt) < self.v_limit
                self.v = error(1) / self.dt;
            else
                self.v = self.v_limit;
            end
            if abs(error(2)/self.dt) < self.om_limit
                self.om = error(2) / self.dt;
            else
                self.om = sign(error(2))*self.om_limit;
            end
            
            % Store commanded velocities
            self.u = [self.v; self.om];
            
            % Add motion noise
            self.v = self.v + self.alph(1)*rand();
            self.om = self.om + self.alph(2)*rand();
        end
        
        function self = updateDynamics(self)
        % Updates the unit state based on the current state and the
        % commanded input velocities
            
            self.X(1) = self.X(1) + self.v*cos(self.X(3))*self.dt;
            self.X(2) = self.X(2) + self.v*sin(self.X(3))*self.dt;
            self.X(3) = self.X(3) + self.om*self.dt;
        end
        
        function pos = getPos(self)
        % Called by either an individual agent or an array of similar
        % agents. Returns the agent's actual position.
            
            % Preallocate the return value and loop through self to account
            % for the case when this function is called by an agent array
            pos = zeros(2,length(self));
            for i = 1:length(self)
                pos(:,i) = self(i).X(1:2);
            end
        end
        
        function pos = getEstPos(self)
        % Called by either an individual agent or an array of similar
        % agents. Returns the agent's estimated position.
            
            % Preallocate the return value and loop through self to account
            % for the case when this function is called by an agent array
            pos = zeros(2,length(self));
            for i = 1:length(self)
                pos(:,i) = self(i).Xe(1:2);
            end
        end
        
        function self = animate(self)
            for i = 1:length(self)
                self(i).animateOne();
            end
        end
    end
    methods (Abstract)
        animateOne(self)
        updateEstimate(self)
    end
end
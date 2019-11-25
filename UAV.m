classdef UAV < handle
    properties
        % State variables
        X % UAV state
        Xe % Estimated UAV state
        Xd % Desired UAV state
        
        % State estimation variables
        EKF_pos % EKF class to calculate and return estimated position
        allies % Allied units to be able to return current GPS locations
        
        % Tracking variables
        enemies % Enemy units to allow for measurements
        enemy_X % Estimated position of enemies
        current_target % The enemy currently being pursued by the UAV
        
        % Noise variables
        alph % Motion noise
        Q % Measurement noise
        
        % Plotting variables
        plotHandles % UAV plot handles
        w % UAV width -- also used in the force controller
        h % UAV height
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
        
        % UAV dynamics properties
        m % mass
        J % moment of inertia
        drag % drag coefficient
        % For velocity-only commands
        v % Linear velocity
        om % Angular velocity
    end
    methods
        function self = UAV(P,Ps,allies,enemies,dt)
        % A UAV object containing the relevant dyanmics, state estimates, and
        % animations.
        % Uses an EKF to estimate position, and velocity commands to update
        % the dynamics, saturating the linear and angular velocities
        % according to defined limits.
        % Takes as arguments:
        % - A struct containing UAV properties
        % - An array of allied units that may be used to estimate position
        % - The length of the simulation time step
        
            %%% State Variables %%%
            % Initialize UAV state
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            % Initialize EKF to estimate UAV state
            self.EKF_pos = EKalFilt(self.X,eye(3),P.sig_r,P.sig_b,...
                P.alph,dt,length(enemies));
            self.Xe = self.EKF_pos.mu;
            self.enemy_X = self.EKF_pos.muE;
            self.current_target = randi(size(self.enemy_X,2));
            % Initialize noise variables
            self.alph = P.alph;
            self.Q = [P.sig_r; P.sig_b];
            % Store allies for use in position estimation and enemies for
            % tracking
            self.allies = allies;
            self.enemies = enemies;
            
            %%% Animation Variables %%%
            % Store shape property variables
            self.w = Ps.w;
            self.h = Ps.h;
            self.box_points = [-self.h/2,-self.h/2,self.h/2,self.h/2;
                               -self.w/2,self.w/2,self.w/2,-self.w/2];
            self.sym_points = [-self.h/4,0,self.h/3,0,-self.h/4,0;
                                -self.w/4,-self.w/4,0,self.w/4,self.w/4,0];
            % Draw the UAV in its initial state
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
            self.updateStateEstimate();
        end
        
        function self = track(self)
            % Creates measurements to the enemy units and estimates their
            % position
            
            % Get location of the enemy units
            enemy_pos = self.enemies.getPos();
            
            % Create measurement data for enemy units
            del = enemy_pos - self.X(1:2);
            obs = [sqrt(del(1,:).^2 + del(2,:).^2);
                wrap_angle(atan2(del(2,:),del(1,:))-self.X(3))];
            
            % Add noise to the measurement data
            obs = obs + self.Q.*rand(size(obs));
            
            % Update estimates based on measurement data
            self.enemy_X = self.EKF_pos.track(obs);
            self.setTarget();
        end
        
        function self = calculateVelocity(self)
            % Calculates the required angular and linear velocity to reach
            % the goal location
            
            % Calculate the x, y, and theta error between the UAV's
            % estimated position and the desired location
            raw_err = self.Xd - self.Xe(1:2);
            raw_err(3) = atan2(self.Xd(2)-self.Xe(2),...
                self.Xd(1)-self.Xe(1))-self.Xe(3);
            
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
            self.v = self.v + self.alph(1)*rand();
            self.om = self.om + self.alph(2)*rand();
        end
        
        function self = updateDynamics(self)
            % Updates the UAV state based on the current state and the
            % commanded input velocities
            
            self.X(1) = self.X(1) + self.v*cos(self.X(3))*self.dt;
            self.X(2) = self.X(2) + self.v*sin(self.X(3))*self.dt;
            self.X(3) = self.X(3) + self.om*self.dt;
        end
        
        function self = setTarget(self)
            % Public function that allows for modification of desired x and
            % y target positions. As part of the project, this should
            % eventually be overwritten with an algorithm that determines
            % the goal position based on the estimated enemy positions.
            if all(all(isnan(self.enemy_X)))
                self.Xd = self.X(1:2);
            else
                self.Xd = self.enemy_X(:,self.current_target);
            end
        end
        
        function self = calculateForce(self)
            % Uses the error between the UAV state and the desired state to
            % calculate the necessary forces to allow the UAV to reach the
            % state
            
            % Determine the error between the desired position and
            % estimated position
            raw_err = self.Xd - self.Xe(1:2);
            self.error(1) = sqrt(raw_err(1)^2+raw_err(2)^2);
            self.error(2) = wrap_angle(raw_err(3));
            
            % Get the desired angle from the angular controller
            force = self.linear_ctrl.PID(self.error(1));
            tau = self.angular_ctrl.PID(self.error(2));
            
            % Determine the controls to the motors using a mixing matrix
            self.u = [self.w/2, -self.w/2; 1, 1]\[tau; force];
        end
        
        function self = propagateDynamics(self)
            % Similar to updateDynamics, this updates the UAV state based
            % on commanded inputs; however, it assumes the inputs to be
            % force-based rather than velocity-based, and uses equations of
            % motion to more realistically model UAV dynamics.
            
            % Unpack state and control inputs
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            fl = self.u(1);
            fr = self.u(2);
            
            % All I need to do is calculate the derivative so I can
            % multiply it by the time step and add it to my current state.
            % I need to be sure to add noise in as well.
            % Necessary properties include mass and moment of inertia.
            % I just want to use simple dynamics here--nothing too fancy.
            vdot = 1/self.m*((fl+fr)-self.drag*v); % v needs to be defined somewhere.
            zddot = 1/self.mt*(-(fl+fr)*sin(theta)-self.mu*zdot);
            hddot = 1/self.mt*((fl+fr)*cos(theta)-self.mt*self.g);
            thetaddot = 1/(self.Jc+2*self.mr*self.d^2)*(self.d*(fr-fl));
            
            % build xdot and return
            xdot = [thetadot; zdot; hdot; thetaddot; zddot; hddot];
        end
        
        function self = updateStateEstimate(self)
            % Using an EKF and the positions of the allied units, updates
            % the UAV's belief of its current state.
            
            % Get location of the allied units
            allied_est = [zeros(2,1), self.allies.getGPS()];
            allied_pos = [zeros(2,1), self.allies.getPos()];
            
            % Create measurement data for allied units
            del = allied_pos - self.X(1:2);
            obs = [sqrt(del(1,:).^2 + del(2,:).^2);
                wrap_angle(atan2(del(2,:),del(1,:))-self.X(3))];
            
            % Add noise to the measurement data
            obs = obs + self.Q.*rand(size(obs));
            
            % Update the EKF based on the estimated allied positions and
            % the measurement data
            self.Xe = self.EKF_pos.update([self.v; self.om],allied_est,obs);
        end
        
        function self = animate(self)
            % Animates the UAV true state and estimated state on the
            % current figure.
            
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Unpack estimated state
            xe = self.Xe(1);
            ye = self.Xe(2);
            the = self.Xe(3);
            
            % Determine plotting points
            R = [cos(th), -sin(th); sin(th), cos(th)];
            rot_box = [x;y]+R*self.box_points;
            rot_sym = [x;y]+R*self.sym_points;
            Re = [cos(the), -sin(the); sin(the), cos(the)];
            rot_box_e = [xe;ye]+Re*self.box_points;
            rot_sym_e = [xe;ye]+Re*self.sym_points;
            
            % Plot the UAV
            if isempty(self.plotHandles)
                % On the first call, plot the ground controller
                fill([-self.w/2,self.w/2,self.w/2,-self.w/2],...
                    [-self.h/2,-self.h/2,self.h/2,self.h/2],...
                      [0.2039,0.3647,0.6627]);
                hold on;
                fill([-self.w/4,-self.w/4,0,self.w/4,self.w/4,0],...
                    [self.h/4,0,-self.h/3,0,self.h/4,0],'k');
                plot([-self.w/2,self.w/2],[-self.h/2,self.h/2],'k');
                plot([-self.w/2,self.w/2],[self.h/2,-self.h/2],'k');
                scatter(0,self.h/3,15,'ko','filled')

                % Setup the UAV plot
                self.plotHandles = gobjects(1,4);
                % Plot the true state
                self.plotHandles(1) = fill(rot_box(1,:),rot_box(2,:),...
                    [0.2039,0.3647,0.6627]);
                self.plotHandles(2) = fill(rot_sym(1,:),rot_sym(2,:),'k');
                % Plot the estimated state
                self.plotHandles(3) = fill(rot_box_e(1,:),...
                    rot_box_e(2,:),[0.2039,0.3647,0.6627]);
                self.plotHandles(4) = fill(rot_sym_e(1,:),...
                    rot_sym_e(2,:),'k');
                alpha(self.plotHandles(3:4),0.5);
            else
                % Update the UAV plot
                self.plotHandles(1).XData = rot_box(1,:);
                self.plotHandles(1).YData = rot_box(2,:);
                self.plotHandles(2).XData = rot_sym(1,:);
                self.plotHandles(2).YData = rot_sym(2,:);
                self.plotHandles(3).XData = rot_box_e(1,:);
                self.plotHandles(3).YData = rot_box_e(2,:);
                self.plotHandles(4).XData = rot_sym_e(1,:);
                self.plotHandles(4).YData = rot_sym_e(2,:);
            end
        end
    end
end
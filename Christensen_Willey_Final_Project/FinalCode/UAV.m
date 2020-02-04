classdef UAV < handle
    properties
        % State variables
        X % UAV state
        Xe % Estimated UAV state
        Xd % Desired UAV state
        
        % State estimation variables
        myEKF % EKF class to calculate and return estimated position
        allies % Allied units to be able to return current GPS locations
        
        % Tracking variables
        bet % Field of vision of the UAV
        myMCL % MCL class to estimate enemy motion based on measurements
        enemies % Enemy units to allow for measurements
        enemy_X % Estimated position of enemies
        enemy_sig % Enemy uncertainty
        currentSearchPos % Index of current search destination
        
        % Noise variables
        alph % Motion noise
        Q % Measurement noise
        Qmax
        
        % Plotting variables
        plotHandles % UAV plot handles
        w % UAV width -- also used in the force controller
        h % UAV height
        box_points % Points to plot the UAV body
        sym_points % Points to plot the UAV symbol
        
        % Controller variables
        dt % Time step
        error % Current error (distance and bearing)
        v_limit % Commanded linear velocity limit
        om_limit % Commanded angular velocity limit
        v % Experienced linear velocity
        om % Experienced angular velocity
        u % Commanded linear and angular velocity
    end
    methods
        function self = UAV(P,Pall,allies,enemies)
        % A UAV object containing the relevant dyanmics, state estimates, and
        % animations.
        % Uses an EKF to estimate position, and velocity commands to update
        % the dynamics, saturating the linear and angular velocities
        % according to defined limits.
        % Takes as arguments:
        % - A struct containing UAV properties
        % - A struct containing simulation properties
        % - An array of allied units to estimate position
        % - An array of enemy units to be tracked
        
            % Store the time step size
            self.dt = Pall.dt;
            %%% State Variables %%%
            % Initialize UAV state
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            % Initialize EKF to estimate UAV state
            self.myEKF = EKFtrilat(self.X,eye(3),P.sig_r,P.sig_b,...
                P.alph,Pall.dt,length(enemies));
            self.Xe = self.myEKF.mu;
            % Initialize noise variables
            self.alph = P.alph;
            self.Q = [P.sig_r; P.sig_b];
            self.Qmax = [3; 0.25];
            % Store allies for use in position estimation and enemies for
            % tracking
            self.allies = allies;
            self.enemies = enemies;
            % Setup MCLs for tracking enemies
            self.bet = P.bet;
            self.myMCL = MCL.empty(0,length(self.enemies));
            for i = 1:length(self.enemies)
                self.myMCL(i) = MCL(500,0.1,self.Q,self.dt);
            end
            self.enemy_X = zeros(2,length(self.enemies));
            self.enemy_sig = [NaN NaN];
            
            %%% Animation Variables %%%
            % Store shape property variables
            self.w = Pall.w;
            self.h = Pall.h;
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
            self.currentSearchPos = 1;
        end
        
        function self = move_to_target(self)
        % Using the previously defined target position, moves the UAV
        % toward the goal position, updating both the state and the state
        % estimation
            
            % Determine the required velocities
            self.calculateVelocity();
            
            % Update the dynamics based on the commanded input
            self.updateDynamics();
            
            % Update the estimated position
            self.updateStateEstimate();
        end
        
        function self = track(self)
        % Creates measurements to the enemy units and estimates their
        % positions
            
            % Get location of the enemy units for measurements
            enemy_pos = self.enemies.getPos();
            
            % Create measurement data for enemy units
            del = enemy_pos - self.X(1:2);
            z = [sqrt(del(1,:).^2 + del(2,:).^2);
                wrap_angle(atan2(del(2,:),del(1,:))-self.X(3))];
            
            % Update the MCL for each enemy
            for i = 1:length(self.myMCL)
                p_detect = 25/sqrt(2*pi*20)*...
                    exp(-(z(1,i)/1000*160)^2/(2*20^2));
                % Check possibility of detection (field of view and distance)
                if abs(z(2,i)) < self.bet/2 && rand() < p_detect 
                    % Add noise to measurement data based on distance
                    % If p_detect is 0, I want noise to be Qmax. If
                    % p_detect is 1, I want noise to be Q.
                    noise = self.Q + (self.Qmax - self.Q)*...
                        (1-min([1,p_detect]));
                    z = z + noise.*rand(size(z));
                    self.myMCL(i).update(self.Xe,z(:,i));
                    [self.enemy_X(:,i),self.enemy_sig(i)] =...
                        self.myMCL(i).getEstimates();
                else
                    self.myMCL(i).update_no_meas();
                    [self.enemy_X(:,i),self.enemy_sig(i)] =...
                        self.myMCL(i).getEstimates();
                end
            end
            
            % Set the target based on the enemy with greater uncertainty
            self.setTarget();
            self.setAlliedTarget();
        end
        
        function self = setTarget(self)
        % Sets the UAV target to be the estimated position of the enemy
        % that has a greater error
        
            if any(isnan(self.enemy_sig))
                self.Xd = self.nextSearchPos();
            else
                % Fly toward the target with less position information
                [~,current_target] = max(self.enemy_sig);
                self.Xd = self.enemy_X(:,current_target);
            end
        end
        
        function self = setAlliedTarget(self)
        % Assign the nearest allied units to go to the targets'
        % estimated positions
            
            ally_pos = self.allies.getEstPos();
            for i = 1:length(self.enemies)
                if ~isnan(self.enemy_sig(i))
                [~,closer_ally] = min(sqrt(...
                    (ally_pos(1,:)-self.enemy_X(1,i)).^2+...
                    (ally_pos(1,:)-self.enemy_X(1,i)).^2));
                self.allies(closer_ally).setTarget(...
                    self.enemy_X(:,i));
                end
            end
        end
        
        function pos = nextSearchPos(self)
        % Returns the x and y coordinates of the next search location
        
            % Initialize the list of potential search positions
            search_pos = [300 300 -300 -300; 300 700 700 300];
            % Once the UAV reaches the search position, move on to the next
            if abs(sum(self.X(1:2)-self.Xd)) < 0.5
                ind = find(all(self.Xd==search_pos,1));
                self.currentSearchPos = mod((ind-1)+1,...
                    size(search_pos,2))+1;
            end
            pos = search_pos(:,self.currentSearchPos);
        end
        
        function self = calculateVelocity(self)
        % Calculates the required angular and linear velocity to reach the
        % goal location
        
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
            
            % Store commanded velocity
            self.u = [self.v; self.om];
            
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
        
        function self = updateStateEstimate(self)
            % Using an EKF and the positions of the allied units, updates
            % the UAV's belief of its current state.
            
            % Get location of the allied units
            allied_est = [zeros(2,1), self.allies.getEstPos()];
            allied_pos = [zeros(2,1), self.allies.getPos()];
            
            % Create measurement data for allied units
            del = allied_pos - self.X(1:2);
            obs = [sqrt(del(1,:).^2 + del(2,:).^2);
                wrap_angle(atan2(del(2,:),del(1,:))-self.X(3))];
            
            % Add noise to the measurement data
            obs = obs + self.Q.*rand(size(obs));
            obs = obs(1,:)';
            
            % Update the EKF based on the estimated allied positions and
            % the measurement data
            self.Xe = self.myEKF.update(self.u,allied_est,obs);
        end
        
        function self = capturedEnemy(self,i)
        % Removes an enemy from the UAV tracking list
        
            self.enemies(i) = [];
            self.enemy_X(:,i) = [];
            self.enemy_sig(i) = [];
            self.myMCL(i).unanimate();
            self.myMCL(i) = [];
        end
        
        function self = animate(self)
        % Animates the UAV true state and estimated state on the current
        % figure
            
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
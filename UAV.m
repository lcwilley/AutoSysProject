classdef UAV < handle
    properties
        % State variables
        X % UAV state
        Xe % Estimated UAV state
        Xd % Desired UAV position
        
        % State estimation variables
        EKF_pos % EKF class to calculate and return estimated position
        allies % Allied units to be able to return current GPS locations
        
        % Plotting variables
        plotHandles % UAV plot handles
        w % UAV width -- also used in the controller
        h % UAV height
        
        % Controller variables
        u % commanded force input
        integrator % Current value of the integrator
        error % Current error (distance and bearing)
        error_1 % Previous error (distance and bearing)
        kp % Position gain
        ki % Integrator gain
        limit % Force limit--assumed symmetric
        v_limit
        th_limit
        linear_ctrl % PID controller acting on the distance
        angular_ctrl % PID controller acting on the heading
        dt % Time step
        
        % UAV dynamics properties
        m % mass
        J % moment of inertia
        v
        om
        drag % drag coefficient
    end
    methods
        function self = UAV(P,allies,dt)
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            self.EKF_pos = EKalFilt(self.X,eye(3),P.sig_r,P.sig_b,...
                P.alph,dt);
            self.allies = allies;
            self.w = P.w;
            self.h = P.h;
            self.Xe = self.EKF_pos.mu;
            self.animate();
            self.Xd = [P.xd;
                       P.yd];
            self.th_limit = pi/2; % rad/s
            self.v_limit = 2; %m/s
            self.dt = dt;
            self.integrator = 0.0;
%             self.linear_ctrl = PIDControl(P.kp,P.ki,P.kd,P.limit,P.beta,dt);
%             self.angular_ctrl = PIDControl(P.kp,P.ki,P.kd,P.limit,P.beta,dt);
        end
        
        function self = move_to_target(self)
            % Using the defined target position, moves the UAV toward the
            % goal position
            
            % Determine the required forces
%             self.calculateForce();
            self.skipToVelocity();
            self.updateDynamics();
            
            % Update the estimated position
            self.updateStateEstimate();
        end
        
        function self = skipToVelocity(self)
            % Calculates the required angular and linear velocity to reach
            % the goal location
            raw_err = self.Xd - self.Xe(1:2);
            raw_err(3) = atan2(self.Xd(2)-self.Xe(2),...
                self.Xd(1)-self.Xe(1))-self.Xe(3);
            self.error(1) = sqrt(raw_err(1)^2+raw_err(2)^2);
            if self.error(1) > 0.01
                self.error(2) = wrap_angle(raw_err(3));
            else
                self.error(2) = 0;
            end
            
            if self.error(1) < self.v_limit * self.dt
                self.v = self.error(1);
            else
                self.v = self.v_limit;
            end
            if self.error(2) < self.th_limit * self.dt
                self.om = self.error(2);
            else
                self.om = self.th_limit;
            end
        end
        
        function self = updateDynamics(self)
            self.X(1) = self.X(1) + self.v*cos(self.X(3))*self.dt;
            self.X(2) = self.X(2) + self.v*sin(self.X(3))*self.dt;
            self.X(3) = self.X(3) + self.om*self.dt;
        end
        
        function self = setTarget(self,xt,yt)
            self.Xd = [xt; yt];
        end
        
        function self = calculateForce(self)
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
            % Get location of the allied units
            allied_est = [zeros(2,1), self.allies.getGPS()];
            allied_pos = [zeros(2,1), self.allies.getPos()];
            
            % Create measurement data for allied units
            del = allied_pos - self.X(1:2);
            obs = [sqrt(del(1,:).^2 + del(2,:).^2);
                wrap_angle(atan2(del(2,:),del(1,:))-self.X(3))];
            
            % Update the EKF based on the estimated allied positions and
            % the measurement data
            self.Xe = self.EKF_pos.update([self.v; self.om],allied_est,obs);
        end
        
        function self = animate(self)
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Determine plotting points
            box_points = [-self.h/2,-self.h/2,self.h/2,self.h/2;
                          -self.w/2,self.w/2,self.w/2,-self.w/2];
            line_points = [-self.h/4,0,self.h/3,0,-self.h/4,0;
                           -self.w/4,-self.w/4,0,self.w/4,self.w/4,0];
            R = [cos(th), -sin(th); sin(th), cos(th)];
            rot_box = [x;y]+R*box_points;
            rot_line = [x;y]+R*line_points;
            
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
                self.plotHandles = gobjects(1,2);
                self.plotHandles(1) = fill(rot_box(1,:),rot_box(2,:),...
                    [0.2039,0.3647,0.6627]);
                self.plotHandles(2) = fill(rot_line(1,:),rot_line(2,:),'k');
            else
                % Update the UAV plot
                self.plotHandles(1).XData = rot_box(1,:);
                self.plotHandles(1).YData = rot_box(2,:);
                self.plotHandles(2).XData = rot_line(1,:);
                self.plotHandles(2).YData = rot_line(2,:);
            end
        end
    end
end
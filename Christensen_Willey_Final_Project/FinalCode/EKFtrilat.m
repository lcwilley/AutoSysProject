classdef EKFtrilat < EKF_unicycle
    properties
        % Estimate and Covariance
        muE
        sigE
        
        % Noise parameters
        Qe % Enemy measurement noise matrix
        Ge % Enemy state update matrix
        
        % Tracking information
        Ne % Number of enemy units
        obs % Boolean array indicating if an enemy as been seen before
        
        % Animation variables
        plotHandles % Enemy position plot handles
    end
    methods
        function self = EKFtrilat(mu0, sig0, sig_r, sig_th, alph, dt, Ne)
        % Unicycle EKF model with a trilateration update
            
            self@EKF_unicycle(mu0,sig0,alph,sig_r^2*eye(3),dt);
            self.Ne = Ne;
            self.muE = NaN*ones(4,self.Ne);
            self.sigE = zeros(4,4,self.Ne);
            self.Ge = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
            self.Qe = diag([sig_r sig_th]);
            self.obs = zeros(1,self.Ne);
            self.plotHandles = gobjects(1,self.Ne);
            for i = 1:length(self.plotHandles)
                self.plotHandles(i) = fill(0,0,'r');
                set(self.plotHandles(i),'XData',[],'YData',[]);
            end
        end
        
        function estimates = update(self, u, Lm, z)
            self.predict(u);
            self.correct(Lm, z);
            estimates = self.mu;
        end

        function self = correct(self, Lm, z)
            % Unpack state estimate
            rx = self.mu(1);
            ry = self.mu(2);
            rth = self.mu(3);
            
            % Calculate predicted obsevation
            q = ((Lm(1,:)-rx).^2 + (Lm(2,:)-ry).^2)';
            q(q==0) = 1e-100;
            zhat = sqrt(q);
            
            % Calculate derivative of measurment with respect to state
            H = [-(Lm(1,:)-rx)'./sqrt(q),...
                 -(Lm(2,:)-ry)'./sqrt(q),...
                 zeros(3,1)];
             
            % Calculate Kalman gain
            S = H*self.sig*H' + self.Q;
            K = self.sig*H'/S;
            
            % Update estimate
            self.mu = self.mu + K*(z-zhat);
            self.sig = (eye(length(self.sig))-K*H)*self.sig;
        end
        
        function enemy_X = track(self, z)
            self.predict_enemy();
            self.track_enemy(z);
            enemy_X = self.muE(1:2,:);
        end
        
        function self = predict_enemy(self)
            % Updates the enemy estimate based on the estimated state
            for i = 1:size(self.muE,2)
                if self.obs(i)
                    self.muE(:,i) = self.Ge*self.muE(:,i);
                end
            end
        end
        
        function self = track_enemy(self, z)
            % Updates the UAV tracking estimates based on the measurements
            % taken and assuming a known data association
            % I think it would be cool later to have it just pass in
            % the measurements and see if it can determine which enemy is
            % likely the one that caused the observation
            
            % Loop through each observation
            for i = 1:size(z,2)
                % Check to see if an observation was taken for this enemy
                if ~any(isnan(z(:,i)))
                    % Unpack state estimate
                    rx = self.mu(1);
                    ry = self.mu(2);
                    rth = self.mu(3);
                    
                    % Check if the enemy has been observed before
                    if ~self.obs(i)
                        % Initialize the enemy position, assuming v=0
                        self.muE(:,i) = [rx + z(1,i)*cos(z(2,i)+rth);
                        	             ry + z(1,i)*sin(z(2,i)+rth);
                                         0;
                                         0];

                        % Unpack enemy position
                        ex = self.muE(1,i);
                        ey = self.muE(2,i);
                        vx = self.muE(3,i);
                        vy = self.muE(4,i);
                        
                        % Initialize estimate covariance
                        % H is partial w.r.t. enemy state
                        del = [ex+vx*self.dt - rx; ey+vy*self.dt - ry];
                        q = del'*del;
                        H = [del(1)/sqrt(q) del(2)/sqrt(q)...
                             del(1)/sqrt(q)*self.dt del(2)/sqrt(q)*self.dt;
                             -del(1)/q      del(2)/q...
                             -del(1)/q*self.dt      del(2)/q*self.dt];
                        self.sigE(:,:,i) = H'/(H*H' + self.Qe)*H;
                        self.obs(i) = 1;
                    else
                        % Unpack enemy position
                        ex = self.muE(1,i);
                        ey = self.muE(2,i);
                        vx = self.muE(3,i);
                        vy = self.muE(4,i);
                        
                        % Estimate predicted measurement
                        del = [ex+vx*self.dt - rx; ey+vy*self.dt - ry];
                        q = del'*del;
                        zhat = [sqrt(q);
                                wrap_angle(atan2(del(2),del(1)))];
                            
                        % Jacobian of measurement w.r.t. enemy state
                        H = [del(1)/sqrt(q) del(2)/sqrt(q)...
                             del(1)/sqrt(q)*self.dt del(2)/sqrt(q)*self.dt;
                             -del(1)/q      del(2)/q...
                             -del(1)/q*self.dt      del(2)/q*self.dt];

                        % Calculate measurement covariance
                        S = H*self.sigE(:,:,i)*H' + self.Qe;
                        K = self.sigE(:,:,i)*H'/S;
                        
                        % Determine error
                        error = z(:,i) - zhat;
                        error(2,:) = wrap_angle(error(2,:));

                        % Update estimate
                        self.muE(:,i) = self.muE(:,i) + K*error;
                        self.sigE(:,:,i) = (eye(length(self.sigE))-K*H)*...
                            self.sigE(:,:,i);
                    end
                end
            end
            self.animate();
        end
        
        function self = animate(self)
            % Updates estimate ellipses of estimated enemy positions. Plot
            % initialization occurs in the setup function.
            
            % Loop through all enemies
            for i = 1:self.Ne
                % Only plot enemies that have been observed
                if ~any(isnan(self.muE(:,i)))
                    % Calculate ellipse values with single value
                    % decomposition
                    plt_vals = error_ellipse(self.muE(1:2,i),...
                        self.sigE(1:2,1:2,i));
                    % Update plot values
                    self.plotHandles(i).XData = plt_vals(1,:);
                    self.plotHandles(i).YData = plt_vals(2,:);
                end
            end
        end
    end
end

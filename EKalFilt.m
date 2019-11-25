classdef EKalFilt < handle
    properties
        % Estimate and Covariance
        mu
        sig
        muE
        sigE
        
        % Time parameter
        dt % Time step
        
        % Noise parameters
        alph % Velocity noise
        Q % Sensor noise
        
        % Tracking information
        Ne % Number of enemy units
        obs % Boolean array indicating if an enemy as been seen before
        
        % Animation variables
        plotHandles % Enemy position plot handles
    end
    methods
        % NN is total number of time pts
        function self = EKalFilt(mu0, sig0, sig_r, sig_th, alph, dt, Ne)
            self.Ne = Ne;
            self.mu = mu0;
            self.sig = sig0;
            self.muE = NaN*ones(2,self.Ne);
            self.sigE = zeros(2,2,self.Ne);
            self.Q = [sig_r^2, 0; 0, sig_th^2];
            self.alph = alph;
            self.dt = dt;
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

        function self = predict(self, u)
            % Unicycle model: [ x+vt*cos(th)*dt;
            %                   y+vt*sin(th)*dt;
            %                   th+wt*dt]
            % Unpack parameters
            th = self.mu(3);
            vt = u(1);
            wt = u(2);

            % Jacobian wrt state
            G = eye(3);
            G(1,3) = -vt*sin(th)*self.dt;
            G(2,3) = vt*cos(th)*self.dt;
            % Jacobian wrt control
            V = [cos(th)*self.dt, 0;
                  sin(th)*self.dt, 0;
                  0, self.dt];
            % Motion noise
            M  = [self.alph(1)*vt^2, 0;
                    0, self.alph(2)*wt^2];

            % Update estimates
            mu_update = [vt*cos(th)*self.dt;
                        vt*sin(th)*self.dt;
                        wrap_angle(wt*self.dt)];


            self.mu = self.mu + mu_update;
            self.mu(3) = wrap_angle(self.mu(3));
            self.sig = G*self.sig*G' + V*M*V';
        end

        function self = correct(self, Lm, z)
            for i = 1:size(Lm,2)
                % Unpack landmark and estimate positions
                mx = Lm(1,i);
                my = Lm(2,i);
                rx = self.mu(1);
                ry = self.mu(2);
                rth = self.mu(3);

                % Calculate predicted obsevation and Kalman gain
                q = (mx - rx)^2 + (my - ry)^2;
                zhat = [sqrt(q);
                        wrap_angle(atan2((my-ry),(mx-rx))-rth)];
                H = [-(mx-rx)/sqrt(q), -(my-ry)/sqrt(q), 0;
                    (my-ry)/q, -(mx-rx)/q, -1];
                S = H*self.sig*H' + self.Q;
                K = self.sig*H'/S;

                % Update estimate
                self.mu = self.mu + K*(z(:,i)-zhat);
                self.mu(3) = wrap_angle(self.mu(3));
                self.sig = (eye(length(self.sig))-K*H)*self.sig;
            end
        end
        
        function enemy_X = track(self, z)
            self.track_enemy(z);
            enemy_X = self.muE;
        end
        
        function self = track_enemy(self, z)
            % Updates the UAV tracking estimates based on the measurements
            % taken and assuming a known data association
            % I think it would be cool later one to have it just pass in
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
                        % Initialize the enemy position
                        self.muE(:,i) = [rx + z(1,i)*cos(z(2,i)+rth);
                        	             ry + z(1,i)*sin(z(2,i)+rth)];

                        % Unpack enemy position
                        ex = self.muE(1,i);
                        ey = self.muE(2,i);
                        
                        % Initialize estimate covariance
                        % H is partial w.r.t. enemy position
                        q = (ex - rx)^2 + (ey - ry)^2;
                        H = [(ex-rx)/sqrt(q), (ey-ry)/sqrt(q);
                             -(ey-ry)/q, (ex-rx)/q];
                        self.sigE(:,:,i) = H\self.Q/(H');
                        self.obs(i) = 1;
                    else
                        % Unpack enemy position
                        ex = self.muE(1,i);
                        ey = self.muE(2,i);

                        % Calculate the estimated measurement
                        q = (ex - rx)^2 + (ey - ry)^2;
                        zhat = [sqrt(q);
                            wrap_angle(atan2(ey-ry,ex-rx)-rth)];

                        % Calculate measurement Jacobian
                        H = [(ex-rx)/sqrt(q), (ey-ry)/sqrt(q);
                             -(ey-ry)/q, (ex-rx)/q];

                        % Calculate measurement covariance
                        S = H*self.sigE(:,:,i)*H' + self.Q;
                        K = self.sigE(:,:,i)*H'/S;

                        % Update estimate
                        error = z(:,i)-zhat;
                        error(2) = wrap_angle(error(2));
                        self.muE(:,i) = self.muE(:,i) + K*error;
                        self.sigE(:,:,i) = (eye(2)-K*H)*self.sigE(:,:,i);
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
                    plt_vals = error_ellipse(self.muE(:,i),self.sigE(:,:,i));
                    % Update plot values
                    self.plotHandles(i).XData = plt_vals(1,:);
                    self.plotHandles(i).YData = plt_vals(2,:);
                end
            end
        end
    end
end

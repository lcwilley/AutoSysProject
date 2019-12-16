classdef MCL < handle
    properties
        NP % Number of particles
        obs % Boolean indicator of state initialization
        
        % Noise parameters
        Q % Measurement noise
        alph % Motion noise
        
        % Motion and measurement models
        g % Motion model
        h % Measurement model
        
        % Estimates
        mu % Current estimates
        sig % Variance of current estimates
        avg % Average estimate
        x % Un-sampled estimates
        w % Weights of unsampled estimates
        avg_wghts % Weigths of resampled estimates
        
        plotHandle
    end
    methods
        function self = MCL(NP,alph0,Q,dt)
            self.NP = NP;
            self.obs = 0;
            self.x = zeros(3,self.NP);
            self.g = @(X,n) [X(1) + 3.3*cos(X(3)+n)*dt;
                             X(2) + 3.3*sin(X(3)+n)*dt
                             atan2(-X(2),-X(1)) + n];
%             self.g = @(X,n) [1 0 dt 0;
%                              0 1 0 dt;
%                              0 0 1 0;
%                              0 0 0 1]*X + [0;0;n];
            self.h = @(X,x) [sqrt((X(1)-x(1))^2+(X(2)-x(2))^2);
                            wrap_angle(atan2(x(2)-X(2),x(1)-X(1))-X(3))];
            % Unpack noise and landmarks
            self.alph = alph0;
            self.Q = Q;
            % Setup average estimate
            self.avg = mean(self.mu,2);
        end
        
        function self = update(self,X,z)
            if self.obs
                self.predict();
                self.correct(X,z);
                self.low_variance_resample();
                self.avg = sum(self.mu.*self.avg_wghts,2);
                self.sig = sum(std(self.mu(1:2,:),0,2));
            else
                % If the enemy has not yet been observed, initialize the
                % position from the observation
                self.x = [X(1) + (z(1)+randn(1,self.NP)*self.Q(1)).*...
                              cos((z(2)+randn(1,self.NP)*self.Q(2))+X(3));
                          X(2) + (z(1)+randn(1,self.NP)*self.Q(1)).*...
                              sin((z(2)+randn(1,self.NP)*self.Q(2))+X(3))];
                angle_to_base = atan2(-self.x(2),-self.x(1));
                self.x = [self.x;
                          angle_to_base+randn(1,self.NP)*self.alph(1)];
                self.mu = self.x;
                self.avg = mean(self.x,2);
                self.sig = sum(std(self.x(1:2,:),0,2));
                self.obs = 1;
            end
            self.animate();
        end
        
        function self = update_no_meas(self)
            if self.obs
                for i = 1:self.NP
                    self.mu(:,i) = self.g(self.mu(:,i),0);
                end
                self.avg = sum(self.mu.*self.avg_wghts,2);
                self.sig = sum(std(self.mu(1:2,:),0,2));
                self.animate();
            end
        end
        
        function self = predict(self)
            for i = 1:self.NP
                % Update based on the motion model
                % It would be nice to have the alpha values change based on
                % the probability of the measurement (i.e., worse
                % measurements result in higher alpha values)
                noise = self.alph(1)*randn();
                self.x(:,i) = self.g(self.mu(:,i),noise);
            end
        end
        
        function self = correct(self,X,z)
        % Correction portion of the MCL based on the UAV position (which is
        % the "known" position measuring to the unknown potential landmark
        % positions) and the current observation
        
            % Initialize the weights back to one
            self.w = ones(1,self.NP);
            for j = 1:self.NP
                % This is estimating the measurement for each particle.
                % This needs to be done backwards.
                meas_error = self.h(X,self.x(:,j)) - z;
                meas_error(2) = wrap_angle(meas_error(2));
                self.w(j) = self.w(j) *...
                    self.meas_prob(meas_error(1),self.Q(1)) *...
                    self.meas_prob(meas_error(2),self.Q(2));
            end
            % Normalize the weights
            if sum(self.w) < 0.01
                self.alph = 1.1*self.alph;
            elseif sum(self.w) > 1
                self.alph = 0.9*self.alph;
            end
            self.w = self.w / sum(self.w);
        end
        
        function self = low_variance_resample(self)
            % This is roulette wheel selection with a uniform offset that
            % helps reduce variance
            r = rand/self.NP;
            c = self.w(1);
            self.avg_wghts = zeros(1,self.NP);
            i = 1;
            for m = 1:self.NP
                U = r + (m-1)/self.NP;
                while U > c
                    i = i + 1;
                    c = c + self.w(i);
                end
                self.mu(:,m) = self.x(:,i);
                self.avg_wghts(m) = self.w(i);
            end
            self.avg_wghts = self.avg_wghts/sum(self.avg_wghts);
            
            % Combat particle deprivation
            P = var(self.x,0,2); % covariance of prior
            uniq = size(unique(self.mu),2); % number of unique particles
            if uniq/self.NP < 0.5 % if there is a lot of duplication
                Qx = P/((self.NP*uniq)^(1/size(self.mu,1))); % add noise
                self.mu = self.mu + sqrt(Qx).*randn(size(self.mu));
            end
        end
        
        function prob = meas_prob(~,a,b)
        % Given a value, a, and the variance, b, returns the probability
        % of the value occurring
            prob = 1/sqrt(2*pi*b)*exp(-1/2*a^2/b);
        end
        
        function [meanPos, spread] = getEstimates(self)
            if self.obs
                meanPos = self.avg(1:2);
                spread = self.sig;
            else
                meanPos = NaN;
                spread = NaN;
            end
        end
        
        function self = animate(self)
            if isempty(self.plotHandle)
                self.plotHandle = scatter(self.mu(1,:),self.mu(2,:),'k.');
            else
                self.plotHandle.XData = self.mu(1,:);
                self.plotHandle.YData = self.mu(2,:);
            end
        end
    end
end
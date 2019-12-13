classdef EKF_unicycle < handle
    properties
        % Estimate and Covariance
        mu % State estimate
        sig % State covariance
        
        % Time parameter
        dt % Time step
        
        % Noise parameters
        alph % Velocity noise
        Q % Sensor noise
    end
    methods
        function self = EKF_unicycle(mu0, sig0, alph, Q, dt)
        % Basic EKF constructor for a unicycle model. Implements only the
        % prediction step.
        % Inputs:
        % - mu0:  initial state estimate
        % - sig0: initial covariance
        % - alph: motion noise
        % - dt:   time step

            self.mu = mu0;
            self.sig = sig0;
            self.alph = alph;
            self.Q = Q;
            self.dt = dt;
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
    end
end

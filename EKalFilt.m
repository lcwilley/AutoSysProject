classdef EKalFilt < handle
    properties
        % Estimate and Covariance
        mu
        sig
        % Time parameter
        dt
        % Noise variables
        Q
        alph
    end
    methods
        % NN is total number of time pts
        function self = EKalFilt(mu0, sig0, sig_r, sig_th, alph, dt)
            self.mu = mu0;
            self.sig = sig0;
            self.Q = [sig_r^2, 0; 0, sig_th^2];
            self.alph = alph;
            self.dt = dt;
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
    end
end

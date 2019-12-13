classdef EKF_GPS < EKF_unicycle
    methods
        function self = EKF_GPS(mu0, sig0, alph, Q, dt)
        % Subclass of EKF_unicycle that uses a GPS measurement update
        
            % Call superclass constructor
            self@EKF_unicycle(mu0, sig0, alph, Q, dt);
        end
        
        function self = correct(self,z)
        % Corrects the Kalman filter using a GPS measurement
        
            % The predicted observation is just the estimated position
            zhat = self.mu(1:2);
            
            % Since the GPS device directly returns our first two states,
            % the derivative of the measurement with respect to state is
            % straightforward
            H = [1 0 0; 0 1 0];
            
            % Calculate Kalman gain
            S = H*self.sig*H' + self.Q;
            K = self.sig*H'/S;
            
            % Update estimate
            self.mu = self.mu + K*(z-zhat);
            self.sig = (eye(length(self.sig))-K*H)*self.sig;
        end
    end
end
classdef PIDControl < handle
    %----------------------------
    properties
        % Gains
        kp         % Proportional
        ki         % Integrator
        kd         % Derivative
        limit      % Force limit
        beta       % Dirty derivative value
        Ts         % Time step
        
        % Values
        error_dot  % Derivative of error
        error_d1   % Previous error
        integrator % Integrator value
    end
    %----------------------------
    methods
        %----------------------------
        function self = PIDControl(kp, ki, kd, limit, beta, Ts)
            self.kp = kp;                 % Proportional control gain
            self.ki = ki;                 % Integral control gain
            self.kd = kd;                 % Derivative control gain
            self.limit = limit;           % The output will saturate at this limit
            self.beta = beta;             % Dirty derivative weighting value
            self.Ts = Ts;                 % Time step

            self.error_dot = 0.0;         % Estimated derivative of error
            self.error_d1 = 0.0;          % Error delayed by one sample
            self.integrator = 0.0;        % Value of the integrator
        end
         %----------------------------
        function u = PID(self, error)
            % Integrate first so error_d1 is updated correctly before
            % differentiation
            self.integrateError(error);
            self.differentiateError(error);

            % Unsaturated force
            u_unsat = self.kp*error...
                    + self.ki*self.integrator...
                    + self.kd*self.error_dot;
            
            % Saturate control signal
            u = self.saturate(u_unsat);
            self.integratorAntiWindup(u, u_unsat);
        end
       %----------------------------
        function u = PD(self, error)
            % Differentiate error and state
            self.differentiateError(error);

            % Unsaturated force
            u_unsat = self.kp*error + self.kd*self.error_dot;
            
            % Saturate control signal
            u = self.saturate(u_unsat);
        end
        %----------------------------
        function self = differentiateError(self, error)
            self.error_dot = self.beta*self.error_dot +...
                (1-self.beta)*((error - self.error_d1) / self.Ts);
            self.error_d1 = error;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator +...
                (self.Ts/2)*(error+self.error_d1);
        end
        %----------------------------
        function self = integratorAntiWindup(self, u_sat, u_unsat)
            % integrator anti-windup
            if self.ki~=0
                self.integrator = self.integrator +...
                    self.Ts/self.ki*(u_sat-u_unsat);
            end
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end








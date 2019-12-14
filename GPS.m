classdef GPS
    properties
        xerr % The current error in the x direction
        yerr % The current error in the y direction
        dt   % Simulation time step
    end
    methods
        function self = GPS(dt)
        % GPS class that, given an agent's actual position, returns
        % simulated GPS measurements based on a constant moving bias and
        % noise parameters taken from Small Unmanned Aircraft, and reduced
        % by 1/3 to simulate recent advances
        
            self.dt = dt;
            % Initialize bias at a random value
            self.xerr = randn()*4.7/3;
            self.yerr = randn()*4.7/3;
        end
        
        function est = getGPS(self,X)
        % Returns the XY state input with noise to simulate a GPS
        % measurement
        
            % Add noise to the current error estimates according to a
            % Gauss-Markov process
            self.xerr = exp(-1/550*self.dt)*self.xerr + 0.21/3*randn();
            self.yerr = exp(-1/550*self.dt)*self.yerr + 0.21/3*randn();
            est = [X(1)+self.xerr; X(2)+self.yerr];
        end
    end
end
classdef GPS
    properties
        xerr
        yerr
        dt
    end
    methods
        function self = GPS(dt)
            self.dt = dt;
            self.xerr = randn()*4.7/3;
            self.yerr = randn()*4.7/3;
        end
        
        function est = getGPS(self,X)
            self.xerr = exp(-1/550*self.dt)*self.xerr + 0.21/3*randn();
            self.yerr = exp(-1/550*self.dt)*self.yerr + 0.21/3*randn();
            est = [X(1)+self.xerr; X(2)+self.yerr];
        end
    end
end
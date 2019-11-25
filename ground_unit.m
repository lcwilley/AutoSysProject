classdef ground_unit < handle
    properties
        X % Unit state
        plotHandles % Unit plot handles
        w % Plotting width
        h % Plotting height
    end
    methods
        function self = ground_unit()
        % A ground unit superclass to help in the creation of enemy and
        % allied units
        end
        
        function est_pos = getGPS(self)
            % Called by either an individual agent or an array of similar
            % agents. Returns the agent's position, introducing noise of up
            % to three meters to simulate GPS accuracy.
            
            % Preallocate the return value and loop through self to account
            % for the case when this function is called by an agent array
            est_pos = zeros(2,length(self));
            for i = 1:length(self)
                est_pos(:,i) = self(i).X(1:2) + rand(2,1)*3;
            end
        end
        
        function pos = getPos(self)
            % Called by either an individual agent or an array of similar
            % agents. Returns the agent's actual position.
            
            % Preallocate the return value and loop through self to account
            % for the case when this function is called by an agent array
            pos = zeros(2,length(self));
            for i = 1:length(self)
                pos(:,i) = self(i).X(1:2);
            end
        end
    end
end
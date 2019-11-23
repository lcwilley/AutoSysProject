classdef allied_unit < handle
    properties
        X % Unit state
        plotHandles % Unit plot handles
    end
    methods
        function self = allied_unit(P)
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            self.animate();
        end
        function est_pos = getGPS(self)
            % This allows for the function to be called by an agent array
            est_pos = zeros(2,length(self));
            for i = 1:length(self)
                est_pos(:,i) = self(i).X(1:2) + rand(2,1)*3;
            end
        end
        function pos = getPos(self)
            % This allows for the function to be called by an agent array
            pos = zeros(2,length(self));
            for i = 1:length(self)
                pos(:,i) = self(i).X(1:2);
            end
        end
        function self = animate(self)
            w = 1;
            h = 0.5;
            
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Determine plotting points
            box_points = [-w/2,w/2,w/2,-w/2;
                          -h/2,-h/2,h/2,h/2];
            line1_points = [-w/2,w/2;-h/2,h/2];
            line2_points = [-w/2,w/2;h/2,-h/2];
            dot_points = [0;h/3];
            
            R = [cos(th), -sin(th); sin(th), cos(th)];
            rot_box = [x;y]+R*box_points;
            rot_line1 = [x;y]+R*line1_points;
            rot_line2 = [x;y]+R*line2_points;
            rot_dot = [x;y]+R*dot_points;
            
            % Plot the allied unit
            if isempty(self.plotHandles)
                self.plotHandles = gobjects(1,4);
                self.plotHandles(1) = fill(rot_box(1,:),rot_box(2,:),...
                    [0.2039,0.3647,0.6627]);
                hold on;
                self.plotHandles(2) = fill(rot_line1(1,:),rot_line1(2,:),'k');
                self.plotHandles(3) = fill(rot_line2(1,:),rot_line2(2,:),'k');
                self.plotHandles(4) = scatter(rot_dot(1),rot_dot(2),15,'ko','filled');
            else
                self.plotHandles(1).XData = rot_box(1,:);
                self.plotHandles(1).YData = rot_box(2,:);
                self.plotHandles(2).XData = rot_line1(1,:);
                self.plotHandles(2).YData = rot_line1(2,:);
                self.plotHandles(3).XData = rot_line2(1,:);
                self.plotHandles(3).YData = rot_line2(2,:);
                self.plotHandles(4).XData = rot_dot(1);
                self.plotHandles(4).YData = rot_dot(2);
            end
        end
    end
end
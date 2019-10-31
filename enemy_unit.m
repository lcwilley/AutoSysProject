classdef enemy_unit < handle
    properties
        X % Unit state
        plotHandles % Unit plot handles
    end
    methods
        function self = enemy_unit(P)
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            self.animate();
        end
        function self = animate(self)
            w = 0.5;
            h = 0.5;
            
            % Unpack state
            x = self.X(1);
            y = self.X(2);
            th = self.X(3);
            
            % Determine plotting points
            box_points = [-w/2,0,w/2,0; 0,h/2,0,-h/2];
            line1_points = [-w/4,w/4; -h/4,h/4];
            line2_points = [-w/4,w/4; h/4,-h/4];
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
                    [0.8431,0.1961,0.1608]);
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
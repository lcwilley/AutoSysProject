classdef UAV < handle
    properties
        X % UAV state
        plotHandles % UAV plot handles
    end
    methods
        function self = UAV(P)
            self.X = [P.x0;
                      P.y0;
                      P.th0];
            self.animate();
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
            line_points = [-w/4,-w/4,0,w/4,w/4,0;
                           -h/4,0,h/3,0,-h/4,0];
            R = [cos(th), -sin(th); sin(th), cos(th)];
            rot_box = [x;y]+R*box_points;
            rot_line = [x;y]+R*line_points;
            
            % Plot the UAV
            if isempty(self.plotHandles)
                % On the first call, plot the ground controller
                fill([-w/2,w/2,w/2,-w/2],[-h/2,-h/2,h/2,h/2],...
                      [0.2039,0.3647,0.6627]);
                fill([-w/4,-w/4,0,w/4,w/4,0],...
                    [h/4,0,-h/3,0,h/4,0],'k');
                plot([-w/2,w/2],[-h/2,h/2],'k','LineWidth',2);
                plot([-w/2,w/2],[h/2,-h/2],'k','LineWidth',2);
                scatter(0,h/3,15,'ko','filled')

                % Setup the UAV plot
                self.plotHandles = gobjects(1,2);
                self.plotHandles(1) = fill(rot_box(1,:),rot_box(2,:),...
                    [0.2039,0.3647,0.6627]);
                hold on;
                self.plotHandles(2) = fill(rot_line(1,:),rot_line(2,:),'k');
            else
                % Update the UAV plot
                self.plotHandles(1).XData = rot_box(1,:);
                self.plotHandles(1).YData = rot_box(2,:);
                self.plotHandles(2).XData = rot_line(1,:);
                self.plotHandles(2).YData = rot_line(2,:);
            end
        end
        function self = plot_self(self)
            
        end
    end
end
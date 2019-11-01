%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name: Main.m                                   %
% Authors: Carsten Christensen and Landon Willey %
% Date: 31 Oct 2019                              %
% Purpose: Loads relevant data, then runs the    %
%  simulation of a UAV simultaneously localizing %
%  itself in the environment and tracking enemy  %
%  forces.                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc

% Load in parameters for the agents
agent_params

% Create instances of each agent using corresponding constructors
clf;
uav = UAV(P_uav,P.dt);
ally(1) = allied_unit(P_ally1);
ally(2) = allied_unit(P_ally2);
enemy(1) = enemy_unit(P_enemy1);
enemy(2) = enemy_unit(P_enemy2);
axis([-7,7,-4,12]);
axis equal

% Main simulation loop
% This will eventually be a while loop
target_pt = [-5+rand*10,rand*10];
uav.setTarget(target_pt(1),target_pt(2));
target_plot = scatter(target_pt(1),target_pt(2),'go','filled');
for t = 1:1000
    uav.move_to_target();
    uav.animate();
    if mod(t,100) == 0 && t ~= 1000
        target_pt = [-5+rand*10,rand*10];
        uav.setTarget(target_pt(1),target_pt(2));
        set(target_plot,'XData',target_pt(1),'YData',target_pt(2));
    end
    pause(0.01)
end








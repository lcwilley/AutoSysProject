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
clf; % Plots are setup in agent constructors, so clear any pre-existing data
% Store allied unites in an arried
ally(1) = allied_unit(P_ally1,P_size);
ally(2) = allied_unit(P_ally2,P_size);
% Store enemy units in an array
enemy(1) = enemy_unit(P_enemy1,P_size);
enemy(2) = enemy_unit(P_enemy2,P_size);
% Create the UAV (also creates stationary ground controller)
uav = UAV(P_uav,P_size,ally,enemy,P.dt);
% Format visualization
axis([-70,70,-20,120]);
axis equal

% Main simulation loop
% This will eventually be a while loop
% Create a random target, and have the UAV move toward it
target_pt = [-50+rand*100,rand*100];
uav.setTarget(target_pt(1),target_pt(2));
% Plot the target on the map
target_plot = scatter(target_pt(1),target_pt(2),'go','filled');
% Loop through the simulation
for t = 1:2000
    % Have the UAV takea  step toward its goal position
    uav.move_to_target();
    uav.track();
    % Update the UAV animation
    uav.animate();
    % Every 100 time steps, change the target position
    if mod(t,200) == 0 && t ~= 2000
        target_pt = [-5+rand*50,rand*50];
        uav.setTarget(target_pt(1),target_pt(2));
        set(target_plot,'XData',target_pt(1),'YData',target_pt(2));
    end
    % Pause for the visualization
    pause(0.001)
end








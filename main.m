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

rng(0); % Fix the random number generator for debugging
% Load in parameters for the agents
agent_params

% Create instances of each agent using corresponding constructors
clf; % Plots are setup in agent constructors, so clear any pre-existing data
% Store allied unites in an arried
ally(1) = allied_unit(P_ally1,P);
ally(2) = allied_unit(P_ally2,P);
% Store enemy units in an array
enemy(1) = enemy_unit(P_enemy1,P);
enemy(2) = enemy_unit(P_enemy2,P);
% Create the UAV (also creates stationary ground controller)
uav = UAV(P_uav,P,ally,enemy);
% Format visualization
axis([-700,700,-200,1200]);
axis equal

% Main simulation loop
% This will eventually be a while loop
% Create random targets for each enemy
enemy_target_pts = [-500+rand(1,2)*1000;rand(1,2)*1000];
uav.setTarget();
enemy.setTarget(enemy_target_pts);
ally.setTarget(enemy_target_pts);
% Plot the target on the map
target_plots = gobjects(1,2);
target_plots(1) = scatter(enemy_target_pts(1,1),enemy_target_pts(2,1),...
    'go','filled');
target_plots(2) = scatter(enemy_target_pts(1,2),enemy_target_pts(2,2),...
    'gd','filled');
% Loop through the simulation
for t = 1:2000
    % Have the UAV move toward its goal position
    uav.move_to_target();
    uav.track();
    % Update the UAV animation
    uav.animate();
    % Have the ground units take a step towards their goal position
    enemy.moveAgent();
    ally.moveAgent();
    % Update the ground unit positions
    enemy.animate();
    ally.animate();
    
    % Every 100 time steps, change the enemy target position
    if mod(t,400) == 0 && t ~= 2000
        enemy_target_pts = [-50+rand(1,2)*100;rand(1,2)*100];
        enemy.setTarget(enemy_target_pts);
        ally.setTarget(enemy_target_pts);
        set(target_plots(1),'XData',enemy_target_pts(1,1),...
            'YData',enemy_target_pts(2,1));
        set(target_plots(2),'XData',enemy_target_pts(1,2),...
            'YData',enemy_target_pts(2,2));
    end
    % Pause for the visualization
    pause(0.001)
end








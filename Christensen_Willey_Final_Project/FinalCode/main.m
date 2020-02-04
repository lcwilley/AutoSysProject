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

film = 0;

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
axis([-800,800,-300,1300]);
axis equal

% Main simulation loop
% This will eventually be a while loop
% Have both enemies start by moving toward the UAV base
enemy_target_pts = [0 0; 0 0];
uav.setTarget();
enemy.setTarget(enemy_target_pts);
t = 0;
if film
    saveas(gcf,['frames/',num2str(t),'.png'],'png');
end
% Loop through the simulation
while ~isempty(enemy)
    % Check for capture
    for i = 1:length(enemy)
        capture_radius = 10;
        dists = pdist([enemy(i).getPos()';ally.getPos()']);
        if dists(1) < capture_radius || dists(2) < capture_radius
            enemy(i).unanimate();
            enemy(i) = [];
            uav.capturedEnemy(i);
            ally(dists < 10).setTarget([0;0]);
        end
    end
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
    % Pause for the visualization
    pause(0.001)
    % Increment time count
    t = t + 1;
    % Export video frame
    if film
        saveas(gcf,['frames/',num2str(t),'.png'],'png');
    end
end







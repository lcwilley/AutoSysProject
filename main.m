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
uav = UAV(P_uav);
% axis([-5,5,-1,4]);
axis equal
ally(1) = allied_unit(P_ally1);
ally(2) = allied_unit(P_ally2);
enemy(1) = enemy_unit(P_enemy1);
enemy(2) = enemy_unit(P_enemy2);
% 
% % Main simulation loop
% % This will eventually be a while loop
% for t = 1:1000
%     
% end








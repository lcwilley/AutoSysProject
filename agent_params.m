%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name: agent_params.m                           %
% Authors: Carsten Christensen and Landon Willey %
% Date: 31 Oct 2019                              %
% Purpose: Initializes starting data for each    %
%  agent.                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

P_uav.x0 = 0;
P_uav.y0 = 0;
P_uav.th0 = 0;
P_uav.th_max = 0.1;
% We assume the UAV controller is always located at 0,0

P_ally1.x0 = 3;
P_ally1.y0 = 2;
P_ally1.th0 = pi/5;
P_ally2.x0 = -3;
P_ally2.y0 = 2;
P_ally2.th0 = -pi/5;

P_enemy1.x0 = 5;
P_enemy1.y0 = 10;
P_enemy1.th0 = 2*pi/3;
P_enemy2.x0 = -5;
P_enemy2.y0 = 10;
P_enemy2.th0 = -2*pi/3;



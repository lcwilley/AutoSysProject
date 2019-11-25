%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name: agent_params.m                           %
% Authors: Carsten Christensen and Landon Willey %
% Date: 31 Oct 2019                              %
% Purpose: Initializes starting data for each    %
%  agent, creating a struct for each unit with   %
%  all relevant data necessary for the agent to  %
%  operate.                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulation time step
P.dt = 0.1;

% Agent visualization variables
P_size.w = 10;
P_size.h = 5;

% UAV parameters
P_uav.x0 = 0;
P_uav.y0 = 0;
P_uav.th0 = 0;
P_uav.th_max = 0.1;
P_uav.xd = 0;
P_uav.yd = 100;
P_uav.sig_r = 0.1;
P_uav.sig_b = 0.05;
P_uav.alph = [0.1 0.1];
P_uav.v_limit = 2; % m/s
P_uav.om_limit = pi/4; % rad/s
% We assume the UAV controller is always located at 0,0

% Allied unit parameters
P_ally1.x0 = 30;
P_ally1.y0 = 20;
P_ally1.th0 = pi/5;
P_ally2.x0 = -30;
P_ally2.y0 = 20;
P_ally2.th0 = -pi/5;

% Enemy unit parameters
P_enemy1.x0 = 50;
P_enemy1.y0 = 100;
P_enemy1.th0 = 2*pi/3;
P_enemy1.xd = 0;
P_enemy1.yd = 0;
P_enemy1.v_limit = 1; % m/s
P_enemy1.om_limit = pi*2; % rad/s

P_enemy2.x0 = -50;
P_enemy2.y0 = 100;
P_enemy2.th0 = -2*pi/3;
P_enemy2.xd = 0;
P_enemy2.yd = 0;
P_enemy2.v_limit = 1; % m/s
P_enemy2.om_limit = pi*2; % rad/s



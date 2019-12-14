%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name: agent_params.m                           %
% Authors: Carsten Christensen and Landon Willey %
% Date: 31 Oct 2019                              %
% Purpose: Initializes starting data for each    %
%  agent, creating a struct for each unit with   %
%  all relevant data necessary for the agent to  %
%  operate.                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Variables for all agents
P.w = 100;
P.h = 50;
% Simulation time step
P.dt = 0.1;

% UAV parameters
P_uav.x0 = 0;
P_uav.y0 = 0;
P_uav.th0 = 0;
P_uav.th_max = 0.1;
P_uav.xd = 0;
P_uav.yd = 1000;
P_uav.sig_r = 0.1;
P_uav.sig_b = 0.05;
P_uav.alph = [0.1 0.1];
P_uav.v_limit = 15; % m/s
P_uav.om_limit = pi/6; % rad/s
P_uav.bet = pi/4; % rad
% We assume the UAV controller is always located at 0,0

% Allied unit parameters
P_ally1.x0 = -300;
P_ally1.y0 = 200;
P_ally1.th0 = 2*pi/5;
P_ally1.v_limit = 3.3; % m/s
P_ally1.om_limit = pi*2; % rad/s
P_ally1.alph = [0.1,0.01];

P_ally2 = P_ally1;
P_ally2.x0 = 300;
P_ally2.th0 = 3*pi/5;

% Enemy unit parameters
P_enemy1.x0 = -500;
P_enemy1.y0 = 1000;
P_enemy1.th0 = -pi/3;
P_enemy1.v_limit = 3.3; % m/s
P_enemy1.om_limit = pi*2; % rad/s
P_enemy1.alph = [0,0];

P_enemy2 = P_enemy1;
P_enemy2.x0 = 500;
P_enemy2.th0 = -2*pi/3;



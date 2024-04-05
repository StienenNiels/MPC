%% Tricopter MPC stabilizing (nonlinear dynamics)
clc
clear
run("parameters.m")
addpath("System_Analysis")
addpath("Functions")
addpath("Plotting")

%% Tunable variables/parameters
% simulation time
variables_struc.simTime = 5;
variables_struc.dt = 0.1;
variables_struc.Np = 10;
variables_struc.payload = false;
variables_struc.terminal_set = true;

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
variables_struc.x0 = [0 0.2 0 0 0 0.2 0 0 0 0.1 0.1 1]';

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
variables_struc.Q = 100*blkdiag(1,1,1,0.5,0.5,10,10,10,10,100,100,400);

% Input weights
% [Omega1 Omega2 Omega3 mu]
variables_struc.R = 0.1*blkdiag(1,1,1,1);

% Rate of change input weights
variables_struc.L = 0.05*blkdiag(1,1,1,10);

% Initial estimate for mhat
variables_struc.mhat = params.m;


[t, y, u, trim,Vf,l,inSet] = MPC_simulation(variables_struc,params);


%% Plot results
% plot 2D results
plot_2D_plots(t, y, u, trim, params, true);

% plot stage and terminal cost
% Zegt nog niet heel veel momenteel
% plot_cost_function(t,Vf,l,inSet);

% show 3D simulation
% Timestep of payload drop is hardcoded for now
payload = variables_struc.payload;
visualize_tricopter_trajectory(y,u,params,payload,0.1);
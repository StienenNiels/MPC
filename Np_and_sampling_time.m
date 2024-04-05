%% Tricopter MPC stabilizing (nonlinear dynamics)
%% Testing influence of parameters and variables
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
variables_struc.x0 = [0 0 0 0 0 0 0 0 0 1 0 1]';

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


%%
close all
legendStrings = {} ;
% for dt = [0.05, 0.1, 0.2]
for Np = [2, 5, 10, 20, 50]

    
    variables_struc.Np = Np;
    fieldName = sprintf('Np%i', Np);

    % Np_seconds = 1;
    % variables_struc.dt = dt;
    % variables_struc.Np = Np_seconds/dt;
    % fieldName = sprintf('dt%i', round(dt*100));

    legendStrings{end+1} = fieldName;

    run("parameters.m");
    [t, y, u, trim,~,~,~] = MPC_simulation(variables_struc,params);

    result.(fieldName).t = t;
    result.(fieldName).y = y;
    result.(fieldName).u = u;
    result.(fieldName).trim = trim;

    plot_2D_plots_consecutive(t, y, u, trim, params, true);
end

legend(legendStrings)
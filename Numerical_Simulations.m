%% Tricopter MPC stabilizing (nonlinear dynamics)
%% Testing influence of parameters and variables
clc
clear
run("parameters.m")
addpath("System_Analysis")
addpath("Functions")
addpath("Plotting")

%% Tunable variables/parameters
% Default settings for all cases
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
disp("Case 0: Default simulation")
disp("Case 1: Sampling time variation")
disp("Case 2: Prediction horizon variation")
n = input('Select a case: ');

switch n
    case 0 % Default simulation
        var_range = 1;
    case 1 % Sampling time variation
        var_range = [0.05, 0.1, 0.2];
    case 2 % Prediction horizon variation
        var_range = [2, 5, 10, 20, 50];
    otherwise % Invalid case selected
        disp('Invalid case selected')
end

close all
legendStrings = {} ;
for var = var_range
    switch n
    case 0 % Default simulation
        fieldName = sprintf('Default');
    case 1 % Sampling time variation
        Np_seconds = 1;
        variables_struc.dt = dt;
        variables_struc.Np = Np_seconds/dt;
        fieldName = sprintf('dt%i', round(dt*100));
    case 2 % Prediction horizon variation
        variables_struc.Np = var;
        fieldName = sprintf('Np%i', var);
    otherwise % Invalid case selected
        disp('Invalid case selected')
    end
    
    % Create legend entries
    legendStrings{end+1} = fieldName;

    % Reload parameters
    run("parameters.m");

    % Perform the MPC simulation
    [t, y, u, trim,~,~,~] = MPC_simulation(variables_struc,params);

    % Save the results to a structure
    result.(fieldName).t = t;
    result.(fieldName).y = y;
    result.(fieldName).u = u;
    result.(fieldName).trim = trim;

    % Initial plot (NOT the report level plots)
    plot_2D_plots_consecutive(t, y, u, trim, params, true);
end

% Add legend to the plot
legend(legendStrings)

%% Save the simulation data to a structure to reuse later
% To be added

%% Generate report level plots
% To be added

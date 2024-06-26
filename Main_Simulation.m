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
variables_struc.T_segment = 3;
variables_struc.dt = 0.1;
variables_struc.Np = 25;
variables_struc.payload_time = 5;
variables_struc.payload = false;
variables_struc.terminal_set = true;
variables_struc.trj_tracking = false;
variables_struc.trj = trajectory(variables_struc.T_segment, variables_struc.dt);
variables_struc.ots = false;

% % Figure used for verifying trajectory
% figure(85), clf;
% plot3(variables_struc.trj(10,:),variables_struc.trj(11,:),variables_struc.trj(12,:))
% %%

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
variables_struc.x0 = [0 0 0 0 0 0 0 0 0 1 0 1]';

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
variables_struc.Q = 100*blkdiag(1,1,1,0.5,0.5,10,10,10,5,500,500,300);

% Input weights
% [Omega1 Omega2 Omega3 mu]
variables_struc.R = 0.1*blkdiag(1,1,1,1);

% Rate of change input weights
variables_struc.L = 0.5*blkdiag(1,1,1,100);

% Initial estimate for mhat
variables_struc.mhat = params.m;

%%
disp("Choose a case to simulate:")
disp("Case 0: Default simulation")
disp("Case 1: Sampling time variation")
disp("Case 2: Prediction horizon variation")
disp("Case 3: Simulate n random initial conditions")
disp("Case 4: Influence of delta_u weighing matrix")
disp("Case 5: Payload dropping")
disp("Case 6: Trajectory tracking")
disp("Case 7: Trajectory tracking with payload drop")
disp("Case 8: LQR and MPC comparison")
n = input('Select a case: ');

switch n
    case 0 % Default simulation
        var_range = 1;
    case 1 % Sampling time variation
        Np_seconds = 3; % Based on simulations so far 3 is about average
        dt = [0.05 0.1, 0.2, 0.5];
        variables_struc.terminal_set = false;
        var_range = 1:size(dt,2);
    case 2 % Prediction horizon variation
        % Np = [2, 5, 10, 20, 25, 30, 35, 40, 50];
        Np = [2,5,10,25,50];
        var_range = 1:size(Np,2);
    case 3 % Test settling time based on different x0
        Nsim = input('Number of random simulations: ');
        var_range = 1:Nsim;
    case 4 % Show influence of L matrix
        L_scale = [0.05 0.5 1 5];
        var_range = 1:size(L_scale,2);
    case 5 % Payload dropping
        var_range = 1;
    case 6 % Trajectory tracking
        var_range = 1;
    case 7 % Trajectory tracking with payload
        var_range = 1;
    case 8 % LQR and MPC comparison
        var_range = 1;
    otherwise % Invalid case selected
        error('Invalid case selected')
end

close all
legendStrings = {} ;
for var = var_range
    switch n
    case 0 % Default simulation
        fieldName = sprintf('default');
        legName   = sprintf('Default');
    case 1 % Sampling time variation
        variables_struc.dt = dt(var);
        variables_struc.Np = Np_seconds/dt(var);
        fieldName = sprintf('dt%i', round(dt(var)*100));
        legName   = sprintf('$$T = %g sec$$', dt(var));
    case 2 % Prediction horizon variation
        variables_struc.Np = Np(var);
        fieldName = sprintf('Np%i', Np(var));
        legName   = sprintf('$$N_p = %i$$', Np(var));
    case 3 % Test settling time based on different x0
        variables_struc.x0 = 2*(rand(12, 1)-0.5);
        fieldName = sprintf('x0%i', var);
        legName   = sprintf('$$x0_{%i}$$', var);
    case 4 % Show influence of L matrix
        variables_struc.L = L_scale(var)*blkdiag(1,1,1,1000);
        fieldName = sprintf('L%i', L_scale(var)*100);
        legName   = sprintf('$$L_{scale} = %g$$', L_scale(var));
    case 5 % Payload dropping
        variables_struc.payload = true;
        variables_struc.payload_time = 2;
        variables_struc.simTime = 5;
        variables_struc.x0 = [0 0 0 0 0 0 0 0 0 0 0 0]';
        fieldName = sprintf('payload');
        legName   = sprintf('Payload');
    case 6 % Trajectory tracking
        variables_struc.trj_tracking = true;
        variables_struc.simTime = 7*variables_struc.T_segment;
        variables_struc.x0 = [0 0 0 0 0 -pi 0 0 0 -2 2 0]';
        fieldName = sprintf('trajectory');
        legName   = sprintf('Trajectory');
    case 7 % Trajectory tracking with payload
        variables_struc.payload = true;
        variables_struc.trj_tracking = true;
        variables_struc.simTime = size(variables_struc.trj,2)*variables_struc.dt;
        variables_struc.payload_time = 2.6*variables_struc.T_segment;
        variables_struc.x0 = [0 0 0 0 0 -pi 0 0 0 -2 2 0]';
        fieldName = sprintf('trajectory');
        legName   = sprintf('Trajectory');
    case 8 % LQR and MPC comparison
        variables_struc.x0 = [0 0 0 0 0 -pi 0 0 0 -5 5 30]';
        [t, y, u, trim] = LQR_simulation(variables_struc,params);
        plot_sampling_time(t, y, u, trim, params, true);
        legName   = sprintf('LQR');
        legendStrings{end+1} = legName;
        fieldName = sprintf('MPC');
        legName   = sprintf('MPC'); 
    otherwise % Invalid case selected
        error('Case not implemented')
    end
    
    % Create legend entries
    legendStrings{end+1} = legName;

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
    switch n
        case 1 % Sampling time variation
            plot_sampling_time(t, y, u, trim, params, true);
        case 2 % Sampling time variation
            plot_sampling_time(t, y, u, trim, params, true);
        case 3 % Sampling time variation
            plot_sampling_time(t, y, u, trim, params, true);
        case 4 % Sampling time variation
            plot_L(t, y, u, trim, params, true);
        case 8 % Plot LQR and MPC comparison
            plot_sampling_time(t, y, u, trim, params, true);
        otherwise
            plot_2D_plots_consecutive(t, y, u, trim, params, true);
    end

    % Check what the settling time is
    is_settled = y(:,[4:6, 10:12]) > -0.01 & y(:,[4:6, 10:12]) < 0.01;
    if is_settled(end)
        [id,~] = find(~is_settled,1,'last');
        settling_time(var) = (id + 1)*variables_struc.dt;
    else
        settling_time(var) = inf;
    end
    fprintf("Settling time is %.2f seconds\n", settling_time(var))
end

% Display the average settling time
avg_settling = mean(settling_time);
fprintf("Settling time is %.2f seconds on average\n", avg_settling)

%% Generate report level plots
% Add legend to the plot and set y limits
switch n
    case 1 % Sampling time variation
        plot_sampling_time_limits(params);
        legend(legendStrings, "Interpreter","latex");
    case 2 % Sampling time variation
        plot_sampling_time_limits(params);
        legend(legendStrings, "Interpreter","latex");
    case 3
        plot_sampling_time_limits(params);
    case 4 % Sampling time variation
        plot_L_limits(params);
        legend(legendStrings, "Interpreter","latex");
    case 7 % Trajectory with payload
        plot_trj(t, y, u, trim, params);
        visualize_tricopter_trajectory_vid(y,u,params,variables_struc,0.1);
    case 8 % LQR and MPC comparison
        plot_sampling_time_limits(params);
        legend(legendStrings, "Interpreter","latex");
    otherwise
        plot_2D_plots_set_limits(params);
        legend(legendStrings, "Interpreter","latex");
        visualize_tricopter_trajectory_vid(y,u,params,variables_struc,0.1);
end


%axis equal

%% Save the simulation data to a structure to reuse later
% To be added

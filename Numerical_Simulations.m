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
variables_struc.simTime = 20;
variables_struc.dt = 0.1;
variables_struc.Np = 25;
variables_struc.payload = false;
variables_struc.terminal_set = true;
variables_struc.trj_tracking = false;
Nsteps = variables_struc.simTime/variables_struc.dt;

% % Define a trajectory to follow
% variables_struc.trj = zeros(16,Nsteps);
% variables_struc.trj(10,1:Nsteps/4) = linspace(0,-2,Nsteps/4);
% variables_struc.trj(11,Nsteps/4+1:Nsteps/2) = linspace(0,2,Nsteps/4);
% variables_struc.trj(12,Nsteps/2+1:3*Nsteps/4) = linspace(0,-2,Nsteps/4);
% variables_struc.trj(10,Nsteps/4+1:end) = -2;
% variables_struc.trj(11,Nsteps/2+1:end) = 2;
% variables_struc.trj(12,3*Nsteps/4+1:end) = -2;

% Define a trajectory to follow
variables_struc.trj = zeros(16,Nsteps);
t = linspace(0, pi, Nsteps/2);
% variables_struc.trj(6,Nsteps/4+1:3*Nsteps/4) = -t;
% variables_struc.trj(6,3*Nsteps/4+1:end) = -pi;
variables_struc.trj(10,Nsteps/4+1:3*Nsteps/4) = -sin(t);
variables_struc.trj(11,Nsteps/4+1:3*Nsteps/4) = -1+cos(t);
variables_struc.trj(10,3*Nsteps/4+1:end) = linspace(0,1,Nsteps/4);
% variables_struc.trj(10,3*Nsteps/4+1:end) = 0;
variables_struc.trj(11,3*Nsteps/4+1:end) = -2;
% variables_struc.trj(12,3*Nsteps/4+1:end) = -1;

% Figure used for verifying trajectory
figure(85), clf;
plot3(variables_struc.trj(10,:),variables_struc.trj(11,:),variables_struc.trj(12,:))
%%

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
variables_struc.x0 = [0 0 0 0 0 0 0 0 0 0 0 0]';

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
variables_struc.Q = 100*blkdiag(1,1,1,0.5,0.5,10,10,10,5,100,100,300);

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
n = input('Select a case: ');

switch n
    case 0 % Default simulation
        var_range = 1;
    case 1 % Sampling time variation
        Np_seconds = 3; % Based on simulations so far 3 is about average
        dt = [0.1, 0.2, 0.5];
        var_range = 1:size(dt,2);
    case 2 % Prediction horizon variation
        % Np = [2, 5, 10, 20, 25, 30, 35, 40, 50];
        Np = [2,5,10,20,25,30,35];
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
    case 7 
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
        variables_struc.simTime = 10;
        fieldName = sprintf('payload');
        legName   = sprintf('Payload');
    case 6 % Trajectory tracking
        variables_struc.trj_tracking = true;
        variables_struc.x0 = [0 0 0 0 0 0 0 0 0 0 0 0]';
        fieldName = sprintf('trajectory');
        legName   = sprintf('Trajectory');
    case 7 % Trajectory tracking with payload
        variables_struc.payload = true;
        variables_struc.trj_tracking = true;
        variables_struc.x0 = [0 0 0 0 0 0 0 0 0 0 0 0]';
        fieldName = sprintf('trajectory');
        legName   = sprintf('Trajectory');
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
    plot_2D_plots_consecutive(t, y, u, trim, params, true);

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

% Add legend to the plot and set y limits
plot_2D_plots_set_limits(params);
legend(legendStrings, "Interpreter","latex");

%% Save the simulation data to a structure to reuse later
% To be added

%% Generate report level plots
% To be added

payload = variables_struc.payload;
visualize_tricopter_trajectory_vid(y,u,params,payload,0.1);

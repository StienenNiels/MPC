%% Tricopter MPC stabilizing (nonlinear dynamics)
clc
clear
run("parameters.m")
addpath("System_Analysis")

%% Tunable variables/parameters
% simulation time
simTime = 1;
dt = 0.1;
payload = false;

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
x0 = [0 0 0 0 0 0 0 0 0 0.1 0.1 0.1]';

% prediction horizon
N = 20; 
control_horizon = 10;

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
Q = 100*blkdiag(1,1,1,0.5,0.5,10,10,10,10,100,100,400);

% Input weights
% [Omega1 Omega2 Omega3 mu]
R = 0*blkdiag(1,1,1,1);

% Rate of change input weights
L = 0.05*blkdiag(1,1,1,10);

%Input constraints
u_cont_up = [1000;1000;1000;pi/2-params.trim.mu];
u_cont_low = [-1000;-1000;-1000;-pi/2-params.trim.mu];

%State contstraints
x_cont = [pi/2;pi/2;2*pi];

%% Define state space and check controllability
sysc = init_ss_cont(params);
check_controllability(sysc);

%% Discretize system
Tvec = simTime/dt;
sysd = c2d(sysc,dt);

A = sysd.A;
B = sysd.B;
% C = sysd.C;

%% Implement rate of change penalty
[A,B,C,Q,R,M,P,x0] = rate_change_pen(A,B,Q,R,L,x0);
sysd = ss(A,B,C,[],dt);

%% Model predictive control
x = zeros(length(A(:,1)),Tvec);
u = zeros(length(B(1,:)),Tvec);
y = zeros(length(C(:,1)),Tvec);
t = zeros(1,Tvec);

Vf = zeros(1,Tvec);                % terminal cost sequence
l = zeros(1,Tvec);                 % stage cost sequence

x(:,1) = x0';

%% Prediction model and cost function
dim.N = N;
dim.nx = size(A,1);
dim.nu = size(B,2);
dim.ny = size(C,1);
dim.ncy = 3;

[T,Tcon,S,Scon]=predmodgen(sysd,dim);            %Generation of prediction model 
[H,h,const]=costgen(T,S,Q,R,dim,x0,P,M);  %Writing cost function in quadratic form

%%
for k = 1:1:Tvec
    t(k) = (k-1)*dt;
    if ( mod(t(k),1) == 0 ) 
        fprintf('t = %d sec \n', t(k));
    end

    % determine reference states based on reference input r
    x0 = x(:,k);
    [~,h,~]=costgen(T,S,Q,R,dim,x0,P,M);

    % compute control action
    cvx_begin quiet
        variable u_N(4*N)
        minimize ( (1/2)*quad_form(u_N,H) + h'*u_N )
        % input constraints
        u_N <= repmat(u_cont_up,[N 1]);
        u_N >= repmat(u_cont_low,[N 1]);
        % state constraints
        Scon*u_N <= -Tcon*x0 + repmat(x_cont,[N 1]);
        Scon*u_N >= -Tcon*x0 - repmat(x_cont,[N 1]);
        % Additional constraints to keep the control inputs constant after the first 5 steps
        for i = control_horizon+1:N
            u_N((i-1)*4+1:i*4) == u_N((control_horizon-1)*4+1:control_horizon*4);
        end
    cvx_end

    u_N
    u(:,k) = u_N(1:4); % MPC control action

    % Simulate payload dropping without changing dynamics mpc uses
    if k == 50 && payload
        params.m = 0.8*params.m;
    end

    % apply control action  
    x(:,k+1) = simulate_dynamics(x(:,k),u(:,k),dt,params);
    y(:,k) = C*x(:,k);

    % Calculate terminal and stage cost
    [P,~,~] = dare(A,B,Q,R);
    % Shouldn't Vf be calculated at xN instead of xk?
    Vf(k) = 0.5*x(:,k)'*P*x(:,k);
    l(k) = 0.5*x(:,k)'*Q*x(:,k) + 0.5*u(:,k)'*R*u(:,k) +x(:,k)'*M*u(:,k);
end

% states_trajectory: Nx16 matrix of 12 states and 4 inputs over time
states_trajectory = y';
control_inputs = u';

%% Plot results
% plot 2D results
plot_2D_plots(t, states_trajectory, control_inputs, params);

% plot stage and terminal cost
% Zegt nog niet heel veel momenteel
% plot_cost_function(t,Vf,l);

% show 3D simulation
% Timestep of payload drop is hardcoded for now
visualize_tricopter_trajectory(states_trajectory,control_inputs,params,payload,0.1);

saved_data.t = t;
saved_data.x = states_trajectory;
saved_data.u = u;
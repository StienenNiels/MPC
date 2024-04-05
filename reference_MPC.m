%% Tricopter MPC stabilizing (nonlinear dynamics)
clc
clear
run("parameters.m")
addpath("System_Analysis")
addpath("Functions")
addpath("Plotting")

%% Tunable variables/parameters
% simulation time
simTime = 5;
dt = 0.1;
payload = false;

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
x0 = [0 0 0 0 0 0 0 0 0 0 1 0]';

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
Q = 100*blkdiag(1,1,1,0.5,0.5,10,10,10,10,100,100,400);

% Input weights
% [Omega1 Omega2 Omega3 mu]
R = 0.1*blkdiag(1,1,1,1);

% Rate of change input weights
L = 0.05*blkdiag(1,1,1,10);

Np = 10;

% Initial estimate for mhat
mhat = 1.25;

%% Define state space and check controllability
sysc = init_ss_cont(params);
check_controllability(sysc);

%% Discretize system
Tvec = simTime/dt;
sysd = c2d(sysc,dt);
check_eLQR(sysd,Q,R);

A = sysd.A;
B = sysd.B;
C = sysd.C;

%% Implement rate of change penalty
[A,B,C,Q,R,M,P,x0] = rate_change_pen(A,B,Q,R,L,x0);
sysd = ss(A,B,C,[],dt);
check_eLQR(sysd,Q,R,M);

%% Model predictive control
x = zeros(length(A(:,1)),Tvec);
u = zeros(length(B(1,:)),Tvec);
y = zeros(length(C(:,1)),Tvec);
trim = zeros(5,Tvec);
t = zeros(1,Tvec);

Vf = zeros(1,Tvec);                % terminal cost sequence
l = zeros(1,Tvec);                 % stage cost sequence

x(:,1) = x0';

%% Prediction model and cost function
dim.N = Np;
dim.nx = size(A,1);
dim.nu = size(B,2);
dim.ny = size(C,1);
dim.ncy = 3;

[A_lift,~,B_lift,~]=predmodgen(sysd,dim);            %Generation of prediction model 

x_con = [100*ones(3,1); pi/2;pi/2;2*pi; 100*ones(10,1)];
% u_cont_up = [1000;1000;1000;pi/2-params.trim.mu];
% u_cont_low = [1000;1000;1000;pi/2+params.trim.mu];

u_cont_up = [3000-params.trim.Omega1;3000-params.trim.Omega2;3000-params.trim.Omega3;pi/2-params.trim.mu];
u_cont_low = [params.trim.Omega1;params.trim.Omega2;params.trim.Omega3;pi/2+params.trim.mu];

[A_con,b_con_lim,b_con_x0,Xf_set_H,Xf_set_h] = constraint_matrices(A_lift,B_lift,u_cont_up,u_cont_low,x_con,A,B,Q,R,M,Np, true);

%%
tic
for k = 1:1:Tvec
    t(k) = (k-1)*dt;
    if ( mod(t(k),1) == 0 ) 
        fprintf('t = %d sec \n', t(k));
    end

    % determine reference states based on reference input r
    x0 = x(:,k);
    [H,h,~]=costgen(A_lift,B_lift,Q,R,dim,x0,P,M);
    b_con = b_con_lim - b_con_x0*x0;

    % solve QP problem
    warning off
    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex','LinearSolver','sparse');
    u_N = quadprog(H,h,A_con,b_con,[],[],[],[],[],opts);
    warning on

    u(:,k) = u_N(1:4); % MPC control action

    % Simulate payload dropping without changing dynamics mpc uses
    if k == 50 && payload
        params.m = params.m_tricopter;
    end
    
    [params, mhat] = estimate_trim(params, mhat);
    trim(:,k) = [params.trim.phi;
                 params.trim.mu;
                 params.trim.Omega1;
                 params.trim.Omega2;
                 params.trim.Omega3];
    if k > 45 && k < 70
        mhat;
    end
    %Input constraints
    u_cont_up = [3000-params.trim.Omega1;3000-params.trim.Omega2;3000-params.trim.Omega3;pi/2-params.trim.mu];
    u_cont_low = [params.trim.Omega1;params.trim.Omega2;params.trim.Omega3;pi/2+params.trim.mu];

    % apply control action  
    x(:,k+1) = simulate_dynamics(x(:,k),u(:,k),dt,params);
    y(:,k) = C*x(:,k);

    % Calculate terminal and stage cost
    [P,~,~] = idare(A,B,Q,R);
    % Shouldn't Vf be calculated at xN instead of xk?
    Vf(k) = 0.5*x(:,k)'*P*x(:,k);
    l(k) = 0.5*x(:,k)'*Q*x(:,k) + 0.5*u(:,k)'*R*u(:,k) +x(:,k)'*M*u(:,k);
end
toc

% states_trajectory: Nx16 matrix of 12 states and 4 inputs over time
states_trajectory = y';
control_inputs = u';
trim_inputs = trim';

%% Plot results
% plot 2D results
plot_2D_plots(t, states_trajectory, control_inputs, trim_inputs, params, true);

% plot stage and terminal cost
% Zegt nog niet heel veel momenteel
% plot_cost_function(t,Vf,l);

% show 3D simulation
% Timestep of payload drop is hardcoded for now
visualize_tricopter_trajectory(states_trajectory,control_inputs,params,payload,0.1);

saved_data.t = t;
saved_data.x = states_trajectory;
saved_data.u = u;
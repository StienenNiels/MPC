%% TRICOPTER MPC STABILIZING
clc
clear
run("parameters.m")

%% Tunable variables/parameters
% simulation time
simTime = 5;
dt = 0.1;

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
x0 = [0 0 0 0 0 0 0 0 0 0.1 0.1 0.1]';

% prediction horizon
N = 20; 

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
Q = 100*blkdiag(1,1,1,0.1,0.1,0.1,10,10,5,100,100,400);

% Input weights
% [Omega1 Omega2 Omega3 mu]
R = 0*blkdiag(1,1,1,1);

% Rate of change input weights
L = 0.05*blkdiag(1,1,1,1);

%% DEFINE STATE SPACE SYSTEM
sysc = init_ss_cont(params);
check_controllability(sysc);

%% DISCRETIZE SYSTEM
Tvec = simTime/dt;
sysd = c2d(sysc,dt);

A = sysd.A;
B = sysd.B;
C = sysd.C;

%% Implement rate of change penalty
% Augment the state with the previous control action
% This allows us to formulate it as a standard LQR problem with cross
% terms.
% For reference look at exercise set 2, problem 5
% x is now 16 states being: [u v w phi theta psi p q r X_b Y_b Z_b Omega1 Omega2 Omega3 mu]
n_x = size(A,1);
n_u = size(B,2);

Pdare = idare(A,B,Q,R);

A = [A, zeros(n_x,n_u);
     zeros(n_u,n_x), zeros(n_u,n_u)];
B = [B; eye(n_u)];
C = eye(n_x+n_u);

Q = [Q,zeros(n_x,n_u);
     zeros(n_u,n_x),L];
R = R+L;
M = -[zeros(n_u);L];
SM = size(M);
Pdare = [Pdare,zeros(n_x,n_u);
         zeros(n_u,n_x),L];

x0 = [x0;0;0;0;0];

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
[H,h,const]=costgen(T,S,Q,R,dim,x0,Pdare,M);  %Writing cost function in quadratic form

%%
%Input constraints
u_cont_up = [1000;1000;1000;pi/2-params.trim.mu];
u_cont_low = [-1000;-1000;-1000;-pi/2-params.trim.mu];

%State contstraints
x_cont = [pi/2;pi/2;2*pi];

for k = 1:1:Tvec
    t(k) = (k-1)*dt;
    if ( mod(t(k),1) == 0 ) 
        fprintf('t = %d sec \n', t(k));
    end

    % determine reference states based on reference input r
    x0 = x(:,k);
    [~,h,~]=costgen(T,S,Q,R,dim,x0,Pdare,M);

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
    cvx_end

    u(:,k) = u_N(1:4); % MPC control action

    % apply control action
    x(:,k+1) = A*x(:,k) + B*u(:,k);
    y(:,k) = C*x(:,k);
end

% states_trajectory: Nx12 matrix of trajectory of 12 states
states_trajectory = y';
control_inputs = u';

%% PLOT RESULTS
% plot 2D results
plot_2D_plots(t, states_trajectory, control_inputs);

% show 3D simulation
X = states_trajectory(:,[10 11 12 7 8 9]);
u_cont = control_inputs(:,4);
visualize_tricopter_trajectory(X,u_cont,params,0.1);

saved_data.t = t;
saved_data.x = states_trajectory;
saved_data.u = u;
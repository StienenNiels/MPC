%% TRICOPTER MPC STABILIZING

%% INIT
clc
clear

%% DEFINE STATE SPACE SYSTEM
sysc = init_ss_cont();
check_controllability(sysc);

%% DISCRETIZE SYSTEM

dt = 0.1;
% simulation time
simTime = 5;
T = simTime/dt;

sysd = c2d(sysc,dt);

A = sysd.A;
B = sysd.B;
C = sysd.C;

%% Model predictive control
%[u v w phi theta psi p q r X_b Y_b Z_b]
x0 = [0 0 0 0 0 0 0 0 0 0.1 0.1 0.1]';


r = [ 0*linspace(1,2,(T+1));
      0*ones(1,(T+1));
      0*ones(1,(T+1))];

% B_ref relates reference to states x_ref = B_ref*r
B_ref = zeros(12,3);
B_ref(10,1) = 1;
B_ref(11,2) = 1;
B_ref(12,3) = 1;

x = zeros(length(A(:,1)),T);
u = zeros(length(B(1,:)),T);
y = zeros(length(C(:,1)),T);
t = zeros(1,T);

Vf = zeros(1,T);                % terminal cost sequence
l = zeros(1,T);                 % stage cost sequence

x(:,1) = x0';

%% Prediction model and cost function
% prediction horizon
N = 20; 
% Q = 100*blkdiag(1,1,10,eye(6),100*eye(2),500);
Q = 100*blkdiag(1,1,10,eye(5),10,100*eye(2),500);
R = 0.001*blkdiag(1,1,1,1);

Qbar = kron(Q,eye(N));
Rbar = kron(R,eye(N));

dim.N = N;
dim.nx = size(A,1);
dim.nu = size(B,2);
dim.ny = size(C,1);
dim.ncy = 3;

[P,Pcon,S,Scon]=predmodgen(sysd,dim);            %Generation of prediction model 
[H,h,const]=costgen(P,S,Q,R,dim,x0);  %Writing cost function in quadratic form

%%
% mu trim
% Parameters
K_F = 1.97*10^-6;
K_M = 2.88*10^-7;
l1 = 0.2483;
mu = atan(K_M/(l1*K_F));

% % Separate saturation limits per input
% u_cont = @(s) [min(max(s, -1000), 1000); 
%                      min(max(s, -1000), 1000); 
%                      min(max(s, -1000), 1000); 
%                      min(max(s, -pi/2-mu), pi/2-mu)];

%Input constraints
u_cont_up = [1000;1000;1000;pi/2-mu];
u_cont_low = [-1000;-1000;-1000;-pi/2-mu];

%State contstraints
x_cont = [pi/2;pi/2;2*pi];

for k = 1:1:T
    t(k) = (k-1)*dt;
    if ( mod(t(k),1) == 0 ) 
        fprintf('t = %d sec \n', t(k));
    end

    % determine reference states based on reference input r
    x_ref = B_ref*r(:,k);
    x0 = x(:,k) - x_ref;
    [~,h,~]=costgen(P,S,Q,R,dim,x0);

    % compute control action
    cvx_begin quiet
        variable u_N(4*N)
        minimize ( (1/2)*quad_form(u_N,H) + h'*u_N )
        % input constraints
        u_N <= repmat(u_cont_up,[N 1]);
        u_N >= repmat(u_cont_low,[N 1]);
        % state constraints
        Scon*u_N <= -Pcon*x0 + repmat(x_cont,[N 1]);
        Scon*u_N >= -Pcon*x0 - repmat(x_cont,[N 1]);
    cvx_end

    u(:,k) = u_N(1:4); % MPC control action

    % apply control action
    x(:,k+1) = A*x(:,k) + B*u(:,k) - B_ref*r(:,k);
    y(:,k) = C*x(:,k);
end

%%
% states_trajectory: Nx12 matrix of trajectory of 12 states
states_trajectory = y';
control_inputs = u';

% PLOT RESULTS
% plot 2D results
plot_2D_plots(t, states_trajectory, control_inputs);

% plot_inputs(t,u,0.1);

% show 3D simulation
X = states_trajectory(:,[10 11 12 7 8 9]);
u_cont = control_inputs(:,4);
visualize_tricopter_trajectory(X,u_cont,0.1);

saved_data.t = t;
saved_data.x = states_trajectory;
saved_data.u = u;


%% Nonlinear dynamics graveyard
% xk = x(:,k);
% uk = u(:,k);
% 
% ODEFUN = @(t,xk) nonlinear_dynamics(xk,uk);
% [TOUT,XOUT] = ode45(ODEFUN,[0 dt], x(:,k));
% x(:,k+1) = XOUT(end,:);
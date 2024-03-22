%% TRICOPTER LQR STABILIZING

%% INIT
clc
clear

%% DEFINE STATE SPACE SYSTEM
sysc = init_ss_cont();
check_controllability(sysc);

%% DISCRETIZE SYSTEM

h = 0.01;
% simulation time
simTime = 10;
T = simTime/h;

sysd = c2d(sysc,h);

A = sysd.A;
B = sysd.B;
C = sysd.C;

%% LINEAR QUADRATIC REGULATOR

x0 = [0 0 0 0 0 0 0 0 0 1 1 1]';

% reference sequence
% r = [ zeros(1,(T+1));
%       zeros(1,(T+1));
%       zeros(1,(T+1))];

r = [ 0*linspace(1,2,(T+1));
      0*ones(1,(T+1));
      0*ones(1,(T+1))];
  
Q = blkdiag(1,1,10,0.1*eye(6),1*eye(2),100);
R = 0.0001*blkdiag(1,1,1,1);

[K,S,e] = dlqr(A,B,Q,R,[]); 

B_ref = zeros(12,3);
B_ref(10,1) = 1;
B_ref(11,2) = 1;
B_ref(12,3) = 1;

% define closed loop system with LQR control law
sysd_cl_unnormalized = ss(A-B*K,B_ref,C,[],h);

% normalize closed-loop reference tracking gains 
dcgain_cl = dcgain(sysd_cl_unnormalized);
B_ref(10,1) = 1/dcgain_cl(1,1);
B_ref(11,2) = 1/dcgain_cl(2,2);
B_ref(12,3) = 1/dcgain_cl(3,3);

x = zeros(length(A(:,1)),T);
u = zeros(length(B(1,:)),T);
y = zeros(length(C(:,1)),T);
t = zeros(1,T);

x(:,1) = x0';

% mu trim
% Parameters
K_F = 1.97*10^-6;
K_M = 2.88*10^-7;
l1 = 0.2483;
mu = atan(K_M/(l1*K_F));

% Separate saturation limits per input
sat1 = @(s) min(max(s, -1000), 1000);
sat2 = @(s) min(max(s, -1000), 1000);
sat3 = @(s) min(max(s, -1000), 1000);
sat4 = @(s) min(max(s, -pi/2-mu), pi/2-mu);

for k = 1:1:T
    t(k) = (k-1)*h;
    
    % compute control action
    u(:,k) = -K*x(:,k);  
    
    % u(:,k) = sat(u(:,k));
    u(1,k) = sat1(u(1,k));
    u(2,k) = sat2(u(2,k));
    u(3,k) = sat3(u(3,k));
    u(4,k) = sat4(u(4,k));
    
    % apply control action
    x(:,k+1) = A*x(:,k) + B*u(:,k) - B_ref*r(:,k);
    y(:,k) = C*x(:,k);
end

% states_trajectory: Nx12 matrix of trajectory of 12 states
states_trajectory = y';
control_inputs = u';

%% PLOT RESULTS
% plot 2D results
plot_2D_plots(t, states_trajectory, control_inputs);

% plot_inputs(t,u,0.1);

% show 3D simulation
X = states_trajectory(:,[10 11 12 7 8 9]);
U = control_inputs(:,4);
visualize_tricopter_trajectory(X,U,0)%.01);

saved_data.t = t;
saved_data.x = states_trajectory;
saved_data.u = u;
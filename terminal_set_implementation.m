%% Stability of linearized tricopter
 
% Given a set of linearized system matrices, this code determines the
% terminal set X_f which guarantees asymptotic stability. Finally, the
% value of X_N (calygraph X) can be be determined for a given prediction
% horizon N

%%
clc
clear
run("parameters.m")
addpath("System_Analysis")
addpath("Functions")
addpath("Plotting")

%% Initialization
fprintf('\tinitializing ... \n');
sysc = init_ss_cont(params);
% DISCRETIZE SYSTEM
        
dt = 0.1;            % sampling time (sec)
sysd = c2d(sysc,dt); % convert to disrete-time system
A = sysd.A;
B = sysd.B;
C = eye(12);                           % Measured outputs
fprintf('\tdone!\n');

%% Determine optimal LQR gain
fprintf('\tdetermining LQR optimal control action ... \n');

% Initial conditions
% [u v w phi theta psi p q r X_b Y_b Z_b]
x0 = [0 0 0 0 0 0 0 0 0 0.1 0.1 0.1]';

% State weights
% [u v w phi theta psi p q r X_b Y_b Z_b]
Q = 100*blkdiag(1,1,1,0.5,0.5,10,10,10,10,100,100,400);

% Input weights
% [Omega1 Omega2 Omega3 mu]
R = 0.1*blkdiag(1,1,1,1);

% Rate of change input weights
L = 0.05*blkdiag(1,1,1,10);

[A,B,C,Q,R,M,P,x0] = rate_change_pen(A,B,Q,R,L,x0);
sysd = ss(A,B,C,[],dt);

N = 30;                     % prediction horizon
dim.N = N;
dim.nx = size(A,1);
dim.nu = size(B,2);
dim.ny = size(C,1);
dim.ncy = 3;

[A_lift,~,B_lift,~]=predmodgen(sysd,dim);            %Generation of prediction model 

x_con = [100*ones(3,1); pi/2;pi/2;2*pi; 100*ones(10,1)];
u_cont_up = [1000;1000;1000;pi/2-params.trim.mu];
u_cont_low = [1000;1000;1000;pi/2+params.trim.mu];

[A_con,b_con_lim,b_con_x0,Xf_set_H,Xf_set_h] = constraint_matrices(A_lift,B_lift,u_cont_up,u_cont_low,x_con,A,B,Q,R,M,N, true);

%%
clc
% given X_f calculated before, we can test if a given state x0 is within
% this set or not. This is done here:

% state x0
%    [u v w phi theta psi p q r X_b Y_b Z_b O O O mu]   <-- state names
x0 = [0 0 0 0   0     0   0 0 0 0.5   0.5   1   0 0 0 0]'; % <-- state values

% check if the x-location is within X_f
inSet = all(Xf_set_H*x0 <= Xf_set_h);
if inSet
    fprintf('x0 is in the set X_f\n');
else
    fprintf('x0 is NOT in the set X_f\n');
end

fprintf('\t - N = %i (prediction horizon) \n',N);

[H,h,const]=costgen(A_lift,B_lift,Q,R,dim,x0,P,M);  %Writing cost function in quadratic form
b_con = b_con_lim - b_con_x0*x0;

% solve QP problem
warning off
opts = optimoptions('quadprog','Display','off');
u_N = quadprog(H,h,A_con,b_con,[],[],[],[],[],opts);
warning on

x(:,1) = x0;

for k = 1:N
    % obtain optimal control action at k=0
    u(:,k) = u_N((k-1)*4+1:(k-1)*4+4);
    
    % apply control action
    x(:,k+1) = A*x(:,k) + B*u(:,k);
    % x(:,k+1) = simulate_dynamics(x(:,k),u(:,k),dt,params);

end

inSet = all(Xf_set_H*x(:,N+1) <= Xf_set_h);
max(max(u))
mean(u,"all")
if inSet
    fprintf('x_N is in the set X_f\n');
    % mean(x(:,N+1))
    x(:,N+1)
else
    fprintf('x_N is NOT in the set X_f\n');
    x(:,N+1)
end

t = 1:N;
plot_2D_plots(t, x(:,1:N)', u', true, params, false);







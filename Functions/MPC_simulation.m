function [t, y, u, trim,Vf,l,inSet] = MPC_simulation(variables_struc,params)

%% Unpack structure
dt = variables_struc.dt;
simTime = variables_struc.simTime;
Q = variables_struc.Q;
R = variables_struc.R;
L = variables_struc.L;
x0 = variables_struc.x0;
Np = variables_struc.Np;
terminal_set = variables_struc.terminal_set;
payload = variables_struc.payload;
mhat = variables_struc.mhat;

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
inSet = zeros(1,Tvec);             % Whether in Xf at time step

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

[A_con,b_con_lim,b_con_x0,Xf_set_H,Xf_set_h] = constraint_matrices(A_lift,B_lift,u_cont_up,u_cont_low,x_con,A,B,Q,R,M,Np, terminal_set);

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
    % x(:,k+1) = A*x(:,k) + B*u(:,k);
    x(:,k+1) = simulate_dynamics(x(:,k),u(:,k),dt,params);
    y(:,k) = C*x(:,k);

    % Calculate terminal and stage cost
    [P,~,~] = idare(A,B,Q,R);
    inSet(k) = all(Xf_set_H*x0 <= Xf_set_h);
    Vf(k) = 0.5*x(:,k)'*P*x(:,k);
    l(k) = 0.5*x(:,k)'*Q*x(:,k) + 0.5*u(:,k)'*R*u(:,k) +x(:,k)'*M*u(:,k);
end
toc

% states_trajectory: Nx16 matrix of 12 states and 4 inputs over time
y = y';
u = u';
trim = trim';
end
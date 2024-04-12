function [t, y, u, trim] = LQR_simulation(variables_struc,params)

%% Unpack structure
dt = variables_struc.dt;
simTime = variables_struc.simTime;
Q = variables_struc.Q;
R = variables_struc.R;
L = variables_struc.L;
x0 = variables_struc.x0;

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

%% Set up LQR required code
[K,S,e] = dlqr(A,B,Q,R,M); 

% reference sequence
r = [ zeros(1,(Tvec+1));
      zeros(1,(Tvec+1));
      zeros(1,(Tvec+1))];

B_ref = zeros(16,3);
B_ref(10,1) = 1;
B_ref(11,2) = 1;
B_ref(12,3) = 1;

% define closed loop system with LQR control law
sysd_cl_unnormalized = ss(A-B*K,B_ref,C,[],dt);

% normalize closed-loop reference tracking gains 
dcgain_cl = dcgain(sysd_cl_unnormalized);
B_ref(10,1) = 1/dcgain_cl(1,1);
B_ref(11,2) = 1/dcgain_cl(2,2);
B_ref(12,3) = 1/dcgain_cl(3,3);

%% Initialize empty vectors
x = zeros(length(A(:,1)),Tvec);
u = zeros(length(B(1,:)),Tvec);
y = zeros(length(C(:,1)),Tvec);
trim = repmat([params.trim.phi;params.trim.mu;params.trim.Omega1;params.trim.Omega2;params.trim.Omega3],Tvec);
t = zeros(1,Tvec);

x(:,1) = x0';

%% Constraints
x_con = [100*ones(3,1); pi/2;pi/2;2*pi; 100*ones(10,1)];
up = [3000-params.trim.Omega1;3000-params.trim.Omega2;3000-params.trim.Omega3;pi/2-params.trim.mu];
ul = [-params.trim.Omega1;-params.trim.Omega2;-params.trim.Omega3;-pi/2-params.trim.mu];

% Separate saturation limits per input
sat1 = @(s) min(max(s, ul(1)), up(1));
sat2 = @(s) min(max(s, ul(2)), up(2));
sat3 = @(s) min(max(s, ul(3)), up(3));
sat4 = @(s) min(max(s, ul(4)), up(4));

%%
tic
ll = 0;
for k = 1:1:Tvec
    t(k) = (k-1)*dt;
    if ( mod(t(k),1) == 0 ) 
        fprintf(repmat('\b',1,ll));
        ll = fprintf('t = %d sec, %d%% done \n', t(k), round(100*t(k)/simTime));
    end

    % compute control action
    u(:,k) = -K*x(:,k);  

    % Constrain control actions
    u(1,k) = sat1(u(1,k));
    u(2,k) = sat2(u(2,k));
    u(3,k) = sat3(u(3,k));
    u(4,k) = sat4(u(4,k));

    % apply control action  
    % x(:,k+1) = A*x(:,k) + B*u(:,k) - B_ref*r(:,k);
    x(:,k+1) = simulate_dynamics(x(:,k),u(:,k),dt,params);
    y(:,k) = C*x(:,k);
end

fprintf(repmat('\b',1,ll));
fprintf('t = %d sec, 100%% done \n', simTime);
toc

% states_trajectory: Nx16 matrix of 12 states and 4 inputs over time
t = t(1:end-1);
y = y(:,2:end)';
u = u(:,2:end)';
trim = trim(:,2:end)';
end
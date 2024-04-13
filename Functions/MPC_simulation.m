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
payload_time = variables_struc.payload_time;
trj_tracking = variables_struc.trj_tracking;
ots = variables_struc.ots;
mhat = variables_struc.mhat;

if payload
    params.m = params.m + params.m_payload;
end

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

if trj_tracking
    if size(variables_struc.trj,2) ~= Tvec
        size(variables_struc.trj,2)
        Tvec
        error("Trajectory does not have the correct length")
    end
    trj = [variables_struc.trj,repmat(variables_struc.trj(:,end), 1, Np+1)];
end

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

[A_con,b_con_lim,b_con_x0,Xf_set_H,Xf_set_h,b_con_xref] = constraint_matrices(A_lift,B_lift,u_cont_up,u_cont_low,x_con,A,B,Q,R,M,Np, terminal_set);

%% OTS calculation offline
if ots
    
    N_ots = size(trj,2);
    x_ref = trj;
    psi_ref = x(6,1);
    Rpsi = [cos(psi_ref)  -sin(psi_ref);
            sin(psi_ref)  cos(psi_ref)];
    dxy = Rpsi*trj(10:11,:);
    dpsi = atan2(dxy(2,2:end)-dxy(2,1:end-1),-dxy(1,2:end)+dxy(1,1:end-1));
    x_ref(6,1:end-1) = psi_ref + dpsi;

    xu_unop = [x_ref; zeros(4,size(x_ref,2))];
    xu_unop = reshape(xu_unop,[],1);
    C_ots = zeros(16);
    C_ots(6,6) = 1;
    C_ots(10,10) = 1;
    C_ots(11,11) = 1;
    C_ots(12,12) = 1;
    
    A_eq = [eye(16)-A, -B;
            C_ots, zeros(size(C_ots,1),size(B,2))];
    B_eq = [zeros(16,size(trj,2));trj];

    u_up = [3000-params.trim.Omega1;3000-params.trim.Omega2;3000-params.trim.Omega3;pi/2-params.trim.mu];
    u_low = [-params.trim.Omega1;-params.trim.Omega2;-params.trim.Omega3;-pi/2-params.trim.mu];
    xu_up = [x_con; u_up];
    xu_lo = [-x_con; u_low];

    A_eq = kron(eye(N_ots),A_eq);
    B_eq = reshape(B_eq,[],1);
    xu_up = repmat(xu_up,N_ots,1);
    xu_lo = repmat(xu_lo,N_ots,1);

    OTS_weight = eye(20);
    OTS_weight(13:16,13:16) = 0*eye(4);
    OTS_weight(10:12,10:12) = 1e8*eye(3);

    cvx_begin
        variable xu(20*N_ots)
        minimize( (xu)'*kron(eye(N_ots),OTS_weight)*(xu))
        subject to 
            A_eq * xu == B_eq
            xu <= xu_up
            xu >= xu_lo
    cvx_end
    
    xu_ref = reshape(xu,[],N_ots);
    x_trj = xu_ref(1:16,:);
    u_trj = xu_ref(17:20,:);
    size(x_trj);
    figure(854), clf;
    subplot(1,2,1)
    plot(x_trj');
    legend();
    subplot(1,2,2)
    plot(u_trj');
    legend();
end

%%
tic
ll = 0;
for k = 1:1:Tvec
    t(k) = (k-1)*dt;
    if ( mod(t(k),1) == 0 ) 
        fprintf(repmat('\b',1,ll));
        ll = fprintf('t = %d sec, %d%% done \n', t(k), round(100*t(k)/simTime));
    end

    % determine reference states based on reference input r
    x0 = x(:,k);
    if trj_tracking && ~ots
        x_ref = trj(:,k:k+Np);
        psi_ref = x(6,k);
        Rpsi = [cos(psi_ref)  -sin(psi_ref);
                sin(psi_ref)  cos(psi_ref)];
        dxy = Rpsi*trj(10:11,k:k+Np+1);
        dpsi = atan2(dxy(2,2:end)-dxy(2,1:end-1),-dxy(1,2:end)+dxy(1,1:end-1));
        x_ref(6,:) = psi_ref + dpsi;
        x_ref = reshape(x_ref,[],1);
        [H,h,~]=costgen(A_lift,B_lift,Q,R,dim,x0,P,M,x_ref);
        b_con = b_con_lim - b_con_x0*x0 + b_con_xref*x_ref;
    elseif trj_tracking && ots
        x_ref = x_trj(:,k:k+Np);
        psi_ref = x(6,k);
        Rpsi = [cos(psi_ref)  -sin(psi_ref);
                sin(psi_ref)  cos(psi_ref)];
        dxy = Rpsi*x_trj(10:11,k:k+Np+1);
        dpsi = atan2(dxy(2,2:end)-dxy(2,1:end-1),-dxy(1,2:end)+dxy(1,1:end-1));
        x_ref(6,:) = psi_ref + dpsi;
        x_ref = reshape(x_ref,[],1);
        [H,h,~]=costgen(A_lift,B_lift,Q,R,dim,x0,P,M,x_ref);
        b_con = b_con_lim - b_con_x0*x0 + b_con_xref*x_ref;
    else
        [H,h,~]=costgen(A_lift,B_lift,Q,R,dim,x0,P,M);
        b_con = b_con_lim - b_con_x0*x0;
    end
    
    % solve QP problem
    warning off
    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex','LinearSolver','sparse');
    u_N = quadprog(H,h,A_con,b_con,[],[],[],[],[],opts);
    warning on

    u(:,k) = u_N(1:4); % MPC control action

    % Simulate payload dropping without changing dynamics mpc uses
    if k == payload_time/dt && payload
        params.m = params.m_tricopter;
    end
    
    [params, mhat] = estimate_trim(params, mhat);
    trim(:,k) = [params.trim.phi;
                 params.trim.mu;
                 params.trim.Omega1;
                 params.trim.Omega2;
                 params.trim.Omega3];

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
fprintf(repmat('\b',1,ll));
fprintf('t = %d sec, 100%% done \n', simTime);
toc

% states_trajectory: Nx16 matrix of 12 states and 4 inputs over time
t = t(1:end-1);
y = y(:,2:end)';
u = u(:,2:end)';
trim = trim(:,2:end)';
end
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

[P,K_LQR,~] = idare(A,B,Q,R,M);  % determine LQR gain for unconstrained system

A_K = A-B*K_LQR;              % closed-loop LQR system
eigvals_A_K = eig(A_K);         % determine closed-loop eigenvalues

fprintf('\tdone!\n');
%% Determine invariant set X_f
fprintf('\testimating X_f invariant set \n');

dim.nx = size(A,1);        %
dim.nu = size(B,2);         %

u_lim = [100;100;100;pi/2];      % bound on control inputs
x_lim = [10;10;10; pi/2;pi/2;2*pi; 10*ones(3,1); 5*ones(3,1)];
x_lim = [x_lim;u_lim];

[Xf_set_H,Xf_set_h] = max_control_admissable_set(A,B,K_LQR,u_lim,x_lim);

fprintf('\tSuccesfully constructed terminal set X_f\n');

%% CHECK CONSTRAINT

% given X_f calculated before, we can test if a given state x0 is within
% this set or not. This is done here:

% state x0
%    [u v w phi theta psi p q r X_b Y_b Z_b O O O mu]   <-- state names
x0 = [0 0 0 0   0     0   0 0 0 0   0   0   0 0 0 0]'; % <-- state values

% check if the x-location is within X_f
inSet = all(Xf_set_H*x0 <= Xf_set_h);
if inSet
    fprintf('\t x0 is in the set X_f\n');
else
    fprintf('\t x0 is NOT in the set X_f\n');
end

%% Test if MPC control law guides to X_f in N steps (X_N)

% Having constructed X_f, we empirically construct a state X_N (calygraph X) 
% to determine which states can be steered to the terminal set X_f in N 
% steps.

fprintf('\tDetermining X_N emperically for different Beta values ...\n');

N = 10;                     % prediction horizon
fprintf('\t - N = %i (prediction horizon) \n',N);

x = zeros(dim.nx,N+1);      % state trajectory

% initial state
% x0 = [0 0 0 0 0 0  0 0 0 0.1 0.1 0.1]';
% x0 = [0.05 0 0.1 0 0 0  0.05 0 0.4 0 0 0]';
x(:,1) = x0;

% terminal cost = unconstrained optimal cost (Lec 5 pg 6)
[S,~,~] = idare(A,B,Q,R);  % terminal cost

% determine prediction matrices
Qbar = kron(Q,eye(N));
Rbar = kron(R,eye(N));
Sbar = S;

LTI.A = A;
LTI.B = B;
LTI.C = C;

dim.N = N;
dim.nx = size(A,1);
dim.nu = size(B,2);
dim.ny = size(C,1);
dim.ncy = 3;

[T,Tcon,S,Scon]=predmodgen(sysd,dim);            %Generation of prediction model 
% Tcon = T;
% Scon = S;

%Input constraints
u_cont_up = [100;100;100;pi/2-params.trim.mu];
u_cont_low = [-100;-100;-100;-pi/2-params.trim.mu];
%State contstraints
x_cont = [pi/2;pi/2;2*pi];
% x_cont = [100*ones(3,1); pi/2;pi/2;2*pi; 100*ones(6,1);u_cont_up];

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

for i = 1:2
    if i == 1
        A_con = [Scon; -Scon];
        b_con = [-Tcon*x0 + repmat(x_cont,[N+1 1]); Tcon*x0 + repmat(x_cont,[N+1 1])];
        [H,h,const]=costgen(T,S,Q,R,dim,x0,1*P,M);  %Writing cost function in quadratic form
        fprintf('Without terminal constraint set\n')
    else
        A_con = [Scon; -Scon];
        b_con = [-Tcon*x0 + repmat(x_cont,[N+1 1]); Tcon*x0 + repmat(x_cont,[N+1 1])];
        A_con = [A_con; Xf_set_H*[zeros(size(B,1),(N-1)*size(B,2)),B]];
        b_con = [b_con; Xf_set_h];
        [H,h,const]=costgen(T,S,Q,R,dim,x0,1*P,M);  %Writing cost function in quadratic form
        fprintf('With terminal constraint set\n')
    end

    size(A_con)
    % solve QP problem
    warning off
    opts = optimoptions('quadprog','Display','off');
    u_N = quadprog(H,h,A_con,b_con,[],[],repmat(u_cont_low,[N 1]),repmat(u_cont_up,[N 1]),[],opts);
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
    end
    x_comp(:,:,i) = x;
end

% diff = mean(x_comp(:,1)-x_comp(:,2))
figure(1), clf;
plot3(x_comp(10,:,1),x_comp(11,:,1),x_comp(12,:,1)), hold on;
plot3(x_comp(10,:,2),x_comp(11,:,2),x_comp(12,:,2)), hold on;
disp("Done")

%%

% Define IC_test_vals as the set of initial conditions to consider
r = 1;
s = 1;

res_x = 10;
res_y = 10;
bnd_x_up = 2;
bnd_x_lo = -bnd_x_up;
bnd_y_up = 10;
bnd_y_lo = -bnd_y_up;

betarange = [1];
mat = zeros(res_x,res_y,size(betarange,1));
beta_i = 1;

%     % state constraints
%     Scon*u_N <= -Tcon*x0 + repmat(x_cont,[Np+1 1]);
%     Scon*u_N >= -Tcon*x0 - repmat(x_cont,[Np+1 1]);

% State constraints can als be rewritten into the form A*u_N <= b
A_con = [Scon; -Scon];
b_con = [-Tcon*x0 + repmat(x_cont,[N+1 1]); Tcon*x0 + repmat(x_cont,[N+1 1])];

A_con = [A_con; Xf_set_H*[zeros(size(B,1),9*size(B,2)),B]];
b_con = [b_con; Xf_set_h];


for betaVal = betarange
    
    P = betaVal*P;
    fprintf('\t - Beta = %d \n',betaVal)

    [H,h,const]=costgen(T,S,Q,R,dim,x0,P,M);  %Writing cost function in quadratic form

    r = 1;

    % loop through different initial conditions 
    for initial_x = linspace(bnd_x_lo,bnd_x_up,res_x)
        s = 1;
        for initial_y = linspace(bnd_y_lo,bnd_y_up,res_y)
            x(:,1) = [initial_y 0 0 0 0 0 0 0 0 initial_x 0 0 0 0 0 0]';
            % x(:,1) = [0 initial_y 0 0 0 0 0 0 0 0 initial_x 0 0 0 0 0]';
            % x(:,1) = [0 0 initial_y 0 0 0 0 0 0 0 0 initial_x 0 0 0 0]';
            for k = 1:N     
                % define current state position
                x0 = x(:,k);
                [~,h,~]=costgen(T,S,Q,R,dim,x0,P,M);
                % cvx_begin quiet
                %     variable u_N(4*Np)
                %     % Additional constraints to keep the control inputs constant after the first nc steps
                %     % U_repeat = reshape(u_N(1:4*Nc), 4, []);
                %     % u_N(4*Nc+1:end) == repmat(U_repeat(:,end), Np-Nc, 1);
                %     for i = Nc+1:Np
                %         u_N((i-1)*4+1:i*4) == u_N((Nc-1)*4+1:Nc*4);
                %     end
                %     minimize ( (1/2)*quad_form(u_N,H) + h'*u_N )
                %     % input constraints
                %     u_N <= repmat(u_cont_up,[Np 1]);
                %     u_N >= repmat(u_cont_low,[Np 1]);
                %     % state constraints
                %     Scon*u_N <= -Tcon*x0 + repmat(x_cont,[Np+1 1]);
                %     Scon*u_N >= -Tcon*x0 - repmat(x_cont,[Np+1 1]);
                % 
                % cvx_end

                warning off
                % solve QP problem
                opts = optimoptions('quadprog','Display','off');
                u_N = quadprog(H,h,A_con,b_con,[],[],repmat(u_cont_low,[N 1]),repmat(u_cont_up,[N 1]),[],opts);

                % obtain optimal control action at k=0
                u(:,k) = u_N(1:4);

                % apply control action
                % x(:,k+1) = A*x(:,k) + B*u(:,k);
                x(:,k+1) = simulate_dynamics(x(:,k),u(:,k),dt,params);
                warning on
                
            end

            inSet = all(Xf_set_H*x(:,N+1) <= Xf_set_h);
            fprintf('Is %i, %i in the set? %d\n', initial_y, initial_x, inSet);
            if mod(r,5) == 0 && s == 1
                fprintf('iteration r = %i, total in set = %i\n', r, sum(mat(:,:,beta_i),"all"));
            end
            mat(r,s,beta_i) = inSet;
            s = s+1;
        end
        r = r+1;
    end
    beta_i = beta_i + 1;
end

fprintf('\tdone!\n');

% plot the initial conditions which were steered towards the terminal set
% X_f within N steps

fprintf('\tplotting results ... \n');
% figure(2), clf;
% title('Emperical plot of X_N');
% subplot(1,3,1);
% imagesc(mat(:,:,1));
% ylabel('$\beta$ = 0.1','interpreter','latex');
% 
% subplot(1,3,2);
% imagesc(mat(:,:,2));
% ylabel('$\beta$ = 1.0','interpreter','latex');
% 
% subplot(1,3,3);
% imagesc(mat(:,:,3));
% ylabel('$\beta$ = 10','interpreter','latex');

figure(1), clf;
title('Emperical plot of X_N');
imagesc(mat(:,:,1));
ylabel('$\beta$ = 1.0','interpreter','latex');

fprintf('\tdone! \n');

%%
% % Useless plot, nice though
% %% PLOT THE CONSTRAINT SET X_f
% 
% H_xyz = Xf_set_H(:,[10,11,12]);
% 
% % Define the original list (assuming it's a row vector)
% original_list = 1:size(H_xyz,1);
% 
% % Reshape the list into a matrix with 20 rows
% locations = reshape(original_list, 40, []);
% 
% figure(13), clf;
% subplot(1,2,1)
% size(locations,2)
% for i = 1:size(locations,1)
%     plot3(H_xyz(locations(i,:),1),H_xyz(locations(i,:),2),H_xyz(locations(i,:),3),'*m-');
%     % plot3(H_xyz((i-1)*20+1:(i-1)*20+20,1), ...
%     %       H_xyz((i-1)*20+1:(i-1)*20+20,2), ...
%     %       H_xyz((i-1)*20+1:(i-1)*20+20,3),'*m');
%     hold on
% end
% i = 13
% plot3(H_xyz(locations(i,:),1),H_xyz(locations(i,:),2),H_xyz(locations(i,:),3),'*k-'); hold on;
% i = 14
% plot3(H_xyz(locations(i,:),1),H_xyz(locations(i,:),2),H_xyz(locations(i,:),3),'*k-'); hold on;
% i = 15
% plot3(H_xyz(locations(i,:),1),H_xyz(locations(i,:),2),H_xyz(locations(i,:),3),'*k-'); hold on;
% grid();
% 
% subplot(1,2,2)
% plot3(H_xyz(:,1),H_xyz(:,2),H_xyz(:,3),'*m-');
% grid();
% 
% K_3d = convhull(H_xyz);
% hold on
% trisurf(K_3d,H_xyz(:,1),H_xyz(:,2),H_xyz(:,3))
% [K,V] = convhull(H_xyz(:,1),H_xyz(:,2));
% hold on
% plot3(H_xyz(K,1),H_xyz(K,2),zeros(size(H_xyz(K,3))),'r-');
% [K,V] = convhull(H_xyz(:,1),H_xyz(:,3));
% hold on
% plot3(H_xyz(K,1),zeros(size(H_xyz(K,2))),H_xyz(K,3),'b-');
% [K,V] = convhull(H_xyz(:,2),H_xyz(:,3));
% hold on
% plot3(zeros(size(H_xyz(K,1))),H_xyz(K,2),H_xyz(K,3),'g-');

% %% PLOT OF REPORT
% %% TEST IF MPC LAW GUIDES TOWARDS X_f IN N-STEPS
% 
% % Having constructed X_f, we empirically construct a state X_N (calygraph X) 
% % to determine which states can be steered to the terminal set X_f in N 
% % steps.
% 
% fprintf('\tDetermining X_N emperically ...\n');
% 
% N = 10;                     % prediction horizon
% fprintf('\t - N = %i (prediction horizon) \n',N);
% 
% x = zeros(dim.nx,N+1);      % state trajectory
% 
% % initial state
% x0 = [0 0 0 0 0 0  0 0 0 0 0 0 ]';
% x(:,1) = x0;
% 
% % tuning weights
% Q = 10*eye(dim.nx);         % state cost
% R = 0.1*eye(dim.nu);        % input cost
% 
% % terminal cost = unconstrained optimal cost (Lec 5 pg 6)
% [S,~,~] = dare(A,B,Q,R);  % terminal cost % OLD: S = 10*eye(size(A));
% 
% % determine prediction matrices
% Qbar = kron(Q,eye(N));
% Rbar = kron(R,eye(N));
% Sbar = S;
% 
% LTI.A = A;
% LTI.B = B;
% LTI.C = C;
% 
% dim.N = N;
% dim.nx = size(A,1);
% dim.nu = size(B,2);
% dim.ny = size(C,1);
% 
% [T,Tcon,S,Scon]=predmodgen(sysd,dim);            %Generation of prediction model 
% 
% % % define input constraints
% % u_limit = 0.1;
% % lb = -u_limit*ones(4*N,1);
% % ub = u_limit*ones(4*N,1);
% 
% u_limit = [100;100;100;pi/2];      % bound on control inputs
% ub = [u_limit;u_limit;u_limit;u_limit];
% lb = -ub;
% 
% Aeq = Xf_set_H;
% beq = Xf_set_h;
% 
% % Define IC_test_vals as the set of initial conditions to consider
% r = 1;
% s = 1;
% 
% res = 25;
% bnd_x_up = 15;
% bnd_y_up = 15;
% 
% mat_plot = zeros(res,res);
% betaVal = 1;
% 
% Sbar = betaVal*S;
% 
% [H,d,const]=costgen(T,S,Q,R,dim,x0);
% 
% r = 1;
% 
% % loop through different initial conditions 
% for initial_r = linspace(-bnd_x_up,bnd_x_up,res)
%     s = 1;
%     for initial_s = linspace(-bnd_y_up,bnd_y_up,res)
%         x(:,1) = [0 initial_r  0 0 0 0  0 0 0 0 initial_s 0 ]';
%         for k = 1:N     
%             % define current state position
%             x_current = x(:,k);
%              [~,d,~]=costgen(T,S,Q,R,dim,x0);
%              warning off
%             % solve QP problem
%             opts = optimoptions('quadprog','Display','off');
%             uopt = quadprog(H,d,[],[],[],[],lb,ub,[],opts);
%             warning on
% 
%             % obtain optimal control action at k=0
%             u(:,k) = uopt(1:4);
% 
%             % apply control action
%             % x(:,k+1) = Ad*x(:,k) + Bd*u(:,k);
%             x_temp = simulate_dynamics(x(:,k),u(:,k),h,params);
%             x(:,k+1) = x_temp(1:12);
%         end
%         inSet = all(Xf_set_H*x(:,N+1) <= Xf_set_h);
%         mat_plot(r,s) = inSet;
%         % matPedro(r,s) = all(H_pedro*x(:,N+1) <= h_pedro);
%         s = s+1;
%     end
%     r = r+1;
% end
% 
% fprintf('\tdone!\n');
% 
% % plot the initial conditions which were steered towards the terminal set
% % X_f within N steps
% 
% fprintf('\tplotting results ... \n');
% figure(2);
% clf;
% hold on;
% 
% r = 1;
% for r_plot = linspace(-bnd_x_up,bnd_x_up,res)
%     s = 1;
%     for s_plot = linspace(-bnd_y_up,bnd_y_up,res)
%         if (mat_plot(r,s) == 1)
%             plot(r_plot,s_plot,'sg','MarkerFaceColor','g','MarkerEdgeColor','g','MarkerSize',8);
%         else
%             plot(r_plot,s_plot,'sr','MarkerFaceColor','r','MarkerEdgeColor','r','MarkerSize',8);
%         end
%         s = s+1;
%     end
%     r = r+1;
% end
% xlabel('r');
% ylabel('$\dot{r}$','interpreter','latex');
% grid on;
% xlim([-1.05*bnd_x_up 1.05*bnd_x_up]);
% ylim([-1.05*bnd_y_up 1.05*bnd_y_up]);
% fprintf('\tdone! \n');








%% "Terminal penalty with and without terminal constraint"
clear all
close all
clc

%% Model definition

% Define model, cost function, and bounds.
A = [2, 1; 0, 2];
B = [1, 0; 0, 1];
N = 3;

alpha = 1e-5;
Q = alpha*eye(2);
R = eye(2);

% Bounds on variables.
xlb = [-15; -inf()];
xub = [15; inf()];
ulb = [-5; -5];
uub = [5; 5];

% Find LQR.

[K, P] = dlqr(A, B, Q, R);
K = -K; % Sign convention.

% Also calculate the control law with increased penalty.
P3 = 3*P;
for i = 1:N
    [P3, K3] = riciter(A, B, Q, R, P3);
end

%% Compute X_f, X_N

% Calculate Xn and Xf (maximum LQR-invariant set) using normal penalty.
[Xn, V, Z] = findXn(A, B, K, N, xlb, xub, ulb, uub, 'lqr');
title('Terminal Penalty P');

% Also calculate Xf using increased penalty.
[Xn3, V3] = findXn(A, B, K3, 0, xlb, xub, ulb, uub, 'lqr');
hold('on');
plot(V3{1}(1,:), V3{1}(2,:), '--xk');
legstrs = [get(legend(), 'string'), {'Xf w/ 3P'}];
legend(legstrs{:});

%% Get MPC matrices.

model = struct('A', A, 'B', B, 'N', N);
constraint = Z;
penalty = struct('Q', Q, 'R', R, 'P', P);
terminal1 = Xn{1}; % Terminal set.
terminal3 = Xn3{1}; % Terminal set with increased penalty.

%%
% Remove terminal constraint and see if MPC is still stabilizing.
[x1, x2] = meshgrid(linspace(-10, 10, 15), linspace(-5, 5, 15));
x0 = [x1(:), x2(:)]';
Npts = size(x0, 2);
Nstab = 10; % Number of steps to try to declare stabilizing.
Ps = {P, P3};
names = {'one', 'three'};
Xf = {terminal1, terminal3};
x0stabilizing = struct();
terminal = []; % Remove terminal constraint.
for n = 1:length(names)
    stab = false(Npts, 1);
    penalty.P = Ps{n};
    mpcmats = []; % Need to rebuild matrices.
    
    for i = 1:Npts
        % Get initial condition.
        model.x0 = x0(:,i);

        % Otherwise, simulate MPC.
        for k = 0:Nstab
            % If initial condition is inside Xf, then the trajectory is
            % stabilized.
            if all(Xf{n}.A*model.x0 <= Xf{n}.b)
                stab(i) = true();
                break;
            end

            % Give up if we aren't stabilizing after Nstab steps.
            if k == Nstab
                break;
            end

            % Otherwise, solve MPC problem.
            [xk, uk, ~, status, mpcmats] = linearmpc(model, constraint, ...
                                                     penalty, terminal, ...
                                                     mpcmats);
            model.x0 = xk(:,2); % Advance state.
            if ~(status.optimal == 0)
                % Problem is infeasible. Give up.
                break
            end
        end
        
    end
    
    x0stabilizing.(names{n}) = x0(:,stab); % Save results to struct.
end

%% 
% Check where MPC gives infinite-horizon control law. Only need to check
% feasible points.
[x1, x2] = meshgrid(linspace(-10, 10, 25), linspace(-5, 5, 25));
x0 = [x1(:), x2(:)]';
X3 = Xn{end}; % Feasible set.
x0 = x0(:,all(bsxfun(@le, X3.A*x0, X3.b))); % Keep only feasible points.
Npts = size(x0, 2);
infhorizon = false(Npts, 1);
terminal = terminal1; % Use original terminal penalty.
mpcmats = [];
for i = 1:Npts
    model.x0 = x0(:,i);
    [xk, uk, ~, status, mpcmats] = linearmpc(model, constraint, penalty, ...
                                             terminal, mpcmats);
    if status.optimal == 0
        % Check whether the ending state is in the interior (with tolerance).
        infhorizon(i) = all(terminal.A*xk(:,end) < terminal.b - 1e-5);
    else
        warning('%d: not optimal (%s = %d)', i, status.solver, status.flag);
    end
end
x0infhorizon = x0(:,infhorizon);

% Make plots.
datasets = {
    x0infhorizon', 'Control Law = Inf. Horizon';
    x0stabilizing.one', 'Stabilizing with penalty 1/2*P';
    x0stabilizing.three', 'Stabilizing with penalty 3/2*P';
};
for i = 1:size(datasets, 1)
    figure();
    pts = datasets{i, 1};
    plot(V{end}(1,:), V{end}(2,:), '-k', pts(:,1), pts(:,2), 'xk');
    xlabel('x_1');
    ylabel('x_2');
    title(datasets{i, 2});
end

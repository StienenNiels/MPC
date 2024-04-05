function [H,h] = max_control_admissable_set(A,B,K,x_lim,u_up,u_lo)

    % Extension of algorithm 3.2 in the paper:
    % "Linear Systems with State and Control Constraints: The Theory and Application of Maximal Output Admissible Sets"
    % by Elmer G. Gilbert, and Kok Tin Tan

    % Inputs:
    % A: System matrix
    % B: Control matrix
    % K: Feedback gain for optimal LQR
    % u_lim: Control input constraints
    % x_lim: State constraints

    % Outputs:
    % H: Polyhedron representing the set of admissable states
    % h: Polyhedron representing the set of admissable states
    % such that Hx <= h represents the control invariant admissable set X_f

    % Initialization
    % Constraints
    f_max = [x_lim; u_up];
    f_min = [x_lim; u_lo]; % The negative sign is placed in H, due to the condition Hx <= h
    f = [f_max; f_min];

    % Number of constraints
    s = size(f,1);

    % Setup extended matrices
    A_ext = A - B*K;
    K_ext = [eye(size(A)); K]; % Finish this line later

    % Algorithm settings
    exit_flag = 0;
    opts = optimoptions('linprog','Display','off');
    H = [];
    h = [];
    t = 0;

    while exit_flag == 0
        fprintf('\tk = %i \n',t);
        % Solve optimization problem
        H = [H; K_ext*A_ext^t; -K_ext*A_ext^t];
        h = [h; f];

        J = [K_ext*A_ext^(t+1); -K_ext*A_ext^(t+1)];

        opt_val = zeros(1,s);

        for i = 1:s
            [~,fval,exit] = linprog(-J(i,:),H,h,[],[],[],[],opts);
            if exit == -3 % If problem is unbounded
                fval = inf; % Then fval is inf.
            end
            opt_val(i) = -fval-f(i);
        end

        % Check if solution is feasible
        if all(opt_val <= 0-eps) && exit ~= -3
            exit_flag = 1;
            fprintf('\tdone!\n');
            fprintf('\tNumber of constraints: %i \n',size(H,1));
        else
            t = t + 1;
        end
    end
end
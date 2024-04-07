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
    f = [x_lim; x_lim];

    % Number of constraints
    s = size(f,1);

    % Setup extended matrices
    A_ext = A - B*K;

    % Algorithm settings
    exit_flag = 0;
    opts = optimoptions('linprog','Display','off');
    H = [];
    h = [];
    t = 0;

    while exit_flag == 0
        if mod(t,10) == 0
            fprintf('\tX_f generation, k = %i \n',t);
        end
        % Solve optimization problem
        H = [H; A_ext^t; -A_ext^t];
        h = [h; f];

        J = [A_ext^(t+1); -A_ext^(t+1)];

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
            fprintf('\tDone! Needed %i iterations\n',t);
            fprintf('\tNumber of constraints: %i \n',size(H,1));
        else
            t = t + 1;
        end
    end
end
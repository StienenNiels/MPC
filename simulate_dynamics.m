function xkp1 = simulate_dynamics(xk,uk,dt,params)

    % Function to simulate the nonlinear dynamics with the found control
    % inputs
    
    % Trim condition values
    x_trim = [0 0 0 params.trim.phi 0 0 0 0 0 0 0 0]';
    u_trim = [params.trim.Omega1; 
               params.trim.Omega2; 
               params.trim.Omega3; 
               params.trim.mu];

    % Add trim condition to the states and inputs for simulation
    xk_plus_trim = xk(1:12) + x_trim;
    uk_plus_trim = uk + u_trim;
    
    % Simulate the nonlinear dynamics for the duration of one timestep
    odefun = @(t,xk_plus_trim) nonlinear_dynamics(xk_plus_trim,uk_plus_trim,params);
    [~,x_out] = ode45(odefun,[0 dt], xk_plus_trim);

    % Convert the output back to the linearized case by subtracting the
    % trim conditions
    xkp1 = [x_out(end,:)'-x_trim;
            uk_plus_trim-u_trim];

end
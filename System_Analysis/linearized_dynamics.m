function dx_dot = linearized_dynamics(dx, du)

    % States:
    % [w, delta phi, delta theta, delta psi, p, q, r]

    % Inputs:
    % [d_collective, d_longitudinal, d_lateral, d_pedal]

    % Parameters
    K_F = 1.97*10^-6;
    K_M = 2.88*10^-7;
    l1 = 0.2483;
    l2 = 0.1241;
    I_xx = 0.0239;
    I_yy = 0.01271;
    I_zz = 0.01273;
    m = 1.1;
    g = 9.81;
    pt = atan(-l2*K_M/(l1*(l1+l2)*K_F));

    A = [0,-g*sin(pt),0,0,0,0,0;
         0,0,0,0,1,0,0;
         0,0,0,0,0,cos(pt),-sin(pt);
         0,0,0,0,0,sin(pt),cos(pt);
         0,0,0,0,0,0,0;
         0,0,0,0,0,0,0;
         0,0,0,0,0,0,0;];
    
    B = [1/m, 0,0,0;
         0,0,0,0;
         0,0,0,0;
         0,0,0,0;
         0,0,1/I_xx,0;
         0,1/I_yy,0,0;
         0,0,0,1/I_zz];

    dx_dot = A*dx + B*du;

end
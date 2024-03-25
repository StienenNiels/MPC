function x_dot = nonlinear_dynamics(x, input, params)

    % Parameters
    m = params.m;
    I_xx = params.I_xx;
    I_yy = params.I_yy;
    I_zz = params.I_zz;
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3;
    K_F = params.K_F;
    K_M = params.K_M;
    g = params.g;
    
    % States
    u = x(1);
    v = x(2);
    w = x(3);
    phi = x(4);
    theta = x(5);
    % psi = x(6);
    p = x(7);
    q = x(8);
    r = x(9);
    % X_b = x(10);
    % Y_b = x(11);
    % Z_b = x(12);

    % Inputs
    O1 = input(1)^2;
    O2 = input(2)^2;
    O3 = input(3)^2;
    mu = input(4);

    % Forces
    F_x = 0;
    F_y = K_F*O1*sin(mu);
    F_z = -K_F*(O1*cos(mu) + O2 + O3);
    
    % Moments
    M_x = -l3*K_F*(O2 - O3);
    M_y = -l2*K_F*(O2 + O3) + l1*K_F*O1*cos(mu);
    M_z = l1*K_F*O1*sin(mu)+K_M*(-O1*cos(mu) + O2 - O3);
    
    % State equations
    u_d = r*v - q*w - g*sin(theta) + F_x/m;
    v_d = -r*u + p*w + g*cos(theta)*sin(phi) + F_y/m;
    w_d = q*u - p*v + g*cos(theta)*cos(phi) + F_z/m;
    p_d = ((I_yy - I_zz)*q*r + M_x)/I_xx;
    q_d = ((I_zz - I_xx)*p*r + M_y)/I_yy;
    r_d = ((I_yy - I_xx)*p*q + M_z)/I_zz;
    phi_d = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    theta_d = q*cos(phi) - r*sin(phi);
    psi_d = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    X_b_d = u;
    Y_b_d = v;
    Z_b_d = w;
    
    % State vector derivative
    x_dot =[u_d;
            v_d;
            w_d;
            phi_d;
            theta_d;
            psi_d;
            p_d;
            q_d;
            r_d;
            X_b_d;
            Y_b_d;
            Z_b_d];

end
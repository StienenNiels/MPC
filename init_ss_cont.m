function sysc = init_ss_cont()
    
    % Parameters
    K_F = 1.97*10^-6;
    K_M = 2.88*10^-7;
    l1 = 0.2483;
    l2 = 0.1241;
    l3 = 0.2150;
    I_xx = 0.0239;
    I_yy = 0.01271;
    I_zz = 0.01273;
    m = 1.1;
    g = 9.81;
    
    % Linearize around trim conditions
    u = 0;
    v = 0;
    w = 0;
    p = 0;
    q = 0;
    r = 0;
    psi = 0;
    theta = 0;
    phi = atan(-l2*K_M/(l1*(l1+l2)*K_F));
    mu = atan(K_M/(l1*K_F));
    Omega1 = sqrt(l2*g*m*cos(phi)/((l1+l2)*K_F*cos(mu)));
    Omega2 = sqrt(l1*g*m*cos(phi)/(2*(l1+l2)*K_F));
    Omega3 = Omega2;

    % Refer to dynamics_livescript_jacobian
    A = [0, r, -q, 0, -g*cos(theta), 0, 0, -w, v, 0, 0, 0; 
         -r, 0, p, g*cos(phi)*cos(theta), -g*sin(phi)*sin(theta), 0, w, 0, -u, 0, 0, 0; 
         q, -p, 0, -g*cos(theta)*sin(phi), -g*cos(phi)*sin(theta), 0, -v, u, 0, 0, 0, 0; 
         0, 0, 0, q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta), r*cos(phi)*(tan(theta)^2 + 1) + q*sin(phi)*(tan(theta)^2 + 1), 0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta), 0, 0, 0; 
         0, 0, 0, - r*cos(phi) - q*sin(phi), 0, 0, 0, cos(phi), -sin(phi), 0, 0, 0; 
         0, 0, 0, (q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta), (r*cos(phi)*sin(theta))/cos(theta)^2 + (q*sin(phi)*sin(theta))/cos(theta)^2, 0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta), 0, 0, 0; 
         0, 0, 0, 0, 0, 0, 0, (r*(- I_zz + I_yy))/I_xx, (q*(- I_zz + I_yy))/I_xx, 0, 0, 0; 
         0, 0, 0, 0, 0, 0, -(r*(- I_zz + I_xx))/I_yy, 0, -(p*(- I_zz + I_xx))/I_yy, 0, 0, 0; 
         0, 0, 0, 0, 0, 0, -(q*(- I_yy + I_xx))/I_zz, -(p*(- I_yy + I_xx))/I_zz, 0, 0, 0, 0; 
         1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
         0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 
         0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    B = [0, 0, 0, 0; 
         (2*K_F*Omega1*sin(mu))/m, 0, 0, (K_F*Omega1^2*cos(mu))/m; 
         -(2*K_F*Omega1*cos(mu))/m, -(2*K_F*Omega2)/m, -(2*K_F*Omega3)/m, (K_F*Omega1^2*sin(mu))/m; 
         0, 0, 0, 0; 
         0, 0, 0, 0; 
         0, 0, 0, 0; 
         0, -(2*K_F*Omega2*l3)/I_xx, (2*K_F*Omega3*l3)/I_xx, 0; 
         (2*K_F*Omega1*l1*cos(mu))/I_yy, -(2*K_F*Omega2*l2)/I_yy, -(2*K_F*Omega3*l2)/I_yy, -(K_F*Omega1^2*l1*sin(mu))/I_yy; 
         -(2*K_M*Omega1*cos(mu) - 2*K_F*Omega1*l1*sin(mu))/I_zz, (2*K_M*Omega2)/I_zz, -(2*K_M*Omega3)/I_zz, (K_M*Omega1^2*sin(mu) + K_F*Omega1^2*l1*cos(mu))/I_zz; 
         0, 0, 0, 0; 
         0, 0, 0, 0; 
         0, 0, 0, 0];

    C = eye(size(A));

    % continuous-time state-space model
    sysc = ss(A,B,C,[]);

end
clear
clc

% inputs % The omega are squared!
syms mu Omega1 Omega2 Omega3

% parameters
syms K_F K_M l1 l2 l3 g I_xx I_yy I_zz m

% states
syms u v w p q r phi psi theta X_b Y_b Z_b

assume(Omega1>0 & Omega2>0 & Omega3>0)

params_sym = [K_F K_M l1 l2 l3 I_xx I_yy I_zz m g];
params_num = [1.97*10^-6 2.88*10^-7 0.2483 0.1241 0.2150 0.0239 0.01271 0.01273 1.1 9.81];


%%Forces
O1 = Omega1;
O2 = Omega2;
O3 = Omega3;

F_x = 0;
F_y = K_F*O1*sin(mu);
F_z = -K_F*(O1*cos(mu) + O2 + O3);
%%Moments
M_x = -l3*K_F*(O2 - O3);
M_y = -l2*K_F*(O2 + O3) + l1*K_F*O1*cos(mu);
M_z = l1*K_F*O1*sin(mu)+K_M*(-O1*cos(mu) + O2 - O3);

F_xt = -m*g*sin(theta);
F_yt = m*g*sin(phi)*cos(theta)+K_F*O1*sin(mu);
F_zt = m*g*cos(phi)*cos(theta)-K_F*(O1*cos(mu)+O2+O3);
M_xt = M_x;
M_yt = M_y;
M_zt = M_z;


% Total_FM = [F_xt;F_yt;F_zt;M_xt;M_yt;M_zt];
% Total_FM = subs(Total_FM,[u,v,w,p,q,r],[0,0,0,0,0,0]);
% trim = solve(Total_FM,[phi,theta,mu,Omega1,Omega2,Omega3])
% % F_xt gives theta_trim=0
% Total_FM = [F_yt;F_zt;M_xt;M_yt;M_zt];
% Total_FM = subs(Total_FM,[u,v,w,p,q,r,theta],[0,0,0,0,0,0,0]);
% trim = solve(Total_FM,[phi,mu,Omega1,Omega2,Omega3])
% % M_x gives Omega2 = Omega3
Total_FM = [F_yt;F_zt;M_yt;M_zt];
Total_FM = subs(Total_FM,[u,v,w,p,q,r,theta,Omega3],[0,0,0,0,0,0,0,Omega2])
trim = solve(Total_FM,[phi,mu,Omega1,Omega2], 'Real', true)
phi_trim = trim.phi;
phi_trim = rad2deg(double(subs(phi_trim,params_sym,params_num)))
mu_trim = trim.mu;
mu_trim = rad2deg(double(subs(mu_trim,params_sym,params_num)))
Omega1_trim = trim.Omega1;
Omega1_trim = double(sqrt(subs(Omega1_trim,params_sym,params_num)))
Omega2_trim = trim.Omega2;
Omega2_trim = double(sqrt(subs(Omega2_trim,params_sym,params_num)))


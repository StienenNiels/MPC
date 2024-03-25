%% Variables
m = 1.1; %kg Tricopter mass
m_payload = rand(1)*0.1+0.1;
m_total = m + m_payload;
I_xx = 0.0239; %kgm^2
I_yy = 0.01271;
I_zz = 0.01273;
l1 = 0.2483;
l2 = 0.1241;
l3 = 0.2150;
K_F = 1.970*10^-6;
K_M = 2.88*10^-7;
g = 9.81;

%% Variables
params.m = m;
params.m_payload = m_payload;
params.m_total = m_total;
params.I_xx = I_xx;
params.I_yy = I_yy;
params.I_zz = I_zz;
params.l1 = l1;
params.l2 = l2;
params.l3 = l3;
params.K_F = K_F;
params.K_M = K_M;
params.g = g;

%% Trim Condition
mu = atan(K_M/(l1*K_F));
phi = atan(-l2*K_M/(l1*(l1+l2)*K_F));
params.trim.mu = mu;
params.trim.phi = phi;
params.trim.theta = 0;
params.trim.psi = 0;
params.trim.Omega1 = sqrt(l2*g*m*cos(phi)/((l1+l2)*K_F*cos(mu)));
params.trim.Omega2 = sqrt(l1*g*m*cos(phi)/(2*(l1+l2)*K_F));
params.trim.Omega3 = params.trim.Omega2;

%% Remove everythin except params
clearvars -except params

function [params, mhat] = estimate_trim(params, mhat)

mhat = mhat - 0.2*(mhat - params.m);
m = mhat;

l1 = params.l1;
l2 = params.l2;
K_F = params.K_F;
K_M = params.K_M;
g = params.g;


mu = atan(K_M/(l1*K_F));
phi = atan(-l2*K_M/(l1*(l1+l2)*K_F));
params.trim.mu = mu;
params.trim.phi = phi;
params.trim.theta = 0;
params.trim.psi = 0;
params.trim.Omega1 = sqrt(l2*g*m*cos(phi)/((l1+l2)*K_F*cos(mu)));
params.trim.Omega2 = sqrt(l1*g*m*cos(phi)/(2*(l1+l2)*K_F));
params.trim.Omega3 = params.trim.Omega2;

end
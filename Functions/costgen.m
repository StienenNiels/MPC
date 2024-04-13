function [H,h,c]=costgen(T,S,Q,R,dim,x0,P,M,x_ref)

% Cost function generation
% Converts a cost function of the form:
% V_N(x(0),u_n) = 0.5*sum_{0,N-1}(x'Qx + u'Ru + 2x'Mu) + 0.5x_N'Px_N
% into a form only dependent on u_N:
% V_N(u_N) = 0.5u'Hu + h'u + c

if nargin == 6
    M = zeros(dim.nx,dim.nu);
    P = Q;
end

if nargin == 8
    x_ref = zeros(size(T,1),1);
end

% Define the bar matrices Qbar, Rbar, Mbar
Qbar = blkdiag(kron(eye(dim.N),Q),P); 
Rbar = kron(eye(dim.N),R);
Mbar = [kron(eye(dim.N),M);zeros(dim.nx,dim.N*dim.nu)];

% Calculate H,h,c
H = sparse(Rbar + S'*Qbar*S + 2*S'*Mbar);  
c = x0'*T'*Qbar*T*x0;

% Stabilizing version
% h = sparse(S'*Qbar*T*x0 + Mbar'*T*x0);

% Reference tracking version
h = sparse(S'*Qbar*(T*x0 - x_ref) + Mbar'*(T*x0 - x_ref));

end
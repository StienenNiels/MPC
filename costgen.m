function [H,h,c]=costgen(T,S,Q,R,dim,x0,P,M)

% Cost function generation
% Converts a cost function of the form:
% V_N(x(0),u_n) = 0.5*sum_{0,N-1}(x'Qx + u'Ru + 2x'Mu) + 0.5x_N'Px_N
% into a form only dependent on u_N:
% V_N(u_N) = 0.5u'Hu + h'u + c

if nargin == 6
    M = zeros(dim.nx,dim.nu);
    P = Q;
end


% Define the bar matrices Qbar, Rbar, Mbar
Qbar = kron(eye(dim.N),Q); 
Qbar(end-dim.nx+1:end,end-dim.nx+1:end) = P;
Rbar = kron(eye(dim.N),R);
Mbar = [kron(eye(dim.N),M)];
Mbar(end-dim.nx+1:end,end-dim.nu+1:end) = zeros(dim.nx,dim.nu);

% Calculate H,h,c
H = Rbar + S'*Qbar*S + 2*S'*Mbar;  
h = S'*Qbar*T*x0 + Mbar'*T*x0;
c = x0'*T'*Qbar*T*x0;

end
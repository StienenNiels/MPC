function [H,h,c]=costgen(T,S,Q,R,dim,x0,P,M)

Qbar = kron(eye(dim.N),Q); 
Qbar(end-dim.nx+1:end,end-dim.nx+1:end) = P;
Rbar = kron(eye(dim.N),R);
Mbar = [kron(eye(dim.N),M);zeros(2*dim.N*dim.nu,dim.N*dim.nu)];

H = Rbar + S'*Qbar*S + 2*S'*Mbar;  
h = S'*Qbar*T*x0 + Mbar'*T*x0;
c = x0'*T'*Qbar*T*x0;

end
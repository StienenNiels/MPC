function [T,Tcon,S,Scon]=predmodgen(LTI,dim)

% Prediction matrices generation
% This function computes the prediction matrices to be used in the
% optimization problem

% Constrained states
% [u v w phi theta psi p q r X_b Y_b Z_b Omega1 Omega2 Omega3 mu]
xcon = [4,5,6];

% Prediction matrix from initial state
T=zeros(dim.ny*(dim.N),dim.nx);
Tcon=zeros(dim.ncy*(dim.N),dim.nx);

for k=0:dim.N-1
    % Calculate slice
    Pslice = LTI.C*LTI.A^k;
    % Update full T matrix
    T(k*dim.ny+1:(k+1)*dim.ny,:)=Pslice;
    % Update constrained T matrix
    Tcon(k*dim.ncy+1:(k+1)*dim.ncy,:)=Pslice(xcon,:);
end

% Prediction matrix from input
S=zeros(dim.ny*(dim.N),dim.nu*(dim.N));
Scon=zeros(dim.ncy*(dim.N),dim.nu*(dim.N));

for k=1:dim.N-1
    for i=0:k-1
        % Calculate slice
        Sslice = LTI.C*LTI.A^(k-1-i)*LTI.B;
        % Update full S matrix
        S(k*dim.ny+1:(k+1)*dim.ny,i*dim.nu+1:(i+1)*dim.nu)= Sslice;
        % Update constrained S matrix
        Scon(k*dim.ncy+1:(k+1)*dim.ncy,i*dim.nu+1:(i+1)*dim.nu)= Sslice(xcon,:);
    end
end



function [P,Pcon,S,Scon]=predmodgen(LTI,dim)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
P=zeros(dim.ny*(dim.N),dim.nx);
Pcon=zeros(dim.ncy*(dim.N),dim.nx);
for k=0:dim.N-1
    Pslice = LTI.C*LTI.A^k;
    P(k*dim.ny+1:(k+1)*dim.ny,:)=Pslice;
    Pcon(k*dim.ncy+1:(k+1)*dim.ncy,:)=Pslice([4,5,6],:);
end

%Prediction matrix from input
S=zeros(dim.ny*(dim.N),dim.nu*(dim.N));
Scon=zeros(dim.ncy*(dim.N),dim.nu*(dim.N));
for k=1:dim.N-1
    for i=0:k-1
        Sslice = LTI.C*LTI.A^(k-1-i)*LTI.B;
        S(k*dim.ny+1:(k+1)*dim.ny,i*dim.nu+1:(i+1)*dim.nu)= Sslice;
        Scon(k*dim.ncy+1:(k+1)*dim.ncy,i*dim.nu+1:(i+1)*dim.nu)= Sslice([4,5,6],:);
    end
end



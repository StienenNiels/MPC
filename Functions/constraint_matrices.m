function [A_con,b_con_lim,b_con_x0,Xf_set_H,Xf_set_h,b_con_xref] = constraint_matrices(A_lift,B_lift,u_cont_up,u_cont_low,x_con,A,B,Q,R,M,N, terminal_set)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

F_up = blkdiag(0*eye(3),eye(3),0*eye(10));
F_lo = -F_up;
F = [F_up;F_lo];
f = [x_con; x_con];

G = [eye(4); -eye(4)];
g = [u_cont_up; u_cont_low];

[~,K_LQR,~] = idare(A,B,Q,R,M);  % determine LQR gain for unconstrained system
if terminal_set
    [Xf_set_H,Xf_set_h] = max_control_admissable_set(A,B,K_LQR,x_con,u_cont_up,u_cont_up);
else
    Xf_set_H = [eye(16);-eye(16)];
    Xf_set_h = zeros(2*16,1);
end

F_tilde = sparse(blkdiag(kron(F,eye(N)),Xf_set_H));
f_tilde = [repmat(f,[N 1]);Xf_set_h];

G_tilde = sparse(kron(G,eye(N)));
g_tilde = repmat(g,[N 1]);

A_con = sparse([F_tilde*B_lift; G_tilde]);
b_con_lim = sparse([f_tilde; g_tilde]);
b_con_x0  = sparse([F_tilde*A_lift; zeros(size(g_tilde,1),size(F_tilde*A_lift,2))]);
b_con_xref  = sparse([F_tilde; zeros(size(g_tilde,1),size(F_tilde,2))]);
end
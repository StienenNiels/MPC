function [A_con,b_con_lim,b_con_x0,b_con_xref] = constraint_matrices_ots(A_lift,B_lift,u_cont_up,u_cont_low,x_con,N)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

F_up = blkdiag(0*eye(3),eye(3),0*eye(10));
F_lo = -F_up;
F = [F_up;F_lo];
f = [x_con; x_con];

G = [eye(4); -eye(4)];
g = [u_cont_up; u_cont_low];

F_tilde = sparse(kron(F,eye(N)));
f_tilde = repmat(f,[N 1]);

G_tilde = sparse(kron(G,eye(N-1)));
g_tilde = repmat(g,[N-1 1]);

A_con = sparse([F_tilde*B_lift; G_tilde]);
b_con_lim = sparse([f_tilde; g_tilde]);
b_con_x0  = sparse([F_tilde*A_lift; zeros(size(g_tilde,1),size(F_tilde*A_lift,2))]);
b_con_xref  = sparse([F_tilde; zeros(size(g_tilde,1),size(F_tilde,2))]);
end
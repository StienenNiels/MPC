function [A,B,C,Q,R,M,P,x0] = rate_change_pen(A,B,Q,R,L,x0)

% Augment the state with the previous control action
% This allows us to formulate it as a standard LQR problem with cross
% terms.
% For reference look at exercise set 2, problem 5
% x is now 16 states being: [u v w phi theta psi p q r X_b Y_b Z_b Omega1 Omega2 Omega3 mu]
n_x = size(A,1);
n_u = size(B,2);

% Calculate terminal cost using the discrete algebraic Ricatti equation
P = idare(A,B,Q,R);

% Extend the system matrices
A = [A, zeros(n_x,n_u);
     zeros(n_u,n_x), zeros(n_u,n_u)];
B = [B; eye(n_u)];
C = eye(n_x+n_u);

% Extend the weighing matrices
Q = [Q,zeros(n_x,n_u);
     zeros(n_u,n_x),L];
R = R+L;
M = -[zeros(n_u);L];
P = [P,zeros(n_x,n_u);
         zeros(n_u,n_x),L];

% Extend the initial state
x0 = [x0;zeros(n_u,1)];
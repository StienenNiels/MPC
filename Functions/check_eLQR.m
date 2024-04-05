function check_eLQR(sysd,Q,R,M)
% Check whether the extended LQR still fullfills the required conditions in
% discrete time

% Take system matrices out of state space
A = sysd.A;
B = sysd.B;
C = sysd.C;

% If no M is given set M to 0 and treat as check for normal LQR
if nargin == 3
    M = zeros(size(B));
end

% Check controllability of the discrete system
controllable = rank(ctrb(A,B)) == size(A,1);
if controllable
    disp("The system is controllable")
else
    disp("The system is NOT controllable")
end

% Check whether R is positive definite
R_pd = all(eig(R)>0,"all");
if R_pd
    disp("R is positive definite")
else
    disp("R is NOT positive definite")
end

% Check whether the extended LQR is positive semidefinite
spd = all(Q-M/R*M'>=0,"all");
if spd
    disp("QMR is positive semidefinite")
else
    disp("QMR is NOT positive semidefinite")
end

% Check whether there are unobservable modes on the unit circle
observable = rank(obsv(A,C)) == size(A,1);
if observable
    disp("There are no unobservable modes")
else
    % Compute the matrix Q−MR^(-1)MT and A−BR^(-1)MT
    M1 = Q - M/R*M';
    M2 = A - B/R*M';

    % Check if there are unobservable modes on the unit circle
    on_unit = (abs(eig(M1))==1) || (abs(eig(M2))==1);
    if on_unit
        disp("There are unobservable modes on the unit circle")
    else
        disp("There are no unobservable modes on the unit circle")
    end
end

end
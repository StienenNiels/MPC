function check_controllability(sysc)

% Check controllability of the continuous system
Ac = sysc.A;
Bc = sysc.B;

Ctrb_rank = rank(ctrb(Ac,Bc));
% disp('Number of states');
% disp(size(Ac,1));
% disp('Rank of controllability matrix');
% disp(Ctrb_rank);
if size(Ac,1) ~= Ctrb_rank
    error("System is not controllable")
end
end
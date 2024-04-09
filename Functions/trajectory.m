function [trj] = trajectory(T_segment,dt)
% Function that generates the trajectory to follow

% Trajectory timestamps
Ts = T_segment/dt;

% Time stamps
T1 = Ts;
T2 = T1+Ts;
T31 = T2 + Ts/2;
T32 = T31 + Ts;
T33 = T32 + Ts/2;
T3 = T33;
T4 = T3+2*Ts;
T5 = T4+Ts;
T6 = T5+Ts;
T7 = T6+Ts;

% Define a trajectory to follow
trj = zeros(16,T7);

% Coordinates along route
P0 = [-2;2;0];
P1 = [-2;2;-2];
P2 = [2;2;-2];
Pwait = [2;1;-2];
P3 = [2;0;-2];
P4 = [-2;0;-2];
P5 = [-2;1;-2];
P6 = [-2;2;-1];
P7 = [-2;2;0];

% Line segment 1
X01 = zeros(1,Ts);
Y01 = zeros(1,Ts);
Z01 = linspace(0,-2,Ts);
trj(10:12,1:T1) = [X01;Y01;Z01] + P0;

% Line segment 2
X12 = linspace(0,4,Ts);
Y12 = zeros(1,Ts);
Z12 = zeros(1,Ts);
trj(10:12,T1+1:T2) = [X12;Y12;Z12] + P1;

% Line segment 3 (split up in segments)
% Move
X23 = zeros(1,Ts/2);
Y23 = linspace(0,-1,Ts/2);
Z23 = zeros(1,Ts/2);
trj(10:12,T2+1:T31) = [X23;Y23;Z23] + P2;

% wait to stabilize for drop
X23 = zeros(1,Ts);
Y23 = zeros(1,Ts);
Z23 = zeros(1,Ts);
trj(10:12,T31+1:T32) = [X23;Y23;Z23] + Pwait;

% Start moving again
X23 = zeros(1,Ts/2);
Y23 = linspace(0,-1,Ts/2);
Z23 = zeros(1,Ts/2);
trj(10:12,T32+1:T3) = [X23;Y23;Z23] + Pwait;

% Line segment 4
angle = linspace(0,pi,2*Ts);
X34 = 2*(cos(angle)-1);
Y34 = 2*(-sin(angle));
Z34 = zeros(1,2*Ts);
trj(10:12,T3+1:T4) = [X34;Y34;Z34] + P3;

% Line segment 5
X45 = zeros(1,Ts);
Y45 = linspace(0,1,Ts);
Z45 = zeros(1,Ts);
trj(10:12,T4+1:T5) = [X45;Y45;Z45] + P4;

% Line segment 6
angle = linspace(0,pi/2,Ts);
X56 = zeros(1,Ts);
Y56 = (sin(angle));
Z56 = (1-cos(angle));
trj(10:12,T5+1:T6) = [X56;Y56;Z56] + P5;

% Line segment 7
X67 = zeros(1,Ts);
Y67 = zeros(1,Ts);
Z67 = linspace(0,1,Ts);
trj(10:12,T6+1:T7) = [X67;Y67;Z67] + P6;
end
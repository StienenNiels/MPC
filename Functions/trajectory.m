function [trj] = trajectory(T_segment,dt)
% Function that generates the trajectory to follow

% Trajectory timestamps
Ts = T_segment/dt;

% Time stamps
T1 = Ts;
T2 = T1+Ts;
T31 = T2 + Ts/2;
T32 = T31 + Ts/2;
T33 = T32 + Ts/2;
T3 = T33;
T4 = T3+2*Ts;
T5 = T4+Ts/3;
T6 = T5+Ts;
T7 = T6+Ts/2;
T8 = T7+Ts/2;

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
X01 = zeros(1,T1);
Y01 = zeros(1,T1);
Z01 = linspace(0,-2,T1);
trj(10:12,1:T1) = [X01;Y01;Z01] + P0;

% Line segment 2
X12 = linspace(0,4,T2-T1);
Y12 = zeros(1,T2-T1);
Z12 = zeros(1,T2-T1);
trj(10:12,T1+1:T2) = [X12;Y12;Z12] + P1;

% Line segment 3 (split up in segments
% Move
X23 = zeros(1,T31-T2);
Y23 = linspace(0,-1,T31-T2);
Z23 = zeros(1,T31-T2);
trj(10:12,T2+1:T31) = [X23;Y23;Z23] + P2;

% wait to stabilize for drop
X23 = zeros(1,T32-T31);
Y23 = zeros(1,T32-T31);
Z23 = zeros(1,T32-T31);
trj(10:12,T31+1:T32) = [X23;Y23;Z23] + Pwait;

% Start moving again
X23 = zeros(1,T3-T32);
Y23 = linspace(0,-1,T3-T32);
Z23 = zeros(1,T3-T32);
trj(10:12,T32+1:T3) = [X23;Y23;Z23] + Pwait;

% Line segment 4
angle = linspace(0,pi,T4-T3);
X34 = 2*(cos(angle)-1);
Y34 = 2*(-sin(angle));
Z34 = zeros(1,T4-T3);
trj(10:12,T3+1:T4) = [X34;Y34;Z34] + P3;

% Line segment 5
X45 = zeros(1,T5-T4);
Y45 = linspace(0,1,T5-T4);
Z45 = zeros(1,T5-T4);
trj(10:12,T4+1:T5) = [X45;Y45;Z45] + P4;

% Line segment 6
angle = linspace(0,pi/2,T6-T5);
X56 = zeros(1,T6-T5);
Y56 = (sin(angle));
Z56 = (1-cos(angle));
trj(10:12,T5+1:T6) = [X56;Y56;Z56] + P5;

% Line segment 7
X67 = zeros(1,T7-T6);
Y67 = zeros(1,T7-T6);
Z67 = linspace(0,1,T7-T6);
trj(10:12,T6+1:T7) = [X67;Y67;Z67] + P6;

% Stationary end
X78 = zeros(1,T8-T7);
Y78 = zeros(1,T8-T7);
Z78 = zeros(1,T8-T7);
trj(10:12,T7+1:T8) = [X78;Y78;Z78] + P7 + [0;0;-0.1]; % So it stays visible in the plot
end
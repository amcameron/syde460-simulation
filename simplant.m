#!/usr/bin/octave 

% Simulation of the models generated from the Hiway 'Demon' model glider
% using the methods presented in the paper by M. V. Cook and M. Spottiswoode
% in the January 2006 issue of The Aeronautical Journal.

% Longitudinal equations:
A_long = [-0.1730  0.6538  0.1388 -9.7222;
          -1.4208 -2.2535 10.7370  1.3093;
           0.2685 -0.4402 -1.4113  0;
           0       0       1       0];
B_long = [0; 0; 7.46; 0];
C_long = eye(2, 4); % output u and w
D_long = zeros(2, 1);
sys_long = ss(A_long, B_long, C_long, D_long);

% Find state and control gains for zero steady-state error
N_long = [A_long B_long; C_long D_long]\[zeros(4, 2); ones(2, 2)];
Nx_long = N(1:size(A_long, 1), :);
Nu_long = N(size(B_long, 2), :);

% Try pole placement: for overshoot <= 5% and settling time <= 4s, we have
% theta >= 43.6° and sigma <= -1
% So, try for poles at -2+j, -2-j, -10, -15
P_long = [-2+j, -2-j, -10, -15];
K_long = place(sys_long, P_long);
disp('P:'), disp(P_long)
disp('eig(A - BK):'), disp(eig(A_long - B_long*K_long))

% Lateral equations:
A_lat = [-0.2195  -0.1580 -10.7980 9.722 -1.3098;
         -1.4670 -21.3180   7.5163 0      0;
          0.2906   3.7362  -2.1119 0      0;
          0        1        0      0      0;
          0        0        1      0      0];
B_lat = [0; 3.6136; -0.4311; 0; 0];
C_lat = eye(5); % output all state variables for now
sys_lat = ss(A_lat, B_lat, C_lat);

% Entire system all at once!
A = blkdiag(A_long, A_lat);
B = blkdiag(B_long, B_lat);
C = eye(9); % output all state variables for now
D = [];
Tsam = 0;
Ncontin = rows(A);
Ndiscr = 0;
STNAME = {"axial velocity", "normal velocity", "pitch rate", ...
        "pitch attitude", "lateral velocity", "roll rate", "yaw rate", ...
        "roll attitude", "yaw attitude"};
INNAME = {"longitudinal control angle", "lateral control angle"};
sys = ss(A, B, C, D, Tsam, Ncontin, Ndiscr, STNAME, INNAME, STNAME);

% TODO: figure out how to apply a step (i.e. what is meaningful to step?)
figure; step(sys)
figure; step(sys, 2)

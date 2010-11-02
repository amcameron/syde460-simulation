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
sys_long = ss(A_long, B_long, eye(4)); % output all state variables for now

% Lateral equations:
A_lat = [-0.2195  -0.1580 -10.7980 9.722 -1.3098;
         -1.4670 -21.3180   7.5163 0      0;
	  0.2906   3.7362  -2.1119 0      0;
	  0        1        0      0      0;
	  0        0        1      0      0];
B_lat = [0; 3.6136; -0.4311; 0; 0];
sys_long = ss(A_lat, B_lat, eye(5)); % output all state variables for now

% Entire system all at once!
A = blkdiag(A_long, A_lat);
B = blkdiag(B_long, B_lat);
C = eye(9); % output all state variables for now

% TODO: figure out how to apply a step (i.e. what is meaningful to step?)

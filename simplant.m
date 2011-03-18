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

% Lateral equations:
A_lat = [-0.2195  -0.1580 -10.7980 9.722 -1.3098;
         -1.4670 -21.3180   7.5163 0      0;
          0.2906   3.7362  -2.1119 0      0;
          0        1        0      0      0;
          0        0        1      0      0];
B_lat = [0; 3.6136; -0.4311; 0; 0];
C_lat = eye(5); % output all state variables for now
D_lat = [];
sys_lat = ss(A_lat, B_lat, C_lat, D_lat);

% Entire system all at once!
A = blkdiag(A_long, A_lat);
B = blkdiag(B_long, B_lat);
% output attitudes (but not yaw)
C = [0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 0 1 0];
D = zeros(2, 2);
STNAME = {'axial velocity', 'normal velocity', 'pitch rate', ...
        'pitch attitude', 'lateral velocity', 'roll rate', 'yaw rate', ...
        'roll attitude', 'yaw attitude'};
INNAME = {'longitudinal control angle', 'lateral control angle'};
sys = ss(A, B, C, D);

Atilde = [A zeros(9,2); C zeros(2,2)]; % 11x11
Btilde = [B; zeros(2,2)];
rank(ctrb(Atilde, Btilde)) % rank is 8, should be 11
%TODO: run PBH controllability test (ECE488 notes S5 p.24)
% -- find which modes are uncontrollable; find if system is Stabilizable
%P = 0.005*[-1, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14];
% Let's try LQR control.
% Q is the penalty for state variables. Let's penalize them  all.
Q = eye(size(Atilde));
% R is the penalty for control signals. Let's penalize them both, especially
% the longitudinal control input.
R = [500 0; 0 1];
K = lqr(Atilde, Btilde, Q, R);
disp('Q:'), disp(Q)
disp('R:'), disp(R)
disp('eig(A~ - B~*K):'), disp(eig(Atilde - Btilde*K))
disp('K:'), disp(K)

%% Prepare Nyquist and pole-zero plots for each input-state TF
%for input=1:size(B, 2)
%	subB = B(:, input);
%	for state=1:size(C, 1)
%		subC = zeros(1, size(C, 1));
%		subC(state) = 1;
%		subsys = ss(A, subB, subC, []);
%		if ((input == 1) && (state < 5)) || ((input == 2) && (state > 4))
%			figure; nyquist(subsys);
%			title([INNAME{input} ' to ' STNAME{state} ' (Nyquist)']);
%			print(sprintf('figs/%d%dnyq.eps', input, state));
%			figure; pzmap(subsys);
%			title([INNAME{input} ' to ' STNAME{state} ' (pzmap)']);
%			print(sprintf('figs/%d%dpz.eps', input, state));
%			close all;
%			disp('Poles:')
%			pole(subsys)
%			disp('Zeroes:')
%			zero(subsys)
%			pause
%		end
%	end
%end

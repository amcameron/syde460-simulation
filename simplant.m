function [A B C H K1 K2] = simplant()
% Simulation of the models generated from the Hiway 'Demon' model glider
% using the methods presented in the paper by M. V. Cook and M. Spottiswoode
% in the January 2006 issue of The Aeronautical Journal.

%% System creation
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

% Augment the system with an integrator for tracking.
Atilde = [A zeros(9,2); C zeros(2,2)]; % 11x11
Btilde = [B; zeros(2,2)];

%% PBH controllability test
e = eig(Atilde);
ctrb_ranks = zeros(size(e));
for i = 1:length(e)
	ctrb_ranks(i) = rank([(e(i)*eye(size(Atilde)) - Atilde) Btilde]);
end
%disp('PBH controllability results - all modes controllable?')
%if all(ctrb_ranks == length(e))
%    disp('Yes!')
%else
%    disp('*** NO! ***')
%end

%% Controller design
% Let's try LQR control.
% Q is the penalty for state variables. Let's penalize them  all.
Q = eye(size(Atilde));
Q(4,4) = 500;
Q(8,8) = 500;
Q(9,9) = 500;
%Q(10, 10) = 90;
%Q(11, 11) = 90;
% R is the penalty for control signals.
R = 50*[1 0; 0 1];
K = lqr(Atilde, Btilde, Q, R);
K1 = K(:, 1:9);
K2 = K(:, 10:11);
%disp('Q:'), disp(Q)
%disp('R:'), disp(R)
%disp('eig(A~ - B~*K):'), disp(eig(Atilde - Btilde*K))
%disp('K:'), disp(K)

%%% Closed-loop response of linearized system
%Acl = [A-B*K1 -B*K2; C zeros(2)];
%Bcl = [zeros(9,2); -eye(2)];
%Ccl = [C zeros(2)];
%Dcl = zeros(2);
%cls_sys = ss(Acl, Bcl, Ccl, Dcl);
%figure
%step(cls_sys)

%% PBH observability test
e = eig(A);
obs_ranks = zeros(size(e));
for i = 1:length(e)
	obs_ranks(i) = rank([C; (A-e(i)*eye(size(A)))]);
end
%disp('PBH observability results - all modes observable?')
%if all(obs_ranks == length(e))
%    disp('Yes!')
%else
%    disp('*** NO! ***')
%end

%% Observer design - angle attitudes (yaw, pitch, roll) are unknown
% Pole placement!
%H = place(A', C', -10*(1:9))';
H = 0;

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

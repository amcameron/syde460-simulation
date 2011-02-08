function xs, ts = runsim()
    % initial position
    % initial velocity
    % inital angles
    % inital angle rates
    xinit = [  2000    2000 2000 ...
	      1   1 -0.2   ...
	      pi/2      pi/64  0   ...
	      0	     0	 0   ...
    ];
    ts = linspace(0, 2, 10000);
    sim = @(x,t)sim3d(t, x);
    xs = lsode(sim, xinit, ts);
end

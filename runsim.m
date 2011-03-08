function xs, ts = runsim()
    % initial position
    % initial velocity
    % inital angles
    % inital angle rates
    xinit = [  2000    2000 2000 ...
	      -8     0   0   ...
	      0      0   0   ...
	      0	     0	 0   ...
    ];
    ts = linspace(0, 2, 10000);
    sim = @(x,t)sim3d(t, x);
    lsode_options('integration method', 'stiff');
    xs = lsode(sim, xinit, ts);
end

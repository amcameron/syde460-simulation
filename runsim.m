function xs, ts = runsim()
    % initial position
    % initial velocity
    % inital angles
    % inital angle rates
    % initial output error
	% initial state estimates (everything above, except for output error)
    xinit = [  2000    2000 2000 ...
          -10.8  0   0   ...
          0      pi/8   0   ...
          0      0   0   ...
          0      0       ...
    ];
    ts = linspace(0, 2, 1000);
    sim = @(x,t)sim3d(t, x);
    lsode_options('integration method', 'stiff');
    xs = lsode(sim, xinit, ts);
end

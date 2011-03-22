function xs, ts = runsim()
    % initial position
    % initial velocity
    % inital angles
    % inital angle rates
    % initial output error
	% initial state estimates (everything above, except for output error)
    xinit = [  2000    2000 2000 ...
          -8     0   0   ...
          0      0   0   ...
          0      0   0   ...
          0      0       ...
    ]';
	x = localize_state(xinit); % initial state, localized
    xhatinit = [ x(4) x(6) x(11) x(8) x(5) x(10) x(12) x(9) x(7) ]';
	xinit = [xinit; xhatinit];
    ts = linspace(0, 2, 1000);
    sim = @(x,t)sim3d(t, x);
    lsode_options('integration method', 'stiff');
    xs = lsode(sim, xinit, ts);
end

function xs, ts = runsim()
    % initial position
    % initial velocity
    % inital angles
    % inital angle rates
    % initial output error
    xinit = [  2000    2000 2000 ...
          -8     0   0   ...
          0      pi/16   0   ...
          0      0   0   ...
          0      0       ...
    ];
    ts = linspace(0, 3.5, 1000);
    sim = @(x,t)sim3d(t, x);
    lsode_options('integration method', 'stiff');
    xs = lsode(sim, xinit, ts);
end

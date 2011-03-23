function [xs, ts] = runsim()
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
    tspan = [0 2.0];
    sim = @sim3d;
    [ts xs] = ode15s(sim, tspan, xinit);
end

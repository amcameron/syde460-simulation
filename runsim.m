function [xs, ts] = runsim()
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
    %ts = linspace(0, 2, 1000);
    tspan = [0 2.0];
    sim = @sim3d;
    %lsode_options('integration method', 'stiff');
    [ts xs] = ode15s(sim, tspan, xinit);
end

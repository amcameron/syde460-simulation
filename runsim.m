function [xs, ts] = runsim()
    %clear all          % uncomment this line if you're messing with simplant
    % initial position
    % initial velocity
    % inital angles
    % inital angle rates
    % initial output error
    xinit = [  2000    2000 2000 ...
          -10.8  0   0   ...
          0      pi/8   0   ...
          0      0   0   ...
          0      0       ...
    ];
    tspan = [0 15.0];
    sim = @sim3d;
    [ts xs] = ode15s(sim, tspan, xinit);
end

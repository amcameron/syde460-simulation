%
% Variable Conventions
% x = { xdot, ydot, zdot, φdot, θdot, ψdot }
% X = { X, Y, Z, Xdot, Ydot, Zdot, Φ, Θ, Ψ, Φdot, Θdot, Ψdot }
% P = { (δZ/δX)*, Zdot* }
% p = { u*, v*, w* } (v* always zero ATM)
% d = { δ, ξ }
%

% Xdot = f(X) - calculate the derivatives of the state vector given the current
% state vector. suitable for running through an ode solver.
function Xdot = sim3d(t, X)
    P    = planpath(X)
    x    = localize_state(X);
    z    = X(13:14);
    d  = controller(x, z);
    % d = max(min(d, pi/2), -pi/2)
    % Xdot = plant(X, [-3*pi/16 0]);
    Xdot = plant(X, d);
    Xdot(13:14) = P;
    Xdot = Xdot';
end


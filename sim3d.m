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
% outline:
% -planpath calculates the desired pitch & roll, P, based on state.
%   (the target is always the origin of the coordinate system.)
%   **THIS** is currently where most of the "magic" happens.
% -the global state, X, is localized to the glider's frame (x)
% -the two "extra" values in X are the integrated output error, z.
%   (z is difference in pitch, and doesn't need localization.)
% -the controller calculates the control input (d) based on the local state and z.
% -the state derivative is calculated by the aerodynamic simulation, plant()
% -the derivative of the integrated output error is the current output error.
function Xdot = sim3d(t, X)
    % P    = planpath(X);
    P = [0 0]';
    x    = [localize_state(X) X(13:23)'];
    z    = X(13:14);
    d  = controller(x, z);
    % Xdot = plant(X, [-3*pi/16 0]);
    Xdot = plant(X, d);
    Xdot(15:23) = observer(x, d);
    Xdot(13:14) = P;
    Xdot = Xdot';
end


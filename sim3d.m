%
% Variable Conventions
% Φ - roll, θ - pitch, ψ - yaw
% x = { xdot, ydot, zdot, φdot, θdot, ψdot }
% X = { X, Y, Z, Xdot, Ydot, Zdot, Φ, Θ, Ψ, Φdot, Θdot, Ψdot }
% P = { (δZ/δX)*, Zdot* }
% p = { u*, v*, w* } (v* always zero ATM)
% d = { δ, ξ }
%

% Xdot = f(X) - calculate the derivatives of the state vector given the current
% state vector. suitable for running through an ode solver.
function Xdot = sim3d(t, X)
    % P    = planpath(X);
    % x    = localize(X);
    % p    = localizeplan(X, P);
    % d    = controller(x, p);
    Xdot = plant(X, [-3*pi/16 0]);
    % Xdot = plant(X, d);
    Xdot = Xdot';
end

% P = planpath(x) - plan a global frame path to the origin given the current
% state vector.
function P = planpath(X)
    % naive linear path planning
    % (δZ/δX)* = Z/X
    P(1) = X(3)/X(1);

    % logarithmic descent rate (arbitrary coefficients for the moment)
    % Zdot* =~ log(Z + 1)
    % Zdot* = 0 when Z = 0
    a = 1;
    b = 1000;
    P(2) = a*log(X(3)/b + 1);
end

% p = localizeplan(X, P) - create a local frame version of a global frame plan
function p = localizeplan(X, P)
    Zdotstar = P(2);
    Xdotstar = P(2)/P(1); % Xdot = Zdot/(δZ/δX) [chain rule]
    p = R(X(7), X(8), X(9)) * [ Xdotstar; 0; Zdotstar ];
end

function x = localize(X)
    x(1:3) = R(X(7), X(8), X(9)) * X(4:6);
    x(4:6) = R(X(7), X(8), X(9)) * X(10:12);
end

function d = controller(x, p)
    theta = acos(dot([p(1) p(3)], [x(1) x(3)])/(norm([p(1) p(3)])*norm([x(1) x(3)])));
    x2d = [ x(1) x(3) theta x(5) ];
    p2d = [ p(1) p(3) ];
    Nx_long = [0.9578 0.9578; 0.9367 0.9367; 0.3144 0.3144; 0.0508 0.0508];
    Nu_long = [0.9578 0.9578];
    K_long = [-3.1178 -1.0271 3.3729 36.1946];
    d = [0 0]';
    d(1) = (Nu_long + K_long*Nx_long)*p2d' - K_long*x2d';
end

% create a forward rotation matrix (global -> local)
% uses 3-2-1 euler angles
function A = R(phi, theta, psi)
    A = [ 1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi) ] * ...
    [ cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta) ] * ...
    [ cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1 ];
end

% Xdot = plant(X, d) - model the plant
% given the state vector and the control input, determine the rate of change of
% the state vector. operates entirely in global frame.
function Xdot = plant(X, d)
    longitudinal = [ -0.1730  0.6538  0.1388 -9.7222; ...
                     -1.4208 -2.2535 10.7370  1.3093; ...
                      0.2685 -0.4402 -1.4113  0; ...
                      0       0       1       0 ];
    lateral = [ -0.2195 -0.1580  -10.798  9.722 -1.3098; ...
                -1.4670 -21.318    7.5163 0      0; ...
                 0.2906   3.7362  -2.1119 0      0; ...
                 0        1        0      0      0; ...
                 0        0        1      0      0 ];

    rot = R(X(7), X(8), X(9));

    x(1:3)   = rot*X(1:3);
    x(4:6)   = rot*X(4:6);
    x(7:9)   = 0;
    x(10:12) = rot*X(10:12);

    longdot = longitudinal * [ (x(4) - 10.8) x(6)  x(11) x(8) ]';
    latdot  = lateral      * [ x(5)          x(10) x(12) x(7) x(9) ]';

    xdot(4)   = longdot(1);
    xdot(5)   = latdot(1);
    xdot(6)   = longdot(2);
    xdot(7)   = latdot(4);
    xdot(8)   = longdot(4);
    xdot(9)   = latdot(5);
    xdot(10)  = latdot(2);
    xdot(11)  = longdot(3);
    xdot(12)  = latdot(3);

    Xdot(1:3)   = X(4:6);
    Xdot(7:9)   = rot\xdot(7:9)';
    Xdot(4:6)   = rot\xdot(4:6)';
    Xdot(10:12) = rot\xdot(10:12)';
    X
    xdot
    Xdot'
end

% coefficient of lift (only considers incidence ATM)
function coef = Cl(alpha, v)
    peak  = 35 * (pi/180); % approximate Cl dropoff by plateau @ 35deg
    angle = min(alpha, peak);

    % linear approximation of lift curve (sampled @ 7deg and 30deg)
    m =  2.4911;
    b = -0.30435;

    coef = m * alpha + b;
end

% coefficient of drag (only considers incidence ATM)
function coef = Cd(alpha, v)
    % piecewise-quadratic approximation of drag curve
    % (sampled at 0deg, 7deg, 15deg, slope @ 7deg fixed at 0)
    a = 0.24555;
    b = -0.060;
    c = 0.08;
    d = 0.14324;
    e = -0.035;
    f = 0.069346;
    if (alpha <= 7*(pi/180))
        coef = a*alpha^2 + b*alpha + c;
    else
        coef = d*alpha^2 + e*alpha + f;
    end
end

% pitching coefficient (only considers incidence ATM)
function coef = Cm(alpha, v)
    % quadratic approximation of pitching curve
    % (sampled at 0deg, 15deg, and 30deg)
    a = 0.076394;
    b = -0.06;
    c = 0.04;
    coef = a*alpha^2 + b*alpha + c;
end

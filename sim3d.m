%
% Variable Conventions
% x = { u, v, w }
% X = { X, Y, Z, Xdot, Ydot, Zdot, Φ, Θ, Ψ, Φdot, Θdot, Ψdot }
% P = { (δZ/δX)*, Zdot* }
% p = { u*, v*, w* } (v* always zero ATM)
% d = { δ, ξ }
%

% Xdot = f(X) - calculate the derivatives of the state vector given the current
% state vector. suitable for running through an ode solver.
function Xdot = f(X)
    P    = planpath(X)
    x    = localize(X)
    p    = localizeplan(X, P)
    d    = controller(x, p)
    Xdot = plant(X, d)
endfunction

% P = planpath(x) - plan a global frame path to the origin given the current
% state vector.
function P = planpath(X)
    % naive linear path planning
    % (δZ/δX)* = Z/X
    P(1) = X(3)/X(1)

    % logarithmic descent rate (arbitrary coefficients for the moment)
    % Zdot* =~ log(Z + 1)
    % Zdot* = 0 when Z = 0
    a = 1
    b = 1000
    P(2) = a*log(X(3)/b + 1)
endfunction

% p = localizeplan(X, P) - create a local frame version of a global frame plan
function p = localizeplan(X, P)
    Zdotstar = P(2)
    Xdotstar = P(2)/P(1) % Xdot = Zdot/(δZ/δX) [chain rule]
    p = R(X(7), X(8), X(9)) * [ Xdotstar; 0; Zdotstar ]
endfunction

function x = localize(X)
    x(1:3) = R(X(7), X(8), X(9)) * X(4:6)'
endfunction

function d = controller(x, p)
    % change u*, w* into a proportion; try and control w so that w/u = w*/u*.
    % so, w = w* u / u*
    w_des = x(1)*p(2)/p(1);
endfunction

% create a forward rotation matrix (global -> local)
% uses 3-2-1 euler angles
function A = R(phi, theta, psi)
    A = [ 1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi) ] * \
	[ cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta) ] * \
	[ cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1 ]
endfunction

% Xdot = plant(X, d) - model the plant
% given the state vector and the control input, determine the rate of change of
% the state vector. operates entirely in global frame.
function Xdot = plant(X, d)
    % body-frame -> global-frame rotation matrix
    Rb = R(X(7), X(8), X(9))';

    % body frame direction vectors
    ubx = Rb * [ 1 0 0 ]';
    uby = Rb * [ 0 1 0 ]';
    ubz = Rb * [ 0 0 1 ]';

    % aerodynamic direction vectors (zero wind assumed ATM)
    uax = -(X(4:6)/norm(X(4:6))); % reverse and normalize wind direction
    uay = uay - (dot(uax,uby)/dot(uax, ubx))*uax; % get uay direction
    uay = uay/norm(uay); % normalize
    uaz = cross(uax, uay);

    % aerodynamic angles
    sideslip = atan2(dot(uax, uby), dot(uax, ubx));
    attack   = asin(dot(uax, ubz));

    true_airspeed = norm(X(7:9)); % wind assumed to be zero ATM

    % direction towards ground
    ug  = [ 0 0  -1 ];
    g   = 9.8; % [N/kg]

    % direction from pilot to wing
    uwpbz = 1/sqrt(1 + tan(d(1))^2 + tan(d(2))^2); % uwp dot ubz
    uwpbx = upwbz * tan(d(1));
    uwpby = upwbz * tan(d(2));
    upw   = Rb * -[ upwbx upwby upwbz ];

    % physical parameters
    S   = 16.26; % wing area [m^2]
    rho = 1.29;  % air density (estimated at sea level, 15C, 1atm on wikipedia) [kg/m^3]
    c   = 1.626; % reference chord [m]
    mp  = 80;    % pilot mass [kg]
    mw  = 31;    % wing mass [kg]
    lwp = 1.2;   % distance from wing to pilot [m]

    m   = mp + mw % total mass [kg]
    lcw = mp/m * lwp % distance between wing and CoM [m]

    % forces & moments
    L  = Cl(attack, true_airspeed) * 1/2 * rho * true_airspeed^2 * S
    D  = Cd(attack, true_airspeed) * 1/2 * rho * true_airspeed^2 * S
    Mp = Cm(attack, true_airspeed) * 1/2 * rho * true_airspeed^2 * S * c
    F  = (mp + mw)*g*ug \
         - L*uaz \
         - D*uax \
         + Mp*lcw*cross(uay, upw)
    M  = Mp*uay \
	- L*lcw*cross(upw, uaz) \
	- D*lcw*cross(upw, uax)

    % F = ma
    Xdot(4:6) = F/m;
    % M = Iα (along primary axes only)
    Xdot(10:12) = ???

    % position derivatives given by speeds
    Xdot(1:3) = X(4:6);
    Xdot(7:9) = X(10:12);
endfunction

% coefficient of lift (only considers incidence ATM)
function coef = Cl(alpha, v)
    peak  = 35 * (pi/180); % approximate Cl dropoff by plateau @ 35deg
    angle = min(alpha, peak);

    % linear approximation of lift curve (sampled @ 7deg and 30deg)
    m =  2.4911;
    b = -0.30435;

    coef = m * alpha + b
endfunction

% coefficient of drag (only considers incidence ATM)
function coef = Cd(alpha, v)
    % piecewise-quadratic approximation of drag curve
    % (sampled at 0deg, 7deg, 15deg, slope @ 7deg fixed at 0)
    a = 0.24555
    b = -0.060
    c = 0.08
    e = 0.14324
    f = -0.035
    g = 0.069346
    if (alpha <= 7*(pi/180))
	coef = a*alpha^2 + b*alpha + c
    else
	coef = d*alpha^2 + e*alpha + f
    endif
endfuction

% pitching coefficient (only considers incidence ATM)
function coef = Cm(alpha, v)
    % quadratic approximation of pitching curve
    % (sampled at 0deg, 15deg, and 30deg)
    a = 0.076394
    b = -0.06
    c = 0.04
    coef = a*alpha^2 + b*alpha + c
endfunction

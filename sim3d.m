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
    % P    = planpath(X);
    % x    = localize_state(X);
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

function x = localize_state(X)
    Rb       = R(X(7), X(8), X(9));
    x(1:3)   = Rb * X(1:3);
    x(4:6)   = Rb * X(4:6);
    x(7:9)   = Rb * X(7:9);
    x(10:12) = Rb * X(10:12);
end

function d = controller(x, p)
    x3d = [ x(4) x(6) x(11) x(8) x(5) x(10) x(12) x(7) x(9) ];
    p2d = [ p(1) p(3) p(2) ];

K = [-1370.30896664482 -64.4008272393267 12.5181483375295 ...
    1295.31022573046 901.881326313797 640.530961365344 ...
    5362.39727293853 -6950.09902648999 9992.89923422948; ...
    -1220.29705308623 -9.09118046452834 2.65157231328462 ...
    444.767267908868 2636.67324025295 1481.32340323496 ...
    12282.0293122896 -15522.1229620402 27609.7212284770];

Nx = [0.957785524498640 0.957785524498640; ...
    0.936698608390999 0.936698608390999; ...
    0.314414864133029,0.314414864133029; ...
    0.0508418054525925,0.0508418054525925; ...
    0.996598222387320,0.996598222387320; ...
    -0.0351095832383896,-0.0351095832383896; ...
    0.0357639152820641,0.0357639152820641; ...
    0.0616524123511705,0.0616524123511705; ...
    0,0];
Nu = [0.936698608390999 0.936698608390999];

    d = (Nu + K*Nx)*p3d' - K*x3d';
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
    % body-frame -> global-frame rotation matrix
    Rb = R(X(7), X(8), X(9))';

    % body frame direction vectors
    ubx = Rb * [ 1 0 0 ]';
    uby = Rb * [ 0 1 0 ]';
    ubz = Rb * [ 0 0 1 ]';

    % aerodynamic direction vectors (zero wind assumed ATM)
    uax = -(X(4:6)/norm(X(4:6))); % reverse and normalize wind direction
    uaz = cross(uax, uby);
    uaz = uaz/norm(uaz);          % normalize
    uay = cross(uaz, uax);

    % aerodynamic angles
    sideslip = atan2(dot(uax, uby), dot(uax, ubx));
    attack   = asin(dot(uax, ubz));

    true_airspeed = norm(X(4:6)); % wind assumed to be zero ATM

    % direction towards ground
    ug  = [ 0 0  -1 ];
    g   = 9.8; % [N/kg]

    % physical parameters
    S   = 16.26;         % wing area [m^2]
    rho = 1.29;          % air density (estimated at sea level, 15C, 1atm on wikipedia) [kg/m^3]
    c   = 1.626;         % reference chord [m]
    mp  = 80;            % pilot mass [kg]
    mw  = 31;            % wing mass [kg]
    lwp = 1.2;           % distance from wing to pilot [m]
    lhp = 0.246 - 0.215; % distance of hang point behind wing CoG [m]
    oswald_eff = 0.95;   % Oswald efficiency number of lift

    % direction from hang point to pilot
    uhpbx = sin(d(1));
    uhpby = sin(d(2))*cos(d(1));
    uhpbz = cos(d(2))*cos(d(1));
    uhp   = Rb * [ uhpbx uhpby uhpbz ]';

    % direction from wing cog to pilot
    wp  = lhp * Rb * [ -1 0 0 ]' + lwp * uhp;

    m   = mp + mw;    % total mass [kg]
    cw  = mp/m * -wp; % direction from CoM to wing [m]
    cp  = cw + wp;    % direction from CoM to pilot [m]

    % forces & moments
    L  = Cl(attack, true_airspeed) * 1/2 * rho * true_airspeed^2 * S;
    D  = (Cd_o(attack, true_airspeed) + Cl(attack, true_airspeed)^2/(pi*oswald_eff*S/c)) ...
	 * 1/2 * rho * true_airspeed^2 * S;
    Mp = Cm(attack, true_airspeed) * 1/2 * rho * true_airspeed^2 * S * c;
    F  = ((mp + mw)*g*ug)' ...
         + L*uaz ...
         + D*uax ...
         + cross(cw, Mp * uay);

    M  = Mp*uay ...
         + cross(cw,  L*uaz);
         + cross(cw, -D*uax);

    % position derivatives given by speeds
    Xdot(1:3) = X(4:6);
    Xdot(7:9) = X(10:12);

    % F = ma
    Xdot(4:6) = F/m;
    % M = Iα (along primary axes only)
    % XXX HORRIBLE ASSUMPTION 
    % model glider as a sphere (great flying spheres of mathland!)
    Xdot(10) = M(1)/(2/5*m*norm(cw)^2);
    Xdot(11) = M(2)/(2/5*m*norm(cw)^2);
    Xdot(12) = M(3)/(2/5*m*norm(cw)^2);
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
function coef = Cd_o(alpha, v)
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

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
    % first priority - align craft orientation with trajectory
    utraj = X(5:7)/norm(X(5:7));
    alignedness = dot(utraj, R(X(7), X(8), X(9)) * [1 0 0]');
    if alignedness < cos(pi/16)
	% alignment outside +-~10 degrees of trajectory
	% decide how to get to trajectory window, banking, pitching, or both
	d_align_by_d_pitch = dot(utraj, R(X(7) + pi/360, X(8), X(9)) * [1 0 0]') - alignedness;
	d_align_by_d_yaw   = dot(utraj, R(X(7), X(8), X(9) + pi/360) * [1 0 0]') - alignedness;
	% magnitude of angle change we need (approximated by 1-cos(x)=x for small x)
	magnitude  = 1 - alignedness;
	deriv_mag  = norm([d_align_by_d_pitch d_align_by_d_yaw]);
	pitch_star = magnitude * d_align_by_d_pitch/deriv_mag;
	yaw_star   = magnitude * d_align_by_d_yaw/deriv_mag;
    else
	% craft is in nominally aligned flight-envelope, attempt manoeuvres towards origin
	uorig = -X(1:3)/norm(X(1:3));
	alignedness = dot(uorig, R(X(7), X(8), X(9)) * [1 0 0]');
	% decide how to point towards origin, banking, pitching, or both
	d_align_by_d_pitch = dot(uorig, R(X(7) + pi/360, X(8), X(9)) * [1 0 0]') - alignedness;
	d_align_by_d_yaw   = dot(uorig, R(X(7), X(8), X(9) + pi/360) * [1 0 0]') - alignedness;
	% magnitude of angle change we need (approximated by 1-cos(x)=x for small x)
	magnitude  = 1 - alignedness;
	deriv_mag  = norm([d_align_by_d_pitch d_align_by_d_yaw]);
	pitch_star = magnitude * d_align_by_d_pitch/deriv_mag;
	yaw_star   = magnitude * d_align_by_d_yaw/deriv_mag;
    end

    % we have control of pitch and roll, but pitch and yaw are how trajectory works
    % roll by negative yaw target, yaw becomes pitch, and (hopefully) things work themselves out
    P = [pitch_star, -yaw_star];
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
    % Cut and paste state variables into the form the linearized model expects.
    % Ditto with the reference input.
    x3d = [ x(4) x(6) x(11) x(8) x(5) x(10) x(12) x(7) x(9) ];
    %TODO: change p input to be pitch & roll (Θ*, Φ*) instead of u*, v*, w*
    p3d = [ p(1) p(3) p(2) ];

    % Create the error signal of the now.
    e3d = [x(8) x(7)] - p3d;

    % Use a persistent accumulator for the integrated output error.
    % Persistent variables are initialized to [].
    persistent z;
    if (isempty(z))
        z = 0;
    end

    %TODO: should this be accounting for timestep? how?
    z = z + e3d;

    Kstate = [ ...
95108.9185474388   29118.8277378886 18.2742882107397 -324895.868845913 ...
-375505.168034732 -38615.7091973777 -323697.174252387 357293.580324904 ...
-4193737.92294423; ...
 127995.903128420  38003.3548986827  4.08649091592160 -425359.863964595 ...
-1248927.45087099 -121358.063111166 -1017512.60373025  1104840.16906034 ...
-13997231.2401959];

    Kref = [ ...
954207.967471406 3322468.63455386; ...
1243204.13304077 11228793.1228830];

    d = (-[Kstate Kref]*[x3d; z])';
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
    % ^-- Andrew endorses the above comment.
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

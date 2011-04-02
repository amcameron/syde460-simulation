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
         + Mp/norm(cross(cw, uay)) * cross(cw, uay)/norm(cross(cw, uay));

    M  = Mp*uay ...
         + cross(cw,  L*uaz) ...
         + cross(cw, -D*uax);

    % d/dt(r) = v
    Xdot(1:3) = X(4:6);

    % euler rate conversion
    Xdot(7:9) = inv( [ 0 -sin(X(7)) cos(X(8))*cos(X(7)); ...
		       0  cos(X(7)) cos(X(8))*sin(X(7)); ...
		       1  0         -sin(X(7)) ] )*X(10:12);

    % F = ma
    Xdot(4:6) = F/m;

    % α = I^-1(M - ω x Iω)
    Ixx = 242.17;
    Iyy = 111.81;
    Izz = 255.99;
    Ixz = -30.54;
    I = [  Ixx 0    -Ixz;
	   0   Iyy   0;
	  -Ixz 0     Izz ];
    Xdot(10:12) = inv(I)*(M - cross(X(10:12), Rb'*I*Rb'*X(10:12)));
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

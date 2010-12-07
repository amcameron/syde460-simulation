%
% Variable Conventions
% x = { u, w, θ, θdot }
% X = { X, Z, Xdot, Zdot, Θ, Θdot, θ, θdot }
% P = { (δZ/δX)*, Zdot* }
% p = { u*, w* }
% d = { δ, ξ }
%

function Xdot = f(t, X)
    P    = planpath(X);
    x    = localize(X);
    p    = localizeplan(X, P);
    d    = controller(x, p);
    xdot = plant(x, d);
    Xdot = globalize(xdot, X);
	Xdot = Xdot';
end

function P = planpath(X)
    % naive linear path planning
    % (δZ/δX)* = Z/X
    P(1) = X(2)/X(1);

    % logarithmic descent rate (arbitrary coefficients for the moment)
    % Zdot* =~ log(Z + 1)
    % Zdot* = 0 when Z = 0
    a = 1;
    b = 1000;
    P(2) = a*log(X(2)/b + 1);
end

function p = localizeplan(X, P)
    Zdotstar = P(2);
    Xdotstar = P(2)/P(1); % Xdot = Zdot/(δZ/δX) [chain rule]
    p = R(X) * [ Xdotstar; Zdotstar ];
end

function x = localize(X)
    x(1:2) = R(X) * X(3:4);
    x(3:4) = X(7:8);
end

function A = R(X)
    A = [ cos(X(5)) -sin(X(5)); sin(X(5)) cos(X(5)) ];
end

function d = controller(x, p)
	Nx_long = [0.9578 0.9578; 0.9367 0.9367; 0.3144 0.3144; 0.0508 0.0508];
	Nu_long = [0.9578 0.9578];
	K_long = [-3.1178 -1.0271 3.3729 36.1946];
	d = [0 0]';
	d(1) = (Nu_long + K_long*Nx_long)*p - K_long*x';
end

function xdot = plant(x, d)
    A_long = [-0.1730  0.6538  0.1388 -9.7222;
              -1.4208 -2.2535 10.7370  1.3093;
               0.2685 -0.4402 -1.4113  0;
               0       0       1       0];
    B_long = [0; 0; 7.46; 0];
    xdot = A_long * x' + B_long * d(1);
end

function Xdot = globalize(xdot, X)
    Xdot(1:2) = X(3:4);
    Xdot(3:4) = R(X)' * xdot(1:2);

    % filter θ into Θ
    % Θdot = Ki * Δθ [axis angle integral controller]
    % θdot = ~(θdot) - Θdot [don't double count angular velocity]
    Ki        = 0.04; % axis angle controller integral gain coefficient
    Xdot(5)   = Ki * (X(7) - X(5));
    Xdot(7)   = xdot(3) - Xdot(5);

    % Xdot(6)  = Θdotdot???
    Xdot(8)   = xdot(4);
end

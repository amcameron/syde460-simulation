%
% Variable Conventions
% x = { u, w, θ, θdot }
% X = { X, Z, Xdot, Zdot, Θ, Θdot, θ, θdot }
% P = { (δZ/δX)*, Zdot* }
% p = { u*, w* }
% d = { δ, ξ }
%

function Xdot = f(X)
    P    = planpath(X)
    x    = localize(X)
    p    = localizeplan(X, P)
    d    = controller(x, p)
    xdot = plant(x, d)
    Xdot = globalize(xdot, X)
endfunction

function P = planpath(X)
    % naive linear path planning
    % (δZ/δX)* = Z/X
    P(1) = X(2)/X(1)

    % logarithmic descent rate
    % Zdot* =~ log(Z + 1)
    % Zdot* = 0 when Z = 0
    a = 1
    b = 1000
    P(2) = a*log(X(2)/b + 1)
endfunction

function p = localizeplan(X, P)
    Zdotstar = P(2)
    Xdotstar = P(2)/P(1) % Xdot = Zdot/(δZ/δX) [chain rule]
    p = R(X) * [ Xdotstar; Zdotstar ]
endfunction

function x = localize(X)
    x(1:2) = R(X) * X(3:4)'
    x(3:4) = X(7:8)
endfunction

function A = R(X)
    A = [ cos(X(5)) -sin(X(5)); sin(X(5)) cos(X(5)) ]
endfunction

function d = controller(x, p)
endfunction

function xdot = plant(x, d)
endfunction

function Xdot = globalize(xdot, X)
    Xdot(1:2) = X(3:4)
    Xdot(3:4) = R(X)' * xdot(1:2)'
    Xdot(5)
endfunction

function xhatdot = observer(x, u)
    % Cut and paste state variables into the form the linearized model expects.
    % Ditto with the reference input.
    xhat = x(15:23)';
    x3d = [ x(4) x(6) x(11) x(22) x(5) x(10) x(12) x(23) x(21) ]';

    persistent A B C H Kstate Kref;
    if (isempty(A))
        [A B C H Kstate Kref] = simplant;
    end

    y = C*x3d;

    xhatdot = A*xhat + B*u' + H*(y - C*xhat);
end


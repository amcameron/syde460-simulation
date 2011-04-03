function d = controller(x, z)
    % Cut and paste state variables into the form the linearized model expects.
    % Ditto with the reference input.
    % Adjust the pitch by the steady-state value to get the disturbance pitch.
    % Steady-state values for roll & yaw are zero.
    % TODO: put disturbance pitch adjustment in localize_state
    % x3d = [ x(4) x(6) x(11) (x(8)+0.2621) x(5) x(12) x(10) x(9) x(7) ]';
    x3d = [ x(4) x(6) x(11) x(8) x(5) x(12) x(10) x(9) x(7) ]';

    persistent A B C H Kstate Kref;
    if (isempty(A))
        [A B C H Kstate Kref] = simplant;
    end

    d = (-[Kstate Kref]*[x3d; z])';

    % Add steady-state trim to the controller's differential output.
    % d     = d + [-0.83301 0];
    d     = [d(1)-0.83301 -d(2)];

    % limit to physically achievable values
    d = max(min(d, pi/4), -pi/4);
end


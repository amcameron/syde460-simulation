function vissim(xs, ts);
    figure
    % positions row

    subplot(3, 3, 1);
    plot(ts, xs(:,1:2));
    title('position');
    legend('x', 'y');

    subplot(3, 3, 2);
    % plot(xs(:,1), xs(:,2));
    % title('birds eye view');
    plot3(xs(:, 1), xs(:,2), xs(:,3));
    axis equal
    title('trajectory');

    subplot(3, 3, 3);
    plot(ts, xs(:, 3));
    title('height');

    % velocities row
    subplot(3, 3, 4);
    plot(ts, xs(:,4:5));
    title('velocities');
    legend('x', 'y');

    subplot(3, 3, 5);
    plot(ts, xs(:,6));
    title('vertical velocity');

    subplot(3, 3, 6);
    plot(ts, (xs(:,4).^2 + xs(:,5).^2 + xs(:,6).^2).^(1/2));
    title('airspeed');

    % angles row
    subplot(3, 3, 7);
    plot(ts, mod(xs(:,7:9)+pi, 2*pi)-pi);
    title('angles');
    legend('\phi', '\theta', '\psi');

    subplot(3, 3, 8);
    plot(ts, xs(:,10:12));
    title('angle rates');
    legend('\phi', '\theta', '\psi');

    % XXX does not count angular velocity energy
    subplot(3, 3, 9);
    mass = 111;
    inertia_matrix = eye(3)*33;
    plot(ts, mass*xs(:,3)*9.8 + ...
         1/2*mass*power((xs(:,4).^2 + xs(:,5).^2 + xs(:,6).^2).^(1/2), 2) + ...
         1/2*dot(xs(:,10:12)', j*xs(:,10:12)', 1)');
    title('energy');

    % plot the path planner and controller outputs
    figure
    [ps ds] = intermediaries(xs, ts);
    subplot(2, 1, 1);
    plot(ts, 180/pi*ps);
    title('path plan');
    legend('pitch', 'roll');

    subplot(2, 1, 2);
    plot(ts, 180/pi*ds);
    title('controller output');
    legend('pitch', 'roll');
end

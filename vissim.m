function vissim(xs, ts);
    % positions row

    subplot(3, 3, 1);
    plot(ts, xs(:,1:2));
    title('position');
    legend('x', 'y');

    subplot(3, 3, 2);
    plot(xs(:,1), xs(:,2));
    title('birds eye view');
    xlabel('x');
    ylabel('y');

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
    plot(ts, norm(xs(:, 4:6), 'rows'));
    title('airspeed');

    % angles row
    subplot(3, 3, 7);
    plot(ts, xs(:,7:9));
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
    plot(ts, xs(:,3)*9.8 + ...
             1/2*mass*power(norm(xs(:,4:6), 'rows'), 2));
    title('energy');
end

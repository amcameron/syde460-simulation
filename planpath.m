% use gradient method to accomplish a number of prioritized objectives
function P = planpath(X)
    % gradient method constants:
    delt = pi/180; % step size for derivative estimation
    step = pi/16;  % step size for gradient method

    % important vectors trajectory and orientation
    utraj             = X(4:6)/norm(X(4:6));
    udir              = R(X(7), X(8), X(9)) * [1 0 0]';
    udir_plus_d_yaw   = R(X(7) + delt, X(8), X(9)) * [1 0 0]';
    udir_plus_d_pitch = R(X(7), X(8) + delt, X(9)) * [1 0 0]';

    % first priority - stay level
    verticality = dot(udir, [0 0 1]);
    if verticality > sin(pi/16)
	d_by_d_yaw   = (1/delt)*(dot(udir_plus_d_yaw,   [0 0 1]) - verticality);
	d_by_d_pitch = (1/delt)*(dot(udir_plus_d_pitch, [0 0 1]) - verticality);
	yaw_star     = -step*d_by_d_yaw;
	pitch_star   = -step*d_by_d_pitch;
    else
	% second priority - align craft orientation with trajectory
	alignedness = dot(utraj, udir);
	if alignedness < cos(pi/16)
	    d_by_d_yaw   = (1/delt)*(dot(utraj, udir_plus_d_yaw)   - alignedness);
	    d_by_d_pitch = (1/delt)*(dot(utraj, udir_plus_d_pitch) - alignedness);
	    yaw_star   = step*d_by_d_yaw;
	    pitch_star = step*d_by_d_pitch;
	else
	    % third priority - steer (trajectory) towards target (origin)
	    uorig        = -X(1:3)/norm(X(1:3));
	    alignedness  = dot(uorig, utraj);
	    d_by_d_yaw   = (1/delt)*(dot(uorig, R(X(7) + delt,X(8),X(9))*R(X(7),X(8),X(9))'*utraj) - alignedness);
	    d_by_d_pitch = (1/delt)*(dot(uorig, R(X(7),X(8) + delt,X(9))*R(X(7),X(8),X(9))'*utraj) - alignedness);
	    yaw_star   = step*d_by_d_yaw;
	    pitch_star = step*d_by_d_pitch;
	end
    end

    % % we have control of pitch and roll, but pitch and yaw are how trajectory works
    % % roll by negative scaled yaw target, yaw becomes pitch, and (hopefully) things work themselves out
    % P = [pitch_star, -yaw_star];

    % we have control of pitch and roll, but pitch and yaw are how trajectory works
    cur_roll = mod(X(9) + pi, 2*pi) - pi;
    P = [sqrt(pitch_star^2 + yaw_star^2), max(min(atan2(-yaw_star, pitch_star), -pi/8 - cur_roll), pi/8 - cur_roll)];
end


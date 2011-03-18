function P = planpath(X)
    % first priority - align craft orientation with trajectory
    utraj = X(4:6)/norm(X(4:6));
    alignedness = dot(utraj, R(X(7), X(8), X(9)) * [1 0 0]');
    if alignedness < cos(pi/16)
	% alignment outside +-~10 degrees of trajectory
	% decide how to get to trajectory window, banking, pitching, or both
	d_align_by_d_yaw   = dot(utraj, R(X(7) + pi/360, X(8), X(9)) * [1 0 0]') - alignedness;
	d_align_by_d_pitch = dot(utraj, R(X(7), X(8) + pi/360, X(9)) * [1 0 0]') - alignedness;
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
	d_align_by_d_yaw   = dot(uorig, R(X(7) + pi/360, X(8), X(9)) * [1 0 0]') - alignedness;
	d_align_by_d_pitch = dot(uorig, R(X(7), X(8) + pi/360, X(9)) * [1 0 0]') - alignedness;
	% magnitude of angle change we need (approximated by 1-cos(x)=x for small x)
	magnitude  = 1 - alignedness;
	deriv_mag  = norm([d_align_by_d_pitch d_align_by_d_yaw]);
	pitch_star = magnitude * d_align_by_d_pitch/deriv_mag;
	yaw_star   = magnitude * d_align_by_d_yaw/deriv_mag;
    end

    % we have control of pitch and roll, but pitch and yaw are how trajectory works
    % roll by negative yaw target, yaw becomes pitch, and (hopefully) things work themselves out
    P = [-X(8), -yaw_star];
end


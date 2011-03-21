% create a forward rotation matrix (global -> local)
% uses 3-2-1 euler angles (yaw-pitch-roll)
function A = R(phi, theta, psi)
    A = [ 1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi) ] * ...
    [ cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta) ] * ...
    [ cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1 ];
end


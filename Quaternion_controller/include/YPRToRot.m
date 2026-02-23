function [R] = YPRToRot(psi, theta, phi)
% The function compute the rotation matrix using the YPR (yaw-pitch-roll)
% convention, given psi, theta, phi.
% Input:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% Output:
% R rotation matrix

R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
    sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

if (theta == pi/2)
    disp("Gimbal lock configuration");
end

end
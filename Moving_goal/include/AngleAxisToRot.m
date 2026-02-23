function R = AngleAxisToRot(h,theta)
% The fuction implement the Rodrigues Formula
% Input: 
% h is the axis of rotation
% theta is the angle of rotation (rad)
% Output:
% R rotation matrix

h = h / norm(h); % Normalize the rotation axis
Hcross = [0 -h(3) h(2); h(3) 0 -h(1); -h(2) h(1) 0]; %cross-product operator
R = eye(3) + sin(theta) * Hcross + (1 - cos(theta)) * Hcross^2; % Rodrigues' rotation formula

end

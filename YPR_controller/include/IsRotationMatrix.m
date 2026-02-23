function [isRotationMatrix] = IsRotationMatrix(R)
% The function checks that the input R is a valid rotation matrix, that is 
% a valid element of SO(3).
% Return true if R is a valid rotation matrix, false otherwise. In the
% latter case, print a warning pointing out the failed check.
isRotationMatrix = all(size(R) == [3, 3]) && norm(R.' * R - eye(3)) < 1e-3 && abs(det(R) - 1) < 1e-3;
if ~isRotationMatrix
    warning('Input matrix R is not a valid rotation matrix.');

end
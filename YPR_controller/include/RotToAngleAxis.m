function [h,theta] = RotToAngleAxis(R)
% Given a rotation matrix this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% Check that R is a valid rotation matrix using IsRotationMatrix()
    if(IsRotationMatrix(R))
        theta = acos((trace(R)-1)/2);

        if abs(theta) < 1e-3
            h = [1; 0; 0]; % Default axis if angle is near zero
            disp('h is arbitrary');
    
        elseif abs(theta - pi) < 1e-3
            h(1,1) = sqrt((R(1,1)+1)/2);
            h(2,1) = mySign(R(2,1)) * mySign(R(1,1)) * sqrt((R(2,2)+1)/2);
            h(3,1) = mySign(R(3,1)) * mySign(R(1,1)) * sqrt((R(3,3)+1)/2);
    
        else
            h = vex((R - R') / 2) / sin(theta);
        end

        h = h / norm(h); % Normalize the axis vector
    else
        error('Input matrix R is not a valid rotation matrix.');
    end
    
end

 
function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
a = [S_a(3,2); S_a(1,3); S_a(2,1)];
end

function y = mySign(x)
    if (x >= 0)
        y = 1;
    else
        y = -1;
    end
end
function [h,theta] = RotToAngleAxis(R)
% Given a rotation matrix this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% Check that R is a valid rotation matrix using IsRotationMatrix()

theta = acos((trace(R)-1)/2);
% theta belongs to [0;pi]
% checking its value with a tolerance of 10^-3
tol = 10^-3;

if (0 < theta) && (theta < tol)
    % theta = 0 -> arbitrary unitary h
    h = [1;0;0];

elseif (theta > (pi-tol)) && (theta < (pi))
   % theta = pi
    h = [sqrt((R(1,1)+1)/2);
    sqrt((R(2,2)+1)/2);
    sqrt((R(3,3)+1)/2);];
    else
        h = (vex((R-transpose(R))/2))/sin(theta);
    end
end
 
function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
    a = [S_a(3,2);
        -S_a(3,1);
         S_a(2,1);];
end
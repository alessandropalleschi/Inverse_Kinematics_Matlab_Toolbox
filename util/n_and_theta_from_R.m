function [n, th] = n_and_theta_from_R(R)
%{
===========================================================================
	This function takes a rotation matrix and extract axis-angle 
    rotation parameters


---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	R       [3 x 3]         	rotation matrix

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	n       [3 x 1]          	rotation axis unit vector

	th      [1 x 1]          	rotation angle
 
===========================================================================
%}

    th = acos((trace(R) - 1) / 2);
    
    if(R == eye(3))     % no rotation
        n = [1; 0; 0];  % not relevant
    else
        n = 1 / (2*sin(th)) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
    end

end
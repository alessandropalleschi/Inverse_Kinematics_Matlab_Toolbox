function eo = eo_using_quat(R_des, R_cur)
%{
===========================================================================
	This function computes the orientation error, for a robot task 
    execution, using quaternions

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

    R_des       	[3 x 3]         	matrix that gives the desired
                                        orientation

    R_cur        	[3 x 3]         	matrix that gives the current
                                        orientation
    
---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	eo              [3 x 1]             column with the orientation error

===========================================================================
%}
    
%     Q_des = (rotm2qu(transpose(R_des))).';     % column
%     Q_cur = (dcm2quat(transpose(R_cur))).';   	% column
%     
%     if(dot(Q_des,Q_cur)<0)
%         Q_des = -Q_des;
%     end
%     % Note: transpose due to opposite dcm definition between 
%     % robotics and Matlab
    QuatErr = rotm2quat(R_des*R_cur.');
    eo = QuatErr(2:4)';
 
end

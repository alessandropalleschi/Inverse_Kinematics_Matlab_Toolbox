function [points] = generate_slerp(q_in, q_fin, N)   
%{
===========================================================================
	This function returns N points of R3, linearly placed between A and B

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	q_in              [4 x 1]  	Initial Quaternion

    q_fin             [4 x 1]   Final quaternion

    N                 [1 x 1] 	number of points

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	points          [Nx4]     matrix with N points

===========================================================================
%}
    t = linspace(0,1,N);
points = NaN(N,4);
    
    for k = 1:N
        points(k,:) = quatnormalize(quatinterp(q_in,q_fin,t(k),'slerp'));
    end

    
end

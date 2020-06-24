function v = skew_2_vect(S)
%{
===========================================================================
	This function takes a skew-simmetric matrix and returns the 
    corresponding vector

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	S       [3 x 3]         	skew-simmetric matrix

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------
    
	v       [3 x 1]          	corresponding vector
 
===========================================================================
%}

	v = [S(3,2); S(1,3); S(2,1)];
 
end
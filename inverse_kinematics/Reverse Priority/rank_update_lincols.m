function T = rank_update_lincols(J, Jra_pinv)
%{
===========================================================================
	This function executes the rank-update procedure

---------------------------------------------------------------------------
INPUT
---------------------------------------------------------------------------

	J                                   matrix to be updated, which gives
                                        the output dimension

    Jra_pinv                            matrix from from which the columns 
                                        are extracted, giving the search 
                                        dimension

---------------------------------------------------------------------------
OUTPUT
---------------------------------------------------------------------------

	T                                   updated matrix

===========================================================================
%}

J_dim = rank(J);
Pinv_dim = size(Jra_pinv, 2);

% first column
T = Jra_pinv(:,1);      % always l.i.
i = 2;               	% i.e.: i = i+1;
j = 2;                	% i.e.: j = j+1;

% other columns
while i <= J_dim
    while j <= Pinv_dim
        tj = Jra_pinv(:,j);     % possible new l.i. column
        j = j+1;
        if rank([T, tj]) == i	% lin. ind.
            T = [T, tj];        % append column 
            i = i+1;
            break
        end
    end
    if j > Pinv_dim && i <= J_dim
        disp('It is a trap! Using lincols!');
        [T, ~] = lincols(Jra_pinv, 1e-6);
%         disp('T = '); disp(T);
%         disp('rank T = '); disp(rank(T));        
        break;
    end
end

if size(T, 2) < J_dim
     disp('WARNING: There are not enough lin. ind. columns in T!');
end
    
end   
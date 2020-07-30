function At = clean_jac(A, Ara)

% CLEAN_JAC chooses the columns of Ara that are not lin. dep. on A and At
% will have full column rank

%   Returns:
%   - At -> the mat with columns of Ara that are not lin. dep. on A

% Get sizes and initialize At
[m,n] = size(Ara);
At = [];

% Choosing lin indep columns in loop
for j = 1:n
    
    col = Ara(:,j);
    
    % if the current column lin indep to A add it!
    if rank([A col]) > rank(A)
        At = [At col];
        A = [A col];
    end
    
end

end


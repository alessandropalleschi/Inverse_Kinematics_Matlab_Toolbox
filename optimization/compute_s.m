function s = compute_s(T)
Diff = T(1:3,2:end)-T(1:3,1:end-1);
norm_diff = vecnorm(Diff);
norm_tot = sum(norm_diff);   
s(1) = 0;
for i=1:size(norm_diff,2)
    s(i+1) = sum(norm_diff(1:i))/norm_tot;
end

% s = s/norm_tot
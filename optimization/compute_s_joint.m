function s = compute_s_joint(q)

    Diff = q(:,2:end)-q(:,1:end-1);

norm_diff = vecnorm(Diff);
norm_tot = sum(norm_diff);
s = zeros(1,size(norm_diff,2)+1);
s(1) = 0;
for i=1:size(norm_diff,2)
    s(i+1) = sum(norm_diff(1:i))/norm_tot;
end
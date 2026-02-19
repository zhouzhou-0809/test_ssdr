function [P_reg, info] = ssdr_regulate(P, ssdr)
%SSDR_REGULATE PSD enforcement + condition cap (placeholder for thesis SSDR).
% Replace with your SSDR operator for final results.

P = 0.5*(P+P');
[V,D] = eig(P);
d = real(diag(D));

d(d < ssdr.smin_floor) = ssdr.smin_floor;

dmax = max(d);
dmin_allowed = max(ssdr.smin_floor, dmax / ssdr.kappa_max);
d(d < dmin_allowed) = dmin_allowed;

P_reg = V*diag(d)*V';
P_reg = 0.5*(P_reg+P_reg');

info = struct();
info.smin = min(d);
info.kappa = max(d)/max(min(d), eps);
info.r_eff = sum(d > ssdr.rank_tau * max(d));
end

function info = cov_metrics(P, rank_tau)
%COV_METRICS Compute covariance spectral metrics.
P = 0.5*(P+P');
[V,D] = eig(P);
d = real(diag(D));
dmax = max(d);
dmin = min(d);
info = struct();
info.smin = dmin;
info.kappa = dmax / max(dmin, eps);
info.r_eff = sum(d > rank_tau * dmax);
end

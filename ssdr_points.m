function [X,w,r_eff,info] = ssdr_points(x,P,ssdr,j0,jm)
%SSDR_POINTS Truncated-spectrum cubature points for CKF (SSDR dimension control).
% Eigen-decompose P (small substate only), choose effective dimension r_eff by spectrum,
% generate 2*r_eff cubature points along retained principal directions.
%
% This implements the core SSDR idea: control effective dimension in spectral domain.

if nargin < 4, j0 = 1e-12; end %#ok<NASGU>
if nargin < 5, jm = 1e-6; end %#ok<NASGU>

n = numel(x);
P = 0.5*(P+P');

[V,D] = eig(P);
d = real(diag(D));

% floor
d(d < ssdr.smin_floor) = ssdr.smin_floor;

% condition cap
dmax = max(d);
dmin_allowed = max(ssdr.smin_floor, dmax / ssdr.kappa_max);
d(d < dmin_allowed) = dmin_allowed;

% effective rank
r_eff = sum(d > ssdr.rank_tau * max(d));
r_eff = max(1, min(r_eff, n));

Vr = V(:,1:r_eff);
dr = d(1:r_eff);

S = Vr * diag(sqrt(dr));      % n×r_eff
Xi = sqrt(r_eff) * S;

X = zeros(n, 2*r_eff);
for i=1:r_eff
    X(:,i) = x + Xi(:,i);
    X(:,i+r_eff) = x - Xi(:,i);
end
w = (1/(2*r_eff)) * ones(1,2*r_eff);

info = struct();
info.smin = min(d);
info.kappa = max(d)/max(min(d), eps);
info.r_eff = r_eff;
end

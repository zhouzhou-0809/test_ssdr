function [assoc, stats] = associate_nn_md(x,P,z_rb,nL,R,opts)
%ASSOCIATE_NN_MD Nearest-neighbor association with Mahalanobis gating.
% Uses EKF-linearized innovation covariance for speed and robustness.

M = size(z_rb,2);
assoc = repmat(struct('type',"drop",'j',0,'nis',NaN), 1, M);

stats = struct();
stats.n_inlier = 0;

if nL==0 || M==0
    for i=1:M, assoc(i).type="new"; end
    return;
end

pose = x(1:3);
Pxx = P(1:3,1:3);

for i=1:M
    zi = z_rb(:,i);
    best_nis = Inf;
    best_j = 0;

    cand = 1:nL;
    if opts.da.max_cand < nL
        cand = cand(1:opts.da.max_cand);
    end

    for j=cand
        lm_idx = 3 + (2*j-1);
        lm = x(lm_idx:lm_idx+1);

        % analytic zhat and Jacobians for PHD bearing:
        [zhat,Hx,Hl] = rb_jac_phd(pose,lm);

        nu = zi - zhat;
        nu(2) = helpers.wrapToPi(nu(2));

        Pll = P(lm_idx:lm_idx+1, lm_idx:lm_idx+1);
        Pxl = P(1:3, lm_idx:lm_idx+1);

        H = [Hx, Hl];
        Psub = [Pxx, Pxl; Pxl', Pll];
        S = H*Psub*H' + R;

% 1) 强制对称（避免数值非对称导致的非SPD）
S = 0.5*(S + S');

% 2) SPD 检查 + jitter（与 SSDR “谱域数值稳定化”一致）
j0 = opts.num.jitter0;
jm = opts.num.jitter_max;

ok = false;
for jt = 1:20
    [U,p] = chol(S + j0*eye(2),'lower');
    if p==0
        ok = true;
        break;
    end
    j0 = min(j0*10, jm);
end

if ~ok
    nis = Inf;   % 关键：不允许用负哨兵值，异常直接拒绝
else
    y = U \ nu;
    nis = y' * y;          % 等价于 nu' * inv(S) * nu，且严格>=0
end

if nis < best_nis
    best_nis = nis;
    best_j = j;
end
    end
    % ---- after the "for j=cand" loop ends ----
    if best_nis <= opts.da.chi2_gate
        assoc(i).type = "assoc";
        assoc(i).j    = best_j;
        assoc(i).nis  = best_nis;
        stats.n_inlier = stats.n_inlier + 1;
    else
        assoc(i).type = "new";
        assoc(i).nis  = best_nis;  % 可选：记录也行（通常是 Inf 或较大值）
    end

end
end

function [zhat,Hx,Hl] = rb_jac_phd(pose,lm)
x = pose(1); y = pose(2); th = pose(3);
lx = lm(1); ly = lm(2);
dx = lx - x; dy = ly - y;
r = hypot(dx,dy);
b = atan2(dy,dx) - th + pi/2;
b = helpers.wrapToPi(b);
zhat = [r;b];

% Jacobians (same as standard, bearing has same derivatives; +pi/2 is constant)
Hx = [ -dx/r, -dy/r, 0;
       dy/(r^2), -dx/(r^2), -1 ];
Hl = [ dx/r, dy/r;
      -dy/(r^2), dx/(r^2) ];
end

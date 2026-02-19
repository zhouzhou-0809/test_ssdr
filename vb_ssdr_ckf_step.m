function [state, map, vb, out] = vb_ssdr_ckf_step(state, map, vb, lambda_prev, ctrl, z_rb, q3, opts)
%VB_SSDR_CKF_STEP One laser-frame VB-SSDR-CKF-SLAM step.
%
% Inputs:
%  state: struct with x,P
%  map:   struct with nL, ids
%  vb:    IW params (u,U,Rhat)
%  lambda_prev: smoothed lambda from previous steps
%  ctrl:  control profile between laser frames (dt profile + avg v/delta)
%  z_rb:  2×M measurements [range; bearing] (bearing matches PHD: +pi/2 convention)
%  q3:    1×M extra attribute (logged only for now)
%  opts:  config
%
% Outputs:
%  state: updated x,P
%  map: updated nL
%  vb: updated IW params
%  out: metrics for evaluation/plots

out = struct();
out.n_meas_raw = size(z_rb,2);
out.n_meas_used = out.n_meas_raw;
out.n_inlier = 0;
out.n_new = 0;
out.nL = map.nL;

use_vb = opts.vb.enable;
use_lambda = opts.lambda.enable;
use_ssdr = opts.ssdr.enable;

% ---------------- 1) Predict ----------------
x = state.x; P = state.P;

dx_pose = helpers.integrate_bicycle(ctrl, x(3), opts.vehicle.L);  % [dx;dy;dtheta]
x_pred = x;
x_pred(1:3) = x(1:3) + dx_pose;
x_pred(3) = helpers.wrapToPi(x_pred(3));

Q = opts.Q0;
if use_lambda
    lam = min(max(lambda_prev,1), opts.lambda.max);
    Q = lam * opts.Q0;
end
P_pred = P;
P_pred(1:3,1:3) = P(1:3,1:3) + Q;
P_pred = 0.5*(P_pred+P_pred');

% SSDR on full P is too expensive. Use pose-block metrics (3x3) for monitoring.
info_pred = helpers.cov_metrics(P_pred(1:3,1:3), opts.ssdr.rank_tau);
out.log10_kappaP_pred = log10(max(info_pred.kappa, eps));
out.log10_sminP_pred  = log10(max(info_pred.smin, realmin));
out.r_pred = info_pred.r_eff;

% ---------------- 2) Association (NN + MD gate) ----------------
Rk = opts.R0;
if use_vb
    Rk = vb.Rhat;
end

[assoc, stats] = helpers.associate_nn_md(x_pred, P_pred, z_rb, map.nL, Rk, opts);
out.n_inlier = stats.n_inlier;

% ---------------- 3) VB update for R_k (IW) ----------------
% Use inlier innovations outer products: A = sum nu nu'
m = 2;
A = zeros(2,2);
nis_list = [];

pose = x_pred(1:3);
for i=1:size(z_rb,2)
    if assoc(i).type ~= "assoc"
        continue;
    end
    j = assoc(i).j;
    lm_idx = 3 + (2*j-1);
    lm = x_pred(lm_idx:lm_idx+1);

    zhat = helpers.h_rb_phd(pose, lm);    % PHD-consistent bearing (+pi/2)
    nu = z_rb(:,i) - zhat;
    nu(2) = helpers.wrapToPi(nu(2));
    A = A + (nu*nu');
    nis_list(end+1,1) = assoc(i).nis; %#ok<AGROW>
end

if ~isempty(nis_list)
    out.nis_mean = mean(nis_list);
    out.nis_sum  = sum(nis_list);
else
    out.nis_mean = NaN;
    out.nis_sum  = NaN;
end

if use_vb
    rho = opts.vb.rho;
    u_prior = rho*(vb.u - m - 1) + (m + 1);
    U_prior = rho*vb.U;
    vb.u = u_prior + numel(nis_list);
    vb.U = U_prior + A;
    vb.Rhat = vb.U / max(vb.u - m - 1, 1e-6);
else
    vb.Rhat = opts.R0;
end
out.Rhat = vb.Rhat;
out.traceRhat = trace(vb.Rhat);

% ---------------- 4) Lambda update ----------------
if use_lambda && isfinite(out.nis_mean)
    switch opts.lambda.mode
        case 'nis_over_m'
            lambda_k = out.nis_mean / m;
        otherwise
            lambda_k = out.nis_mean / m;
    end
    lambda_k = max(1, min(lambda_k, opts.lambda.max));
    out.lambda = lambda_k;
else
    out.lambda = NaN;
end
out.traceQ = trace(Q);

% ---------------- 5) CKF sequential update ----------------
x_upd = x_pred;
P_upd = P_pred;

% Use updated Rk from VB for the updates
Rk = vb.Rhat;

% Update with associated measurements
for i=1:size(z_rb,2)
    if assoc(i).type ~= "assoc"
        continue;
    end
    j = assoc(i).j;
    [x_upd, P_upd, nis_i] = helpers.ckf_update_sub_rb_phd(x_upd, P_upd, z_rb(:,i), j, Rk, opts);
end

% Augment new landmarks
if opts.lm.add_new
    n_added = 0;
    for i=1:size(z_rb,2)
        if assoc(i).type ~= "new"
            continue;
        end
        if n_added >= opts.lm.max_new_per_frame
            break;
        end
        r = z_rb(1,i);
        if ~isfinite(r) || r < opts.lm.min_range || r > opts.lm.max_range
            continue;
        end
        [x_upd, P_upd] = helpers.augment_landmark_rb_phd(x_upd, P_upd, z_rb(:,i), Rk);
        map.nL = map.nL + 1;
        map.ids(map.nL,1) = map.nL;
        n_added = n_added + 1;
    end
    out.n_new = n_added;
end

% Post-step: avoid full eig SSDR. Keep symmetric covariance; compute pose-block metrics.
P_post = 0.5*(P_upd+P_upd');
info_post = helpers.cov_metrics(P_post(1:3,1:3), opts.ssdr.rank_tau);

out.log10_kappaP_post = log10(max(info_post.kappa, eps));
out.log10_sminP_post  = log10(max(info_post.smin, realmin));
out.r_post = info_post.r_eff;

% Commit
state.x = x_upd;
state.x(3) = helpers.wrapToPi(state.x(3));
state.P = P_post;

out.nL = map.nL;

% log q3 basic stats if present (optional)
if ~isempty(q3)
    out.q3_mean = mean(q3);
    out.q3_min = min(q3);
    out.q3_max = max(q3);
else
    out.q3_mean = NaN; out.q3_min = NaN; out.q3_max = NaN;
end

end

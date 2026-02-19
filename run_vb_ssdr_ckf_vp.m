function res = run_vb_ssdr_ckf_vp(mat_path, opts)
%RUN_VB_SSDR_CKF_VP  Complete VB-SSDR-CKF-SLAM runner for Victoria Park (PHD-sync).
%
% res = run_vb_ssdr_ckf_vp(mat_path, opts)
%
% INPUT
%   mat_path : path to VictoriaParkSinSincronizar.mat (PHD-SLAM 2.0 sync format)
%   opts     : (optional) options struct, see default_vb_ssdr_opts()
%
% OUTPUT
%   res.traj : [N×3] pose trajectory (x,y,theta) at laser frames
%   res.log  : struct array with metrics per laser frame
%   res.map  : final landmark map (means/covs)
%
% State layout:
%   x = [x;y;theta; l1x;l1y; l2x;l2y; ...]
% Observation layout per laser frame:
%   zt{k} is 3×M, we use z = zt{k}(1:2,:) (range,bearing)
% bearing definition matches PHD-SLAM 2.0:
%   b = wrapToPi(atan2(dy,dx) - theta + pi/2)
%
% GPS is not used here (kept raw elsewhere).

if nargin < 2 || isempty(opts)
    opts = default_vb_ssdr_opts();
end

S = load(mat_path);
assert(isfield(S,'TLsr') && isfield(S,'time') && isfield(S,'u') && isfield(S,'zt'), ...
    'MAT file must contain TLsr, time, u, zt');

TLsr = S.TLsr(:)';               % 1×N_lsr
t_odo = S.time(:)';              % 1×N_odo
u_odo = double(S.u);             % 2×N_odo
zt = S.zt;                       % 1×N_lsr cell

N = numel(TLsr);
fprintf('N(original)=%d\n', N);
if isfield(opts,'max_steps') && ~isempty(opts.max_steps)
    N = min(N, opts.max_steps);
end
fprintf('N(used)=%d\n', N);


% 设置最大步骤 后期可以去掉
if isfield(opts,'max_steps') && ~isempty(opts.max_steps)
    N = min(N, opts.max_steps);
end


% ----- init filter state -----
state = struct();
state.k = 1;
state.t = TLsr(1);
state.x = zeros(3,1);
state.P = diag([1, 1, deg2rad(10)^2]);   % loose prior

map = struct();
map.nL = 0;
map.ids = zeros(0,1);

% VB state (IW)
vb = struct();
vb.u = opts.vb.u0;
vb.U = opts.vb.U0;
vb.Rhat = opts.R0;

% lambda smoothing
lambda_ema = 1;

% 主循环 前面 初始化计时器：
tStart = tic;   % ← 放在 for k=1:N 之前

% logs
traj = nan(N,3);
log = repmat(empty_log(), N, 1);

traj(1,:) = state.x(1:3).';



for k = 2:N
    state.k = k;
    t0 = TLsr(k-1);
    t1 = TLsr(k);

    % Build control profile between [t0,t1]
    ctrl = helpers.build_ctrl_profile(t_odo, u_odo, t0, t1, opts.vehicle.L);

    % Measurements from zt (3×M): take 1:2
    zk = zt{k};
    if isempty(zk)
        z_rb = zeros(2,0);
        q3 = zeros(1,0);
    else
        zk = double(zk);
        if size(zk,1) ~= 3
            error('zt{%d} expected 3×M, got %dx%d', k, size(zk,1), size(zk,2));
        end
        z_rb = zk(1:2,:);
        q3 = zk(3,:);
    end

    % optional: bearing safety wrap
    z_rb(2,:) = helpers.wrapToPi(z_rb(2,:));

    % Step
    [state, map, vb, out] = vb_ssdr_ckf_step(state, map, vb, lambda_ema, ctrl, z_rb, q3, opts);

    % update lambda EMA
    if isfinite(out.lambda)
        lambda_ema = opts.lambda.alpha*lambda_ema + (1-opts.lambda.alpha)*out.lambda;
    end

    traj(k,:) = state.x(1:3).';
    out = orderfields(out, log(k));
    log(k) = out;

    % stepA：加进度条 + 每帧耗时打印（立刻知道卡在哪）
    if k==1, tStart=tic; end
    if mod(k,50)==0
        fprintf('k=%d/%d, nL=%d, time=%.1fs\n', k, N, map.nL, toc(tStart));
    end

end

% Export map means/covs
[muL, PL] = helpers.extract_landmarks(state.x, state.P);

res = struct();
res.opts = opts;
res.traj = traj;
res.log = log;
res.map = struct('mu', muL, 'P', PL, 'nL', map.nL);
res.meta = struct('mat_path', mat_path, 'N_lsr', N);

end

function out = empty_log()
out = struct();
out.n_meas_raw = 0;
out.n_meas_used = 0;
out.n_inlier = 0;
out.n_new = 0;
out.nL = 0;

out.nis_mean = NaN;
out.nis_sum = NaN;
out.lambda = NaN;
out.traceQ = NaN;

out.traceRhat = NaN;
out.Rhat = NaN(2,2);

out.log10_kappaP_pred = NaN;
out.log10_kappaP_post = NaN;
out.log10_sminP_pred = NaN;
out.log10_sminP_post = NaN;
out.r_pred = NaN;
out.r_post = NaN;
out.q3_mean = NaN;
out.q3_min = NaN;
out.q3_max = NaN;
end

function opts = default_vb_ssdr_opts()
%DEFAULT_VB_SSDR_OPTS Default configuration for VB-SSDR-CKF-SLAM (VP PHD-sync).

opts = struct();

% --- Mode switches ---
% 'G0': SSDR-CKF (fixed Q/R)
% 'G1': + VB for R_k
% 'G2': + lambda for Q_k
% 'G3': VB + lambda + SSDR
opts.mode = 'G3';

% --- Vehicle ---
opts.vehicle = struct();
opts.vehicle.L = 2.83;       % wheelbase (m)

% --- Baseline noises (SI units) ---
% Process noise on pose increment [dx;dy;dtheta]
opts.Q0 = diag([0.10^2, 0.10^2, deg2rad(3.0)^2]);

% Observation noise (range-bearing) baseline (m, rad)
opts.R0 = diag([0.30^2, deg2rad(2.0)^2]);

% --- Data association (NN + Mahalanobis) ---
opts.da = struct();
opts.da.chi2_gate = 9.21;       % chi2inv(0.99,2)
opts.da.max_cand = 200;         % candidate landmarks per measurement
opts.da.wrap_bearing = true;

% --- Landmark management ---
opts.lm = struct();
opts.lm.add_new = true;
opts.lm.min_range = 4.0;
opts.lm.max_range = 60.0;
opts.lm.max_new_per_frame = 5;

% --- VB (Inverse-Wishart for R_k) ---
opts.vb = struct();
opts.vb.enable = true;
opts.vb.rho = 0.90;
opts.vb.niter = 1;              % IW closed-form update is 1 pass; keep field for extensibility
m = 2;
opts.vb.u0 = 24;
opts.vb.U0 = (opts.vb.u0 - m - 1) * opts.R0;

% --- Lambda (adaptive Q scaling) ---
opts.lambda = struct();
opts.lambda.enable = true;
opts.lambda.max = 20;
opts.lambda.alpha = 0.90;       % EMA smoothing
opts.lambda.mode = 'nis_over_m'; % lambda = mean(NIS)/m, clipped

% --- SSDR ---
opts.ssdr = struct();
opts.ssdr.enable = true;
opts.ssdr.kappa_max = 1e8;
opts.ssdr.smin_floor = 1e-12;
opts.ssdr.rank_tau = 1e-10;

% --- Numerics ---
opts.num = struct();
opts.num.jitter0 = 1e-12;
opts.num.jitter_max = 1e-6;

% Apply mode
opts = helpers.apply_mode(opts);

end

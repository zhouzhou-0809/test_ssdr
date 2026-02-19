function ctrl = build_ctrl_profile(t_odo, u_odo, t0, t1, L)
%BUILD_CTRL_PROFILE Extract odometry controls between [t0,t1] (same units as input).
% Assumes t_odo, t0, t1 are in same time unit (PHD-sync uses doubles; typically seconds or ms).
%
% u_odo: 2×N, assumed [v; steering] where v in m/s, steering in rad.

idx = find(t_odo > t0 & t_odo <= t1);
if isempty(idx)
    [~,i1] = min(abs(t_odo - t1));
    idx = i1;
end

tt = double(t_odo(idx));
U = double(u_odo(:,idx));

% dt profile (seconds if tt is seconds)
dt = [tt(1) - t0, diff(tt)];
dt(dt <= 0) = median(dt(dt>0));

ctrl = struct();
ctrl.t0 = t0;
ctrl.t1 = t1;
ctrl.dt_total = max(t1 - t0, eps);
ctrl.t = tt(:);
ctrl.u = U;
ctrl.dt = dt(:);
ctrl.v_bar = sum(U(1,:).*dt) / sum(dt);
ctrl.delta_bar = sum(U(2,:).*dt) / sum(dt);
ctrl.L = L;
end

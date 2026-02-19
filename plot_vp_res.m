function plot_vp_res(res)
%PLOT_VP_RES Minimal plots for VB-SSDR-CKF results.

traj = res.traj;
t = (1:size(traj,1))';

figure; plot(traj(:,1), traj(:,2), 'k-'); axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)'); title(sprintf('VB-SSDR-CKF trajectory (%s)', res.opts.mode));

log = res.log;
nis = arrayfun(@(s) s.nis_mean, log);
figure; plot(t, nis, 'k'); grid on;
xlabel('laser frame'); ylabel('mean NIS'); title('Consistency (NIS)');

lk = arrayfun(@(s) s.lambda, log);
figure; plot(t, lk, 'k'); grid on;
xlabel('laser frame'); ylabel('lambda'); title('Adaptive process scaling');

trR = arrayfun(@(s) s.traceRhat, log);
figure; plot(t, trR, 'k'); grid on;
xlabel('laser frame'); ylabel('trace(Rhat)'); title('VB estimated R_k');

kpred = arrayfun(@(s) s.log10_kappaP_pred, log);
kpost = arrayfun(@(s) s.log10_kappaP_post, log);
figure; plot(t, kpred, 'b'); hold on; plot(t, kpost, 'r'); grid on;
xlabel('laser frame'); ylabel('log10 cond(P)'); legend('pred','post');
title('Covariance conditioning (SSDR)');

end

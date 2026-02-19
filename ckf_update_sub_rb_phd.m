function [x_new,P_new,nis] = ckf_update_sub_rb_phd(x,P,z,lm_j,R,opts)
%CKF_UPDATE_SUB_RB_PHD Fast CKF update on substate [pose; landmark_j] with SSDR dimension control.
% Substate is 5D: [x y theta lx ly]. Cubature points are generated with SSDR effective dimension control.
% Updates pose & selected landmark blocks; keeps other landmark blocks unchanged for scalability.

n = numel(x);
lm_idx = 3 + (2*lm_j-1);
sub_idx = [1:3, lm_idx:lm_idx+1];

xsub = x(sub_idx);
Psub = P(sub_idx, sub_idx);

% SSDR points (5D only)
[Xsub,w,~,~] = helpers.ssdr_points(xsub, Psub, opts.ssdr, opts.num.jitter0, opts.num.jitter_max);

Z = zeros(2, size(Xsub,2));
for c=1:size(Xsub,2)
    pose = Xsub(1:3,c);
    lm = Xsub(4:5,c);
    Z(:,c) = helpers.h_rb_phd(pose, lm);
end

% mean (circular for bearing)
zbar = Z*w.';
cb = sum(cos(Z(2,:)).*w);
sb = sum(sin(Z(2,:)).*w);
zbar(2) = atan2(sb,cb);

Pzz = zeros(2,2);
Pxz_sub = zeros(5,2);
for c=1:size(Xsub,2)
    dz = Z(:,c) - zbar;
    dz(2) = helpers.wrapToPi(dz(2));
    dx = Xsub(:,c) - xsub;
    dx(3) = helpers.wrapToPi(dx(3));
    Pzz = Pzz + w(c) * (dz*dz');
    Pxz_sub = Pxz_sub + w(c) * (dx*dz');
end
Pzz = Pzz + R;

nu = z - zbar;
nu(2) = helpers.wrapToPi(nu(2));

nis = nu'*(Pzz\nu);

Ksub = Pxz_sub / Pzz;

xsub_new = xsub + Ksub*nu;
xsub_new(3) = helpers.wrapToPi(xsub_new(3));

Psub_new = Psub - Ksub*Pzz*Ksub';
Psub_new = 0.5*(Psub_new + Psub_new');

% Inject substate mean/cov back
x_new = x;
x_new(sub_idx) = xsub_new;

P_new = P;
P_new(sub_idx, sub_idx) = Psub_new;
P_new = 0.5*(P_new + P_new');

end

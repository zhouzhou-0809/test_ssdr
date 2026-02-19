function [x_new,P_new] = ckf_update_one_rb_phd(x,P,z,lm_j,R,opts)
%CKF_UPDATE_ONE_RB_PHD CKF update for one range-bearing measurement (PHD bearing convention).

n = numel(x);
[X,w] = helpers.ckf_points(x,P,opts.num.jitter0,opts.num.jitter_max);

Z = zeros(2,size(X,2));
lm_idx = 3 + (2*lm_j-1);
for c=1:size(X,2)
    pose = X(1:3,c);
    lm = X(lm_idx:lm_idx+1,c);
    Z(:,c) = helpers.h_rb_phd(pose,lm);
end

% measurement mean (bearing mean via circular average)
zbar = Z*w.';
cb = sum(cos(Z(2,:)).*w);
sb = sum(sin(Z(2,:)).*w);
zbar(2) = atan2(sb,cb);

Pzz = zeros(2,2);
Pxz = zeros(n,2);
for c=1:size(X,2)
    dz = Z(:,c) - zbar;
    dz(2) = helpers.wrapToPi(dz(2));
    dx = X(:,c) - x;
    dx(3) = helpers.wrapToPi(dx(3));
    Pzz = Pzz + w(c) * (dz*dz');
    Pxz = Pxz + w(c) * (dx*dz');
end
Pzz = Pzz + R;

nu = z - zbar;
nu(2) = helpers.wrapToPi(nu(2));

K = Pxz / Pzz;
x_new = x + K*nu;
x_new(3) = helpers.wrapToPi(x_new(3));
P_new = P - K*Pzz*K';
P_new = 0.5*(P_new+P_new');
end

function zhat = h_rb_phd(pose, lm)
%H_RB_PHD Range-bearing observation model with PHD-SLAM 2.0 bearing convention.
% pose: [x;y;theta], lm: [lx;ly]
% bearing = helpers.wrapToPi(atan2(dy,dx) - theta + pi/2)

x = pose(1); y = pose(2); th = pose(3);
lx = lm(1); ly = lm(2);

dx = lx - x;
dy = ly - y;

r = hypot(dx,dy);
b = atan2(dy,dx) - th + pi/2;
b = helpers.wrapToPi(b);

zhat = [r; b];
end

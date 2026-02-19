function dpose = integrate_bicycle(ctrl, theta0, L)
%INTEGRATE_BICYCLE Integrate bicycle model over the control profile.
% Returns [dx; dy; dtheta] in meters/radians.

v = ctrl.u(1,:);
delta = ctrl.u(2,:);
dt = ctrl.dt(:)';

x = 0; y = 0; th = theta0;

for i=1:numel(dt)
    vi = v(i);
    di = delta(i);
    dti = dt(i);
    omega = (vi/L) * tan(di);
    dth = omega * dti;

    if abs(omega) < 1e-9
        x = x + vi*cos(th)*dti;
        y = y + vi*sin(th)*dti;
        th = th + dth;
    else
        x = x + (vi/omega)*(sin(th+dth) - sin(th));
        y = y + (vi/omega)*(-cos(th+dth) + cos(th));
        th = th + dth;
    end
end

dpose = [x; y; helpers.wrapToPi(th) - helpers.wrapToPi(theta0)];
end

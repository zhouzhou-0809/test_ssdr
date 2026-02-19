function [x_new,P_new] = augment_landmark_rb_phd(x,P,z,R)
%AUGMENT_LANDMARK_RB_PHD Add new landmark from range-bearing with PHD bearing convention.
% PHD bearing: b = atan2(dy,dx) - th + pi/2  => phi = th + b - pi/2

pose = x(1:3);
px = pose(1); py = pose(2); th = pose(3);
r = z(1); b = z(2);

phi = th + b - pi/2;

lx = px + r*cos(phi);
ly = py + r*sin(phi);

x_new = [x; lx; ly];

% Jacobians for augmentation
Jx = [1, 0, -r*sin(phi);
      0, 1,  r*cos(phi)];
Jz = [cos(phi), -r*sin(phi);
      sin(phi),  r*cos(phi)];

Pxlm = P(:,1:3) * Jx';
Plm = Jx*P(1:3,1:3)*Jx' + Jz*R*Jz';

P_new = [P, Pxlm;
         Pxlm', Plm];
P_new = 0.5*(P_new+P_new');
end

function [X,w] = ckf_points(x,P,j0,jm)
%CKF_POINTS Cubature points (2n) for CKF.
n = numel(x);
S = helpers.chol_psd(P, j0, jm);
Xi = sqrt(n) * S;
X = zeros(n, 2*n);
for i=1:n
    X(:,i) = x + Xi(:,i);
    X(:,i+n) = x - Xi(:,i);
end
w = (1/(2*n)) * ones(1,2*n);
end

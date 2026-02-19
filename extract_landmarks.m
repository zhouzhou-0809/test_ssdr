function [muL, PL] = extract_landmarks(x,P)
%EXTRACT_LANDMARKS Extract landmark means/cov blocks.
n = numel(x);
if n <= 3
    muL = zeros(2,0);
    PL = zeros(2,2,0);
    return;
end
nL = (n-3)/2;
muL = reshape(x(4:end), 2, nL);
PL = zeros(2,2,nL);
for j=1:nL
    idx = 3 + (2*j-1);
    PL(:,:,j) = P(idx:idx+1, idx:idx+1);
end
end

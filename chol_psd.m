function S = chol_psd(P, jitter0, jitter_max)
%CHOL_PSD Robust Cholesky with diagonal jitter.
P = 0.5*(P+P');
j = jitter0;
for it=1:20
    [S,p] = chol(P + j*eye(size(P)), 'lower');
    if p==0, return; end
    j = min(j*10, jitter_max);
end
error('chol_psd failed up to jitter=%g', jitter_max);
end

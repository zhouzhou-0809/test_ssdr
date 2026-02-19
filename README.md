# VB-SSDR-CKF-SLAM for Victoria Park (PHD-sync data)

This MATLAB codebase implements a complete **VB-SSDR-CKF-SLAM** pipeline designed for the Victoria Park dataset
in the **PHD-SLAM 2.0 synchronized format**:

- `VictoriaParkSinSincronizar.mat` containing:
  - `TLsr`  (1×7249) laser frame timestamps
  - `time`  (1×61945) odometry timestamps
  - `u`     (2×61945) controls (assumed [speed; steering] = [m/s; rad])
  - `zt`    (1×7249) cell, each `zt{k}` is **3×M** with:
    - row1: range (m)
    - row2: bearing (rad), using PHD definition: atan2(dy,dx) - theta + pi/2 wrapped
    - row3: extra attribute (e.g., trunk width/quality), logged only

GPS is **kept raw** (not interpolated) in this stage.

## Quick start
1) Put this folder somewhere (e.g., `D:/work/VB_SSDR_CKF_VP_MATLAB`)
2) Ensure your PHD-sync MAT file is accessible, e.g.:
   `D:/04Study/19VictoriaPark/PHD2/VictoriaParkSinSincronizar.mat`
3) In MATLAB:
```matlab
addpath(genpath('D:/work/VB_SSDR_CKF_VP_MATLAB'));
res = run_vb_ssdr_ckf_vp('D:/04Study/19VictoriaPark/PHD2/VictoriaParkSinSincronizar.mat');

plot_vp_res(res);
```

## Modes (ablation)
- G0: SSDR-CKF (fixed Q/R)
- G1: + VB for R_k
- G2: + lambda for Q_k
- G3: VB + lambda + SSDR

Set via `opts.mode`.

## Note on SSDR
`helpers/ssdr_regulate.m` is a robust **placeholder** spectral stabilizer (PSD enforcement + condition cap).
Replace it with your thesis SSDR operator for final paper results.


## Scalability patch
This patched version uses **local 5D CKF updates** (pose + associated landmark) and applies SSDR spectral effective-dimension control via `helpers.ssdr_points` on small subspaces. It avoids O(n^3) operations on the full SLAM covariance.

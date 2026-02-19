function opts = apply_mode(opts)
%APPLY_MODE Apply ablation mode switches.

mode = upper(string(opts.mode));
switch mode
    case "G0"
        opts.vb.enable = false;
        opts.lambda.enable = false;
        opts.ssdr.enable = true;
    case "G1"
        opts.vb.enable = true;
        opts.lambda.enable = false;
        opts.ssdr.enable = true;
    case "G2"
        opts.vb.enable = false;
        opts.lambda.enable = true;
        opts.ssdr.enable = true;
    case "G3"
        opts.vb.enable = true;
        opts.lambda.enable = true;
        opts.ssdr.enable = true;
    otherwise
        error('Unknown opts.mode=%s (use G0/G1/G2/G3)', opts.mode);
end
end

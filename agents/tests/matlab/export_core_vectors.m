function export_core_vectors(model_dir, out_csv)
%EXPORT_CORE_VECTORS  Reference vectors for the C core equivalence test.
%
%   EXPORT_CORE_VECTORS(model_dir, out_csv)
%
%   model_dir : folder "Simulation Model v2.0" of the simulation repository
%   out_csv   : default agents/tests/data/core_vectors.csv
%
%   Runs a realistic frame sequence (plant -> LLC emulator with timer clock ->
%   serial channel, 0.5 % frame loss) through the MATLAB sep_fusion_step and
%   sep_estimation_step, and writes every input and output. The C test
%   (tests/test_core.c) replays the inputs through the GENERATED C and must
%   reproduce the outputs (DESIGN.md V-U-3, V-U-5).
%
%   The sequence also contains one corrupted-but-plausible frame, one
%   implausible frame, and every 7th fusion message is dropped before the
%   estimator, so the cumulative-difference path is exercised.
%
%   Columns: tick encL encR gz ax ay az drop | emit S TH OM n_g n_s t_cum
%            still_frames still | did x1..x5 d1 d2 d3

    here = pwd;  c = onCleanup(@() cd(here));
    if nargin < 2 || isempty(out_csv)
        out_csv = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'core_vectors.csv');
    end
    cd(model_dir);

    geo = sep_geo_params();  prm = sep_ekf_params();  sp = sim_params();
    ag  = sp.agents;  Tb = sp.sim.Tb;
    lp  = llc_params();  lp.clock_mode = 'timer';  lp.tx_mode = 'interrupt';
    lp.sw_clock = 0;  lp.tx_block = 0;
    rng(7);

    % straight, pause, turn, pause, straight, pause (s, v, w)
    seg = [0 0 0; 3 0.029 0; 23 0 0; 26 0 0.35; 30.5 0 0; 33.5 0.029 0; 48.5 0 0; 52 0 0];
    N = round(52/Tb);
    stP = plant_init();  stL = llc_init();  stC = channel_init(sp.chan);
    F = zeros(0,7);
    for k = 1:N
        t = (k-1)*Tb;
        j = find(seg(:,1) <= t, 1, 'last');
        [stP, o] = plant_step(stP, seg(j,2), seg(j,3), zeros(1,6), 0, 0, Tb, sp.plant);
        [stL, b, nb, em] = llc_step(stL, o.w, o.a_body, 0, 0, o.arc, randn(1,5), ...
                                    lp, geo, sp.imu, Tb);
        [stC, cb, cn, ar] = channel_step(stC, b, nb, em, rand(1,3), sp.chan, Tb);
        if ar
            [f, ok] = raw_frame_from_bytes(cb, cn);
            if ok
                F(end+1,:) = [f.tick_ms f.encL f.encR f.gyr(3) f.acc]; %#ok<AGROW>
            end
        end
    end
    % anomalies
    F(400,2) = F(400,2) + 400;          % corrupted but plausible
    F(900,2) = F(900,2) + 1e7;          % implausible: must be rejected

    fs = sep_fusion_init();  es = sep_estimation_init(prm);
    n = size(F,1);
    R = nan(n, 8 + 9 + 9);
    n_emit = 0;
    for k = 1:n
        [fs, od, emit] = sep_fusion_step(fs, F(k,1), F(k,2), F(k,3), F(k,4), F(k,5:7), ...
                                         true, geo, ag);
        drop = 0;
        did = -1;  xe = nan(5,1);  dg = nan(3,1);
        if emit
            n_emit = n_emit + 1;
            drop = mod(n_emit, 7) == 0;
            if ~drop
                [es, xe, dg, d] = sep_estimation_step(es, od, prm, ag);
                did = double(d);
            end
        end
        R(k,:) = [F(k,:) drop double(emit) od.S od.TH od.OM od.n_g od.n_s od.t_cum ...
                  od.still_frames double(od.still) did xe.' dg.'];
    end

    fid = fopen(out_csv, 'w');
    assert(fid > 0, 'cannot write %s', out_csv);
    fprintf(fid, ['# tick,encL,encR,gz,ax,ay,az,drop,emit,S,TH,OM,n_g,n_s,t_cum,' ...
                  'still_frames,still,did,x1,x2,x3,x4,x5,d1,d2,d3\n']);
    for k = 1:n
        fprintf(fid, '%.17g,', R(k,1:end-1));
        fprintf(fid, '%.17g\n', R(k,end));
    end
    fclose(fid);
    fprintf('%d frames (%d emitted, %d rejected) -> %s\n', n, n_emit, fs.n_reject, out_csv);
end

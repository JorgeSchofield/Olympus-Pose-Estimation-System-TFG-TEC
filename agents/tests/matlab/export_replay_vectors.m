function export_replay_vectors(model_dir)
%EXPORT_REPLAY_VECTORS  Reference poses for the replay-parity test.
%
%   EXPORT_REPLAY_VECTORS(model_dir)
%
%   model_dir : folder "Simulation Model v2.0" of the simulation repository
%
%   Runs agents/tests/data/replay_input.txt through the model's HLC chain
%   (hlc_replay: raw_frame_from_bytes -> hlc_fusion_inputs -> sep_fusion_step
%   -> sep_estimation_step) once per time base and writes
%   replay_vectors_dt0.csv and replay_vectors_dt1.csv next to it. The C test
%   (tests/test_core.c, test_replay_parity) runs the same file through
%   pe_raw_parse -> pipeline.c -> the generated core, as the agents do in
%   `olympus-pose -r`, and must publish the same poses (DESIGN.md §9.6).
%
%   The input is an llc_emulator.py recording (26.7 Hz, tick +20 ms, IMU lost
%   50-56 s, 1 % frame loss, a corrupted encoder value, an LLC reset, TLM
%   lines) with malformed lines, an implausible jump and one v2.20 frame
%   added. Regenerate after changing the model or pose.conf section 1.

    here = pwd;  c = onCleanup(@() cd(here));
    data = fullfile(fileparts(mfilename('fullpath')), '..', 'data');
    cd(model_dir);
    in = fullfile(data, 'replay_input.txt');
    for k = 0:1
        hlc_replay(in, 'dt_source', k, 'csv', fullfile(data, sprintf('replay_vectors_dt%d.csv', k)));
    end
end

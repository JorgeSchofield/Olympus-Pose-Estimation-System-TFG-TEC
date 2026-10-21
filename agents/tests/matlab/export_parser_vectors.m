function export_parser_vectors(model_dir)
%EXPORT_PARSER_VECTORS  The model parser's verdict on tests/data/parser_cases.txt.
%
%   EXPORT_PARSER_VECTORS(model_dir)
%
%   Parses every line of parser_cases.txt (random mutations of v2.20 and
%   v3.0 RAW lines) with the model's raw_frame_from_bytes and writes
%   parser_vectors.csv: ok, version, tick, ax, gz, encL, encR, temp, imu_ok.
%   tests/test_core.c parses the same lines with pe_raw_parse and must agree
%   on every line (DESIGN.md §9.6).

    here = pwd;  c = onCleanup(@() cd(here));
    data = fullfile(fileparts(mfilename('fullpath')), '..', 'data');
    cd(model_dir);
    L = readlines(fullfile(data, 'parser_cases.txt'));
    if strlength(L(end)) == 0, L = L(1:end-1); end
    V = zeros(numel(L), 9);
    for i = 1:numel(L)
        b = uint8(char(L(i)));
        [f, ok] = raw_frame_from_bytes(b, numel(b));
        if ok
            V(i,:) = [1, f.version, f.tick_ms, f.acc(1), f.gyr(3), f.encL, f.encR, ...
                      f.temp, double(f.imu_ok)];
        end
    end
    fid = fopen(fullfile(data, 'parser_vectors.csv'), 'w');
    fprintf(fid, '%d,%d,%.0f,%d,%d,%d,%d,%d,%d\n', V.');
    fclose(fid);
    fprintf('%d lines, %d accepted\n', numel(L), sum(V(:,1)));
end

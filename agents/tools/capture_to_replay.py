#!/usr/bin/env python3
"""Converts a raw_*.csv of the LLC repo's capture_rover_data.py to a replay file.

    capture_to_replay.py raw_20261006_183344.csv -o run.txt
    olympus-pose -c pose.conf -r run.txt

Each row becomes "@<t_rx_ns> RAW:<tick>:...", the same format llcmux --record
writes, with t_rx_ns taken from the local_ts column (host time.time() at
reception). The frame keeps the version of the capture: 11 fields if the CSV
has the temp_raw and imu_ok columns (firmware v3.0), 9 otherwise (v2.20).

The host timestamps let est.dt_source = 1 be used on these captures, which is
needed while the firmware tick does not count real time (DESIGN.md §10.4).
"""
import argparse
import csv
import sys

V2 = ['tick_ms', 'ax_raw', 'ay_raw', 'az_raw', 'gx_raw', 'gy_raw', 'gz_raw', 'enc_l', 'enc_r']
V3 = V2 + ['temp_raw', 'imu_ok']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', help='raw_YYYYMMDD_HHMMSS.csv from capture_rover_data.py')
    ap.add_argument('-o', '--out', required=True, help='replay file to write')
    a = ap.parse_args()

    with open(a.csv, newline='') as f:
        rd = csv.DictReader(f)
        cols = rd.fieldnames or []
        if 'local_ts' not in cols or any(c not in cols for c in V2):
            sys.exit('capture_to_replay: %s is not a raw_*.csv capture (columns %s)' % (a.csv, cols))
        fields = V3 if all(c in cols for c in V3) else V2
        n = skipped = 0
        t_first = t_last = None
        with open(a.out, 'w', newline='\n') as out:
            for row in rd:
                try:
                    t_ns = int(round(float(row['local_ts']) * 1e9))
                    vals = [str(int(row[c])) for c in fields]
                except (TypeError, ValueError):
                    skipped += 1          # a truncated line from the capture
                    continue
                out.write('@%d RAW:%s\n' % (t_ns, ':'.join(vals)))
                t_first = t_ns if t_first is None else t_first
                t_last = t_ns
                n += 1
    if n == 0:
        sys.exit('capture_to_replay: no frames in %s' % a.csv)
    span = (t_last - t_first) / 1e9
    print('capture_to_replay: %d frames (%s), %d skipped, %.1f s, %.1f Hz -> %s'
          % (n, 'v3.0' if fields is V3 else 'v2.20', skipped, span, (n - 1) / span if span > 0 else 0.0, a.out))


if __name__ == '__main__':
    main()

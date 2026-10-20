#!/usr/bin/env python3
"""Odometry calibration from recorded runs: metres per count per side, track width.

DESIGN.md §9.5. The LLC sends one count sum per side, so each side has a single
constant m_per_tick (thesis eq. 3.1). The model derives it from per-wheel
measurements (sep_geo_params: diameters, counts per revolution, FR = 2.47 x FL);
this tool measures it on the whole rover, which also absorbs lost counts and the
loaded wheel diameter.

    # one or more straight runs of measured length (forward; metres)
    odom_calib.py --straight run1.txt:2.000 --straight run2.txt:2.000
    # optionally a spin in place of a known number of turns (CCW positive)
    odom_calib.py --straight run1.txt:2.0 --spin spin.txt:1

    # several tests in ONE recording: list the motions, then name them FILE#n
    odom_calib.py --list llc_record.txt
    odom_calib.py --straight llc_record.txt#1:2.0 --straight llc_record.txt#2:2.0 \\
                  --spin llc_record.txt#4:1

Each test needs >= 3 s at rest before it (gyro bias) and >= 1 s after (motions
closer than 1 s count as one). Use `llcmux --record-dir` recordings
(llc_record.txt) or capture_to_replay.py output; the "@<t_rx_ns>" timestamps
are used for the gyro integration, so the LLC clock error does not matter.

Straight run, for a measured distance D and the gyro heading change dth:
    m_R * dC_R = D + B*dth/2        m_L * dC_L = D - B*dth/2
(D is taken as an arc if the run curved). Several runs are combined by least
squares. The result is printed as the factors k_cal_R, k_cal_L to set in
sep_geo_params.m, so pose.conf keeps coming from the model (export_pose_conf).

Spin: B_eff = (m_R*dC_R - m_L*dC_L) / (2*pi*turns), and the gyro scale check
gyro_angle / (2*pi*turns).
"""
import argparse
import math
import os
import sys


def load_conf(path):
    vals = {}
    with open(path) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if '=' in line:
                k, v = line.split('=', 1)
                vals[k.strip()] = v.strip()
    try:
        return {'mR': float(vals['geo.m_per_tick_R']), 'mL': float(vals['geo.m_per_tick_L']),
                'B': float(vals['geo.B_eff']), 'gs': float(vals['ag.gyro_scale']),
                'axis': int(vals.get('est.gyro_yaw_axis', '2'))}
    except KeyError as e:
        sys.exit('odom_calib: %s lacks %s' % (path, e))


def unwrap(prev_raw, raw, acc):
    d = (raw - prev_raw + 2 ** 31) % 2 ** 32 - 2 ** 31
    return acc + d


def load_run(path, axis):
    """Frames (t_s, gz_lsb, encL, encR, imu_ok) with unwrapped encoders."""
    out = []
    no_ts = 0
    prev = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            t = None
            if line.startswith('@'):
                ts, _, line = line.partition(' ')
                t = int(ts[1:]) * 1e-9
            if not line.startswith('RAW:'):
                continue
            v = line[4:].split(':')
            if len(v) not in (9, 11):
                continue
            try:
                v = [int(x) for x in v]
            except ValueError:
                continue
            tick, gyr, encL, encR = v[0], v[4:7], v[7], v[8]
            imu_ok = v[10] == 1 if len(v) == 11 else True
            if t is None:
                no_ts += 1
                t = tick * 1e-3
            if prev is not None and tick < prev[0]:
                sys.exit('odom_calib: %s: LLC reset inside the run; record one test per file' % path)
            if prev is None:
                L, R = float(encL), float(encR)
            else:
                L = unwrap(prev[1], encL, out[-1][2])
                R = unwrap(prev[2], encR, out[-1][3])
            prev = (tick, encL, encR)
            out.append((t, gyr[axis], L, R, imu_ok))
    if len(out) < 10:
        sys.exit('odom_calib: %s: only %d RAW frames' % (path, len(out)))
    if no_ts:
        print('  WARNING %s: %d frames without "@t" timestamp, using the LLC tick '
              '(gyro angle off by the LLC clock scale)' % (path, no_ts))
    return out


def analyse(path, p):
    path, _, nseg = path.partition('#')
    fr = load_run(path, p['axis'])
    seg = segments(fr)
    if not seg:
        sys.exit('odom_calib: %s: the encoders never change' % path)
    k = int(nseg) - 1 if nseg else 0
    if not nseg and len(seg) > 1:
        sys.exit('odom_calib: %s has %d motions; name one as FILE#n (see --list)' % (path, len(seg)))
    if not 0 <= k < len(seg):
        sys.exit('odom_calib: %s has %d motions, no #%s' % (path, len(seg), nseg))
    i0, i1 = seg[k][0] - 1, seg[k][1]
    i_rest0 = seg[k - 1][1] + 1 if k > 0 else 0
    i_end = seg[k + 1][0] - 1 if k + 1 < len(seg) else len(fr) - 1
    rest = [f for f in fr[i_rest0:i0 + 1] if f[4]]
    t_rest = fr[i0][0] - fr[i_rest0][0]
    if t_rest < 2.0 or len(rest) < 20:
        print('  WARNING %s: only %.1f s at rest before moving; gyro bias poorly known' % (path, t_rest))
    bias = sum(f[1] for f in rest) / len(rest) if rest else 0.0
    dth = 0.0
    bad = 0
    fr = fr[:i_end + 1]
    for i in range(i0 + 1, len(fr)):
        dt = fr[i][0] - fr[i - 1][0]
        if not (fr[i][4] and fr[i - 1][4]):
            bad += 1
            continue
        dth += 0.5 * ((fr[i][1] - bias) + (fr[i - 1][1] - bias)) * p['gs'] * dt
    if bad:
        print('  WARNING %s: %d frames with imu_ok = 0 during the run; heading incomplete' % (path, bad))
    return {'dCL': fr[-1][2] - fr[i0][2], 'dCR': fr[-1][3] - fr[i0][3], 'dth': dth,
            't_move': fr[i1][0] - fr[i0][0], 't_rest': t_rest, 'bias': bias * p['gs']}


def segments(fr, gap_s=1.0):
    """(first, last) frame indices of each motion; rests shorter than gap_s join motions."""
    seg = []
    for i in range(1, len(fr)):
        if fr[i][2] != fr[i - 1][2] or fr[i][3] != fr[i - 1][3]:
            if seg and fr[i][0] - fr[seg[-1][1]][0] < gap_s:
                seg[-1][1] = i
            else:
                seg.append([i, i])
    return seg


def list_runs(path, p):
    fr = load_run(path, p['axis'])
    print('%s: %d frames, %.1f s' % (os.path.basename(path), len(fr), fr[-1][0] - fr[0][0]))
    for n, (a, b) in enumerate(segments(fr), 1):
        print('  #%d  t %7.1f s  %6.1f s moving  counts R %+d L %+d'
              % (n, fr[a][0] - fr[0][0], fr[b][0] - fr[a - 1][0],
                 fr[b][3] - fr[a - 1][3], fr[b][2] - fr[a - 1][2]))


def split_arg(s):
    path, _, val = s.rpartition(':')
    if not path:
        sys.exit('odom_calib: expected FILE:VALUE, got %s' % s)
    return path, float(val)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-c', '--conf', default='/etc/olympus-pose/pose.conf')
    ap.add_argument('--straight', action='append', default=[], metavar='FILE:METRES')
    ap.add_argument('--spin', action='append', default=[], metavar='FILE:TURNS')
    ap.add_argument('--list', action='append', default=[], metavar='FILE',
                    help='print the motions found in a recording (to name them FILE#n)')
    a = ap.parse_args()
    p = load_conf(a.conf)
    for path in a.list:
        list_runs(path, p)
    if a.list and not a.straight and not a.spin:
        return
    if not a.straight and not a.spin:
        sys.exit('odom_calib: give at least one --straight or --spin run')
    mR, mL, B = p['mR'], p['mL'], p['B']
    print('pose.conf: m_per_tick_R %.6e, m_per_tick_L %.6e, B_eff %.4f m' % (mR, mL, B))

    if a.straight:
        print('\nSTRAIGHT RUNS')
        sRR = sRd = sLL = sLd = 0.0
        runs = []
        for s in a.straight:
            path, D = split_arg(s)
            r = analyse(path, p)
            arc = D if abs(r['dth']) < 1e-3 else D * (r['dth'] / 2) / math.sin(r['dth'] / 2)
            dR, dL = arc + B * r['dth'] / 2, arc - B * r['dth'] / 2
            s_conf = 0.5 * (mR * r['dCR'] + mL * r['dCL'])
            print('  %s: D %.3f m, counts R %d L %d, gyro %+.2f deg, %.1f s moving, '
                  'app estimate %.3f m (%+.1f %%)'
                  % (os.path.basename(path), D, r['dCR'], r['dCL'], math.degrees(r['dth']), r['t_move'],
                     s_conf, 100 * (s_conf / D - 1)))
            sRR += r['dCR'] ** 2; sRd += dR * r['dCR']
            sLL += r['dCL'] ** 2; sLd += dL * r['dCL']
            runs.append((path, D, r))
        mR, mL = sRd / sRR, sLd / sLL
        print('\n  measured m_per_tick_R %.6e  (k_cal_R = %.4f)' % (mR, mR / p['mR']))
        print('  measured m_per_tick_L %.6e  (k_cal_L = %.4f)' % (mL, mL / p['mL']))
        print('  counts per metre: right %.0f, left %.0f (right/left %.3f; pose.conf %.3f)'
              % (1 / abs(mR), 1 / abs(mL), abs(mL / mR), abs(p['mL'] / p['mR'])))
        for path, D, r in runs:
            s = 0.5 * (mR * r['dCR'] + mL * r['dCL'])
            print('  residual %s: %+.1f mm' % (os.path.basename(path), 1000 * (s - D)))
        print('\n  -> sep_geo_params.m: k_cal_R = k_cal_R_old * %.4f; k_cal_L = k_cal_L_old * %.4f;'
              % (mR / p['mR'], mL / p['mL']))
        print('     then export_pose_conf and paste section 1 of pose.conf.')
        if len(runs) < 2:
            print('  NOTE: one run gives no check of repeatability; do at least 3.')

    if a.spin:
        print('\nSPINS (constants: %s)' % ('measured above' if a.straight else 'pose.conf'))
        for s in a.spin:
            path, turns = split_arg(s)
            r = analyse(path, p)
            ang = 2 * math.pi * turns if turns != 0 else r['dth']
            b = (mR * r['dCR'] - mL * r['dCL']) / ang
            print('  %s: %+.2f turns, counts R %d L %d -> B_eff %.4f m (x%.3f of pose.conf)'
                  % (os.path.basename(path), ang / (2 * math.pi), r['dCR'], r['dCL'], b, b / B))
            if turns != 0:
                print('    gyro angle %+.2f deg = %.4f x the true angle (gyro scale check)'
                      % (math.degrees(r['dth']), r['dth'] / ang))
        print('  -> sep_geo_params.m: chi = chi_old * (B_eff measured / B_eff pose.conf)')


if __name__ == '__main__':
    main()

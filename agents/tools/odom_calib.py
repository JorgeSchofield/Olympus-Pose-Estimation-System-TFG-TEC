#!/usr/bin/env python3
"""Odometry calibration from recorded runs: metres per count per side, track width.

DESIGN.md §9.5. The LLC sends one count sum per side, so each side has a single
constant m_per_tick (thesis eq. 3.1). The model derives it from per-wheel
measurements (sep_geo_params: diameters, counts per revolution, k_cal factors);
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

    # a run driven as several commands with stops (HLC station mode): one test
    # is a RANGE of motions, FILE#first-last; --events shows the GUI/pose_ctl
    # MARK labels between the motions to tell which motions form which run
    odom_calib.py --list llc_record.txt --events /var/lib/olympus-pose/runs/<run>/events.csv
    odom_calib.py --list llc_record.txt --events /var/lib/olympus-pose/gui_marks.csv
    odom_calib.py --straight llc_record.txt#1-4:2.0 --straight llc_record.txt#5-8:2.0

Each test needs >= 3 s at rest before it (gyro bias) and >= 1 s after (motions
closer than 1 s count as one). Stops inside a range are fine: the counts and the
gyro are taken over the whole range, from the rest before its first motion to the
rest after its last. The gyro bias is measured again in EVERY rest (before, between
and after the motions, >= 0.5 s each) and interpolated linearly in time between
them, as the estimator does at rest (ZARU): over a stop-and-go run of several
minutes the MPU-9250 bias drifts, and a bias taken only before the first motion
turned 0.06 deg/s of drift into ~30 deg of false heading. Short IMU dropouts
(imu_ok = 0) during a motion are bridged with the last valid sample. LLC resets in the recording (the DTR reset when
olympus_hlc opens its port) are boundaries: no motion is measured across one. Use `llcmux --record-dir` recordings
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
    """Frames (t_s, gz_lsb, encL, encR, imu_ok, epoch) with unwrapped encoders.

    An LLC reset (tick going backwards: power-up, or the DTR pulse llcmux sends
    when olympus_hlc opens its port) starts a new epoch: the counters restart
    from zero, so nothing is measured across it. A recording normally has one
    such reset near its start."""
    out = []
    no_ts = 0
    prev = None
    epoch = 0
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
                epoch += 1                       # LLC reset: counters restart
                prev = None
            if prev is None:
                L, R = float(encL), float(encR)
            else:
                L = unwrap(prev[1], encL, out[-1][2])
                R = unwrap(prev[2], encR, out[-1][3])
            prev = (tick, encL, encR)
            out.append((t, gyr[axis], L, R, imu_ok, epoch))
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
    if not nseg and len(seg) > 1:
        sys.exit('odom_calib: %s has %d motions; name one as FILE#n or a range FILE#n-m (see --list)'
                 % (path, len(seg)))
    try:
        a_, _, b_ = (nseg or '1').partition('-')
        k, k1 = int(a_) - 1, int(b_ or a_) - 1
    except ValueError:
        sys.exit('odom_calib: %s: bad motion "#%s" (use #n or #n-m)' % (path, nseg))
    if not (0 <= k <= k1 < len(seg)):
        sys.exit('odom_calib: %s has %d motions, no #%s' % (path, len(seg), nseg))
    if fr[seg[k][0]][5] != fr[seg[k1][1]][5]:
        sys.exit('odom_calib: %s#%s crosses an LLC reset; counts cannot be joined' % (path, nseg))
    i0, i1 = seg[k][0] - 1, seg[k1][1]
    ep = fr[i0][5]
    e0 = next(i for i in range(len(fr)) if fr[i][5] == ep)               # first frame of the epoch
    e1 = max(i for i in range(e0, len(fr)) if fr[i][5] == ep)          # last frame of the epoch
    i_rest0 = max(seg[k - 1][1] + 1 if k > 0 else 0, e0)
    i_end = min(seg[k1 + 1][0] - 1 if k1 + 1 < len(seg) else len(fr) - 1, e1)
    t_rest = fr[i0][0] - fr[i_rest0][0]
    if t_rest < 2.0:
        print('  WARNING %s: only %.1f s at rest before moving; gyro bias poorly known' % (path, t_rest))
    # Rests used for the bias: before the first motion, every stop between the
    # motions of the range, and after the last one. Each gives (mid time, mean gz).
    rests = [(i_rest0, i0)]
    rests += [(seg[j][1] + 1, seg[j + 1][0] - 1) for j in range(k, k1)]
    rests.append((i1 + 1, i_end))
    knots = []
    for a, b in rests:
        good = [f for f in fr[a:b + 1] if f[4]]
        if len(good) >= 25 and good[-1][0] - good[0][0] >= 0.5:       # >= 0.5 s of valid samples
            knots.append((0.5 * (good[0][0] + good[-1][0]), sum(f[1] for f in good) / len(good)))
    if not knots:
        sys.exit('odom_calib: %s#%s: no rest of >= 0.5 s with a valid IMU; the gyro bias is unknown'
                 % (path, nseg))

    def bias_at(t):                       # piecewise linear, constant beyond the end knots
        if t <= knots[0][0]:
            return knots[0][1]
        for (ta, ba), (tb, bb) in zip(knots, knots[1:]):
            if t <= tb:
                return ba + (bb - ba) * (t - ta) / (tb - ta)
        return knots[-1][1]

    dth = 0.0
    bad = 0
    bad_t = 0.0
    fr = fr[:i_end + 1]
    last = fr[i0][1]
    for i in range(i0 + 1, len(fr)):
        dt = fr[i][0] - fr[i - 1][0]
        g = fr[i][1]
        if fr[i][4]:
            last = g
        else:                              # IMU dropout: hold the last valid sample
            g = last
            bad += 1
            bad_t += dt
        g_prev = fr[i - 1][1] if fr[i - 1][4] else last
        t_mid = 0.5 * (fr[i][0] + fr[i - 1][0])
        dth += (0.5 * (g + g_prev) - bias_at(t_mid)) * p['gs'] * dt
    if bad:
        print('  WARNING %s: %d frames (%.2f s) with imu_ok = 0 during the run, bridged with the '
              'last valid gyro sample; check the IMU wiring' % (path, bad, bad_t))
    b0, b1 = knots[0][1] * p['gs'], knots[-1][1] * p['gs']
    return {'dCL': fr[-1][2] - fr[i0][2], 'dCR': fr[-1][3] - fr[i0][3], 'dth': dth,
            't_move': fr[i1][0] - fr[i0][0], 't_rest': t_rest, 'bias': b0,
            'bias_drift': b1 - b0, 'n_knots': len(knots), 'n_motions': k1 - k + 1}


def segments(fr, gap_s=1.0):
    """(first, last) frame indices of each motion; rests shorter than gap_s join motions."""
    seg = []
    for i in range(1, len(fr)):
        if fr[i][5] != fr[i - 1][5]:
            continue                             # LLC reset between the frames, not motion
        if fr[i][2] != fr[i - 1][2] or fr[i][3] != fr[i - 1][3]:
            if seg and fr[i][0] - fr[seg[-1][1]][0] < gap_s and fr[seg[-1][1]][5] == fr[i][5]:
                seg[-1][1] = i
            else:
                seg.append([i, i])
    return seg


def load_marks(events):
    """MARK labels as (t_s, text), from an olympus-pose events.csv (t_ns, MARK
    events) or from the GUI's gui_marks.csv (t_mono_ns, label). Both times and the
    recording's @t_rx_ns are the same CLOCK_MONOTONIC of the HLC."""
    import csv
    marks = []
    try:
        with open(events, newline='') as f:
            rows = list(csv.reader(f))
    except OSError as e:
        sys.exit('odom_calib: %s' % e)
    if not rows:
        return marks
    head = rows[0]
    if 't_mono_ns' in head and 'label' in head:          # gui_marks.csv
        it, il = head.index('t_mono_ns'), head.index('label')
        ir = head.index('app_reply') if 'app_reply' in head else None
        no_reply = 0
        for r in rows[1:]:
            if len(r) > max(it, il) and r[it].isdigit():
                marks.append((int(r[it]) * 1e-9, r[il]))
                if ir is not None and len(r) > ir and not r[ir].startswith('OK'):
                    no_reply += 1
        if no_reply:
            print('  NOTE %s: olympus-pose did not confirm %d of these marks (app_reply), '
                  'so they are not in its events.csv' % (events, no_reply))
    else:                                                # events.csv: t_ns,agent,event
        for r in rows[1:]:
            if len(r) >= 3 and r[0].isdigit():
                ev = ','.join(r[2:])
                if ev.startswith('MARK '):
                    marks.append((int(r[0]) * 1e-9, ev[5:].strip()))
    return sorted(marks)


def list_runs(path, p, events=None):
    fr = load_run(path, p['axis'])
    print('%s: %d frames, %.1f s, %d LLC reset(s)'
          % (os.path.basename(path), len(fr), fr[-1][0] - fr[0][0], fr[-1][5]))
    marks = load_marks(events) if events else []
    if events:
        print('  %s: %d MARK(s)' % (events, len(marks)))
    t0, t_end = fr[0][0], fr[-1][0]
    outside = [m for m in marks if not t0 - 2 <= m[0] <= t_end + 2]      # 2 s margin: a mark
    marks = [m for m in marks if t0 - 2 <= m[0] <= t_end + 2]           # sent just at a stop

    def flush(t_limit):
        while marks and marks[0][0] <= t_limit:
            t, txt = marks.pop(0)
            print('      t %7.1f s  MARK "%s"' % (t - t0, txt))
    for n, (a, b) in enumerate(segments(fr), 1):
        flush(fr[a][0])
        print('  #%d  t %7.1f s  %6.1f s moving  counts R %+d L %+d'
              % (n, fr[a][0] - t0, fr[b][0] - fr[a - 1][0],
                 fr[b][3] - fr[a - 1][3], fr[b][2] - fr[a - 1][2]))
    flush(float('inf'))
    if outside:
        print('  (%d MARK(s) of %s fall outside this recording: another llcmux start, '
              'another boot, or another run folder)' % (len(outside), events))


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
                    help='print the motions found in a recording (to name them FILE#n or FILE#n-m)')
    ap.add_argument('--events', metavar='EVENTS_CSV',
                    help='with --list: show the MARK labels between the motions, from an olympus-pose '
                         'events.csv or the GUI\'s gui_marks.csv')
    a = ap.parse_args()
    p = load_conf(a.conf)
    for path in a.list:
        list_runs(path, p, a.events)
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
            print('  %s: D %.3f m, counts R %d L %d, gyro %+.2f deg, %.1f s %s, '
                  'app estimate %.3f m (%+.1f %%)'
                  % (os.path.basename(path), D, r['dCR'], r['dCL'], math.degrees(r['dth']), r['t_move'],
                     'moving' if r['n_motions'] == 1 else 'from first to last of %d motions' % r['n_motions'],
                     s_conf, 100 * (s_conf / D - 1)))
            print('      gyro bias %+.3f deg/s, drift %+.3f deg/s over the run (%d rests used)'
                  % (math.degrees(r['bias']), math.degrees(r['bias_drift']), r['n_knots']))
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
            # A left (CCW) spin drives both sides' counts negative (the right motors are
            # mirrored), a right (CW) spin both positive. Catch a turns value with the
            # wrong sign before it produces a negative track width.
            if turns != 0 and (r['dCR'] > 0 and r['dCL'] > 0) == (turns > 0):
                print('  WARNING %s: the counts say this spin was to the %s, but turns = %+g; '
                      'give a %s number of turns (CCW positive)'
                      % (os.path.basename(path), 'right (CW)' if r['dCR'] > 0 else 'left (CCW)', turns,
                         'negative' if r['dCR'] > 0 else 'positive'))
            ang = 2 * math.pi * turns if turns != 0 else r['dth']
            b = (mR * r['dCR'] - mL * r['dCL']) / ang
            print('  %s: %+.2f turns, counts R %d L %d -> B_eff %.4f m (x%.3f of pose.conf)'
                  % (os.path.basename(path), ang / (2 * math.pi), r['dCR'], r['dCL'], b, b / B))
            if turns != 0:
                print('    gyro angle %+.2f deg = %.4f x the true angle (gyro scale check; bias drift '
                      '%+.3f deg/s, %d rests used)'
                      % (math.degrees(r['dth']), r['dth'] / ang, math.degrees(r['bias_drift']), r['n_knots']))
        print('  -> sep_geo_params.m: chi = chi_old * (B_eff measured / B_eff pose.conf)')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""LLC emulator: RAW: frames at 50 Hz for testing olympus-pose without the rover.

DESIGN.md §16.1 (SIL) and V-OE4-2. It plays a scripted motion and writes
firmware-v2.20 frames

    RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>

into a pseudo-terminal (live, paced by the clock) or a file (as fast as
possible, for `olympus-pose -r`). Counts and gyro LSB come from the same model
parameters the app uses (pose.conf), so a perfect run should come back as the
scripted path.

Examples (on the RPi, as root for the real-time settings of the app):

    # 15 min at 50 Hz into a pty, app reads it as its raw input (OE4 test)
    llc_emulator.py --link /tmp/llc_emu --prefix --scenario umbmark --duration 900
    olympus-pose -c pose.conf   # with io.raw_input = /tmp/llc_emu

    # a recorded file for replay, with frame loss and one corrupted frame
    llc_emulator.py --out /tmp/run.txt --prefix --loss 0.005 --corrupt-at 20
    olympus-pose -c pose.conf -r /tmp/run.txt

Scenarios: straight (5 m), spin (360 deg), umbmark (2 m square, CCW) - all start
with 5 s at rest and have 3 s pauses, which the estimator needs to observe the
gyro bias.
"""
import argparse
import math
import os
import random
import sys
import time

V_MAX = 0.029        # m/s, rover ground speed at 100 % PWM (sim_params)
W_TURN = 0.35        # rad/s, turn in place
PAUSE = 3.0          # s, pauses between segments (gyro bias observable via ZARU)
PAUSE_START = 5.0    # s, initial rest: the estimator needs est.t_init_s (3 s) of it


def load_conf(path):
    vals = {}
    with open(path) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if '=' in line:
                k, v = line.split('=', 1)
                vals[k.strip()] = v.strip()
    need = ['geo.m_per_tick_R', 'geo.m_per_tick_L', 'geo.B_eff', 'ag.gyro_scale', 'ag.loop_ms']
    missing = [k for k in need if k not in vals]
    if missing:
        sys.exit('llc_emulator: %s lacks %s' % (path, ', '.join(missing)))
    return {k: float(vals[k]) for k in need}


def scenario(name):
    """List of (duration_s, v_m_s, w_rad_s)."""
    seg = [(PAUSE_START, 0.0, 0.0)]
    if name == 'straight':
        seg += [(5.0 / V_MAX, V_MAX, 0.0), (PAUSE, 0.0, 0.0)]
    elif name == 'spin':
        seg += [(2 * math.pi / W_TURN, 0.0, W_TURN), (PAUSE, 0.0, 0.0)]
    elif name == 'umbmark':
        for _ in range(4):
            seg += [(2.0 / V_MAX, V_MAX, 0.0), (PAUSE, 0.0, 0.0),
                    ((math.pi / 2) / W_TURN, 0.0, W_TURN), (PAUSE, 0.0, 0.0)]
    else:
        sys.exit('llc_emulator: unknown scenario %s' % name)
    return seg


def wrap_i32(v):
    return (v + 2 ** 31) % 2 ** 32 - 2 ** 31


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-c', '--conf', default='/etc/olympus-pose/pose.conf')
    ap.add_argument('--link', help='create a pty and symlink it here (live mode)')
    ap.add_argument('--out', help='write to this file as fast as possible (replay mode)')
    ap.add_argument('--scenario', default='umbmark', choices=['straight', 'spin', 'umbmark'])
    ap.add_argument('--duration', type=float, default=0.0, help='s; repeat the scenario until then (0 = once)')
    ap.add_argument('--rate', type=float, default=50.0, help='frames per second')
    ap.add_argument('--prefix', action='store_true', help='add "@<t_rx_ns> " like llcmux')
    ap.add_argument('--bias', type=float, default=0.012, help='gyro bias, rad/s')
    ap.add_argument('--noise', type=float, default=0.00116,
                    help='gyro noise std per sample, rad/s (model: 0.01 deg/s/sqrt(Hz) at 44 Hz DLPF)')
    ap.add_argument('--loss', type=float, default=0.0, help='frame loss probability')
    ap.add_argument('--corrupt-at', type=float, default=-1, help='s: corrupt one encoder value')
    ap.add_argument('--reset-at', type=float, default=-1, help='s: simulate an LLC reset')
    ap.add_argument('--tlm', action='store_true', help='also emit a TLM: line every second')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    if bool(a.link) == bool(a.out):
        sys.exit('llc_emulator: use exactly one of --link or --out')

    p = load_conf(a.conf)
    rnd = random.Random(a.seed)
    seg = scenario(a.scenario)
    one = sum(s[0] for s in seg)
    total = a.duration if a.duration > 0 else one
    dt = 1.0 / a.rate
    tick_ms = int(round(1000.0 * dt))

    if a.link:
        import pty
        master, slave = pty.openpty()
        import tty
        tty.setraw(slave)
        name = os.ttyname(slave)
        if os.path.lexists(a.link):
            os.remove(a.link)
        os.symlink(name, a.link)
        out = os.fdopen(master, 'wb', buffering=0)
        print('llc_emulator: %s -> %s, %s for %.0f s at %.0f Hz' % (a.link, name, a.scenario, total, a.rate))
    else:
        out = open(a.out, 'wb')

    # side distances: right = v + w*B/2, left = v - w*B/2 (forward positive)
    dR = dL = 0.0
    tick = 0
    k = 0
    t = 0.0
    t0 = time.monotonic()
    t_ns0 = time.monotonic_ns()
    reset_done = False
    try:
        while t < total:
            ts = t % one
            acc = 0.0
            v = w = 0.0
            for d, sv, sw in seg:
                if ts < acc + d:
                    v, w = sv, sw
                    break
                acc += d
            dR += (v + w * p['geo.B_eff'] / 2) * dt
            dL += (v - w * p['geo.B_eff'] / 2) * dt
            if a.reset_at >= 0 and not reset_done and t >= a.reset_at:
                reset_done = True        # LLC reboot: tick and accumulators restart
                dR = dL = 0.0
                tick = 0
            encR = wrap_i32(int(round(dR / p['geo.m_per_tick_R'])))
            encL = wrap_i32(int(round(dL / p['geo.m_per_tick_L'])))
            gz = int(round((w + a.bias + rnd.gauss(0, a.noise)) / p['ag.gyro_scale']))
            gz = max(-32768, min(32767, gz))
            if a.corrupt_at >= 0 and abs(t - a.corrupt_at) < dt / 2:
                encL += 400               # plausible corrupted byte
            line = 'RAW:%d:%d:%d:%d:%d:%d:%d:%d:%d' % (tick % 2 ** 32, rnd.randint(-40, 40),
                                                      rnd.randint(-40, 40), 16384, 0, 0, gz, encL, encR)
            if a.link:
                target = t0 + t
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)
                t_ns = time.monotonic_ns()
            else:
                t_ns = t_ns0 + int(t * 1e9)
            if a.prefix:
                line = '@%d %s' % (t_ns, line)
            if rnd.random() >= a.loss:
                out.write((line + '\n').encode())
            if a.tlm and k % int(a.rate) == 0:
                tl = 'TLM:NORMAL:0:%dms' % tick
                out.write((('@%d ' % t_ns if a.prefix else '') + tl + '\n').encode())
            tick += tick_ms
            k += 1
            t = k * dt
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        out.close()
        if a.link and os.path.islink(a.link):
            os.remove(a.link)
    print('llc_emulator: %d frames, final side distances R %.3f m, L %.3f m' % (k, dR, dL))


if __name__ == '__main__':
    main()

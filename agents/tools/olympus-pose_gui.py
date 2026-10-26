#!/usr/bin/env python3
"""olympus-pose GUI: one web page for the pose, the GPS, test markers, controls and logs.

DESIGN.md §11.4. Runs on the rover next to olympus-pose; open it from a laptop on
the rover's network (no internet needed):

    olympus-pose_gui.py                   # http://<rover-ip>:8080
    olympus-pose_gui.py --run /var/lib/olympus-pose/runs/<run>   # replay a logged run
    olympus-pose_gui.py --run <run> --gpx track.gpx              # GPX with the test marks

Views:
  Pose      live x, y, theta, distance, mode and flags; the pose track; the OE
            indicators from the app's STATS (latency, delivery, LLC clock scale)
  GPS       fix state, satellites, HDOP, fix rate, scatter at rest, track in metres
  Markers   "Mark" stores the 10 s average GPS position, the pose and a MARK event
            in the app's log, for each test point
  Controls  reset pose, suspend/resume agents (through the CMAES AMS), run logs
            to download, GPX export

It listens to the pose datagrams (UDP io.udp_pose_port, so pose_listen.py cannot
run at the same time) and the GPS datagrams (io.udp_gps_port), and talks to the
control port (io.udp_ctrl_port). There is no password: anyone on the rover's
network can use the controls; disruptive ones ask for confirmation. The rover is
never driven from here, and services are not started or stopped.

Cost on the HLC: a separate, normal-priority process (the pose threads are
SCHED_FIFO and cannot be delayed by it). Tracks are built once per datagram and
the page only fetches what it does not have, so a refresh costs the same after
5 min or 5 h.
"""
import argparse
import binascii
import csv
import json
import math
import os
import shutil
import socket
import struct
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, quote, unquote, urlsplit

POSE_FMT = '<4sIQQIHH3d2d4d2dII'      # 128 bytes, src/core/pose_dgram.h
GPS_FMT = '<4sIQ6dI4BII'              # 80 bytes, same file
R_EARTH = 6371000.0
MAX_POINTS = 20000
QUALITY = {0: 'no fix', 1: 'GPS', 2: 'DGPS', 3: 'PPS', 4: 'RTK fixed', 5: 'RTK float',
           6: 'estimated', 7: 'manual', 8: 'simulation'}
FLAGS = ['VALID', 'STILL', 'SLIP', 'STALE', 'GAP', 'IMU_INVALID', 'ROLLOVER', 'LLC_RESET',
         'RESUMED', 'REREF']
MODES = ['WAIT_DATA', 'INITIALIZING', 'RUNNING']
STATS_COLS = ('t_ns,t_run_s,rss_kb,cpu_s,poses,lat_p50_ms,lat_p95_ms,lat_max_ms,interval_p95_ms,'
              'interval_max_ms,llc_clock_scale,acq_delivered,acq_lost,acq_max_gap,acq_drops,'
              'fus_delivered,fus_lost,fus_max_gap,fus_drops,est_delivered,est_lost,est_max_gap,'
              'est_drops,gps_fixes,gps_valid,log_drops,st_acq,st_fus,st_est,st_com,st_gps').split(',')
AGENTS = [('acquisition', 'st_acq'), ('fusion', 'st_fus'), ('estimation', 'st_est'),
          ('communication', 'st_com'), ('gps', 'st_gps')]
AGENT_MODES = ['active', 'suspended', 'waiting', 'terminated', 'not registered']
RUN_FILES = ('.csv', '.txt')


def tail(seq, t_from, key):
    """Elements at the end of seq with key >= t_from (seq ordered by key)."""
    i = len(seq)
    while i > 0 and key(seq[i - 1]) >= t_from:
        i -= 1
    return seq[i:]


def load_conf(path):
    vals = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if '=' in line:
                    k, v = line.split('=', 1)
                    vals[k.strip()] = v.strip()
    except OSError:
        print('olympus-pose_gui: %s not readable, using the default ports' % path)

    def num(k, d):
        try:
            return int(vals.get(k, d))
        except ValueError:
            return d
    # io.pose_frame: axes of the published pose (DESIGN.md §11.0). The app writes
    # x, y in this frame; the page only needs it to draw the heading and labels.
    return {'pose_port': num('io.udp_pose_port', 47001), 'gps_port': num('io.udp_gps_port', 47003),
            'ctrl_port': num('io.udp_ctrl_port', 47002), 'pose_frame': num('io.pose_frame', 0),
            'log_dir': vals.get('io.log_dir') or '/var/lib/olympus-pose/runs'}


def boot_id():
    try:
        with open('/proc/sys/kernel/random/boot_id') as f:
            return f.read().strip()
    except OSError:
        return 'unknown'


class Track:
    """Points [a, b, t_rel, ..., brk, id]; the page asks for ids it does not have."""

    def __init__(self):
        self.epoch = 0
        self.clear()

    def clear(self):
        self.epoch += 1
        self.pts = []
        self.next_id = 1

    def add(self, p):
        p.append(self.next_id)
        self.next_id += 1
        self.pts.append(p)
        if len(self.pts) > MAX_POINTS + 1000:     # trim in blocks, not per point
            del self.pts[:1000]

    def since(self, epoch, since):
        if self.pts and epoch == self.epoch and since >= self.pts[0][-1] - 1:
            return False, tail(self.pts, since + 1, lambda q: q[-1])
        return True, list(self.pts)


class Gps:
    def __init__(self):
        self.track = Track()
        self.rx = self.bad = self.lost = 0
        self.last_seq = None
        self.reset()

    def reset(self):
        self.fixes = []
        self.origin = None
        self.t0 = None
        self.prev = None
        self.n = self.n_valid = 0
        self.track.clear()

    def en(self, lat, lon):
        lat0, lon0 = self.origin
        k = math.pi / 180 * R_EARTH
        return ((lon - lon0) * k * math.cos(math.radians(lat0)), (lat - lat0) * k)

    def add(self, fx):
        if self.t0 is None:
            self.t0 = fx['t']
        self.n += 1
        if fx['valid']:
            self.n_valid += 1
            if self.origin is None:
                self.origin = (fx['lat'], fx['lon'])
            e, n = self.en(fx['lat'], fx['lon'])
            p = self.prev
            brk = int(p is None or not p['valid'] or fx['t'] - p['t'] > 5.0)
            self.track.add([round(e, 3), round(n, 3), round(fx['t'] - self.t0, 1), fx['sats'],
                            fx['utc_ms'], brk])
        self.prev = fx
        self.fixes.append(fx)
        if len(self.fixes) > MAX_POINTS + 1000:
            del self.fixes[:1000]

    def average(self, t_end, window=10.0):
        """Mean of the valid fixes in (t_end - window, t_end] and their 2DRMS spread."""
        fx = [f for f in tail(self.fixes, t_end - window, lambda f: f['t']) if f['valid'] and f['t'] <= t_end]
        if len(fx) < 3:
            return None
        lat = sum(f['lat'] for f in fx) / len(fx)
        lon = sum(f['lon'] for f in fx) / len(fx)
        k = math.pi / 180 * R_EARTH
        c = math.cos(math.radians(lat))
        d2 = [((f['lon'] - lon) * k * c) ** 2 + ((f['lat'] - lat) * k) ** 2 for f in fx]
        return {'lat': lat, 'lon': lon, 'n': len(fx), 'spread_m': 2 * math.sqrt(sum(d2) / len(d2))}

    def snapshot(self, now, epoch, since):
        out = {'n': self.n, 'n_valid': self.n_valid, 'rx': self.rx, 'bad': self.bad, 'lost': self.lost,
               'origin': self.origin, 'epoch': self.track.epoch, 'full': True, 'last': None,
               'age_s': None, 'rate_hz': 0.0, 'track': [], 'series': [], 'recent': [], 'spread': None}
        fx = self.fixes
        if not fx:
            return out
        last = fx[-1]
        out['last'] = {k: last[k] for k in ('lat', 'lon', 'alt', 'hdop', 'speed', 'course', 'utc_ms',
                                            'q', 'sats', 'valid')}
        out['age_s'] = max(0.0, now - last['t'])
        recent = tail(fx, now - 10.0, lambda f: f['t'])
        if len(recent) > 1:
            out['rate_hz'] = (len(recent) - 1) / max(recent[-1]['t'] - recent[0]['t'], 1e-6)
        t0 = self.t0
        out['series'] = [[round(f['t'] - t0, 1), f['sats'], round(f['hdop'], 2), int(f['valid'])]
                         for f in tail(fx, last['t'] - 600.0, lambda f: f['t'])]
        out['recent'] = [{'t': round(f['t'] - t0, 1), 'utc_ms': f['utc_ms'], 'lat': f['lat'], 'lon': f['lon'],
                          'sats': f['sats'], 'hdop': f['hdop'], 'q': f['q'], 'valid': f['valid']}
                         for f in fx[-10:]][::-1]
        trk = self.track.pts
        if trk:
            pts = tail(trk, last['t'] - t0 - 120.0, lambda q: q[2])
            if len(pts) >= 10:
                me = sum(q[0] for q in pts) / len(pts)
                mn = sum(q[1] for q in pts) / len(pts)
                d = sorted(math.hypot(q[0] - me, q[1] - mn) for q in pts)
                drms = math.sqrt(sum(x * x for x in d) / len(d))
                out['spread'] = {'n': len(d), 'cep50': d[len(d) // 2], 'drms2': 2 * drms}
        out['full'], out['track'] = self.track.since(epoch, since)
        return out


class Pose:
    def __init__(self):
        self.track = Track()
        self.rx = self.bad = self.lost = 0
        self.last_seq = None
        self.reset()

    def reset(self):
        self.last = None
        self.t0 = None
        self.stored = None              # last point put in the track
        self.times = deque(maxlen=1000)
        self.track.clear()

    def add(self, d):
        """d: decoded pose with 't' (s, HLC monotonic for live data)."""
        if self.t0 is None:
            self.t0 = d['t']
        prev = self.last
        self.last = d
        self.times.append(d['t'])
        # a jump (RESET_POSE, LLC reset) or a long silence must not be drawn as motion
        brk = int(prev is None or d['t'] - prev['t'] > 2.0 or
                  math.hypot(d['x'] - prev['x'], d['y'] - prev['y']) > 0.5)
        s = self.stored
        if brk or s is None or math.hypot(d['x'] - s[0], d['y'] - s[1]) >= 0.01:
            p = [round(d['x'], 4), round(d['y'], 4), round(d['t'] - self.t0, 2), brk]
            self.track.add(p)
            self.stored = p

    def snapshot(self, now, epoch, since):
        out = {'rx': self.rx, 'bad': self.bad, 'lost': self.lost, 'last': None, 'age_s': None,
               'rate_hz': 0.0, 'epoch': self.track.epoch, 'full': True, 'track': []}
        d = self.last
        if d is None:
            return out
        out['last'] = {k: d[k] for k in ('x', 'y', 'theta', 'omega', 'bias', 's', 'delta', 'mode', 'status',
                                         'Pxx', 'Pyy')}
        out['last']['flags'] = [f for i, f in enumerate(FLAGS) if d['status'] >> i & 1]
        out['last']['mode_name'] = MODES[d['mode']] if d['mode'] < len(MODES) else str(d['mode'])
        out['age_s'] = max(0.0, now - d['t'])
        recent = [t for t in self.times if t >= now - 10.0]
        if len(recent) > 1:
            out['rate_hz'] = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)
        out['full'], out['track'] = self.track.since(epoch, since)
        return out


class Marks:
    """Test markers. Live ones go to a CSV next to the run folders; a replay derives
    them from the MARK events of the run."""
    COLS = ['id', 'boot_id', 't_mono_ns', 'wall_utc', 'label', 'gps_lat', 'gps_lon', 'gps_n',
            'gps_spread_m', 'pose_x', 'pose_y', 'pose_theta', 'pose_s', 'pose_valid', 'app_reply']

    def __init__(self, path):
        self.path = path
        self.boot = boot_id()
        self.rows = []                  # every row of the file (all boots)
        if path and os.path.exists(path):
            try:
                with open(path, newline='') as f:
                    self.rows = list(csv.DictReader(f))
            except OSError:
                pass
        self.next_id = max([int(r['id']) for r in self.rows if r.get('id', '').isdigit()] + [0]) + 1

    def current(self):
        return [r for r in self.rows if r.get('boot_id') == self.boot]

    def _write(self):
        """Returns None, or why the file could not be written (kept in memory then)."""
        if not self.path:
            return None
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            tmp = self.path + '.tmp'
            with open(tmp, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=self.COLS, extrasaction='ignore')
                w.writeheader()
                w.writerows(self.rows)
            os.replace(tmp, self.path)
            return None
        except OSError as e:
            return '%s: %s' % (self.path, e.strerror or e)

    def add(self, row):
        row['id'] = str(self.next_id)
        row['boot_id'] = self.boot
        self.next_id += 1
        self.rows.append(row)
        return self._write()

    def delete(self, mid):
        n = len(self.rows)
        self.rows = [r for r in self.rows if not (r.get('id') == str(mid) and r.get('boot_id') == self.boot)]
        if len(self.rows) != n:
            self._write()
            return True
        return False


def make_mark(label, t, gps, pose_d, t_mono_ns, wall, reply):
    avg = gps.average(t)
    row = {'t_mono_ns': str(t_mono_ns), 'wall_utc': wall, 'label': label, 'app_reply': reply,
           'gps_lat': '', 'gps_lon': '', 'gps_n': '0', 'gps_spread_m': '',
           'pose_x': '', 'pose_y': '', 'pose_theta': '', 'pose_s': '', 'pose_valid': ''}
    if avg:
        row.update({'gps_lat': '%.8f' % avg['lat'], 'gps_lon': '%.8f' % avg['lon'], 'gps_n': str(avg['n']),
                    'gps_spread_m': '%.2f' % avg['spread_m']})
    if pose_d is not None:
        row.update({'pose_x': '%.4f' % pose_d['x'], 'pose_y': '%.4f' % pose_d['y'],
                    'pose_theta': '%.5f' % pose_d['theta'], 'pose_s': '%.4f' % pose_d['s'],
                    'pose_valid': str(pose_d['status'] & 1)})
    return row


def mark_view(r, gps):
    """A mark as the page draws it (GPS east/north in the GPS track frame)."""
    def f(k):
        try:
            return float(r[k])
        except (KeyError, ValueError, TypeError):
            return None
    m = {'id': r.get('id'), 'label': r.get('label', ''), 'wall_utc': r.get('wall_utc', ''),
         'lat': f('gps_lat'), 'lon': f('gps_lon'), 'gps_n': int(f('gps_n') or 0), 'spread': f('gps_spread_m'),
         'x': f('pose_x'), 'y': f('pose_y'), 'theta': f('pose_theta'), 's': f('pose_s'),
         'pose_valid': r.get('pose_valid') == '1', 'e': None, 'n': None}
    if m['lat'] is not None and gps.origin is not None:
        m['e'], m['n'] = gps.en(m['lat'], m['lon'])
    return m


class App:
    def __init__(self, conf, live, source, marks_path):
        self.lock = threading.Lock()
        self.conf = conf
        self.live = live
        self.source = source
        self.gps = Gps()
        self.pose = Pose()
        self.stats = None
        self.stats_t = None
        self.ctrl_ok = False
        self.marks = Marks(marks_path if live else None)
        self.replay_marks = []
        self.t_end = 0.0                # replay: time of the last datagram

    def now(self):
        return time.monotonic() if self.live else self.t_end

    # -- control port ----------------------------------------------------
    def ctrl(self, cmd, timeout=1.0):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        try:
            s.sendto(cmd.encode(), ('127.0.0.1', self.conf['ctrl_port']))
            return s.recv(2048).decode(errors='replace').strip()
        except OSError:
            return None
        finally:
            s.close()

    def poll_stats(self):
        while True:
            reply = self.ctrl('STATS')
            with self.lock:
                self.ctrl_ok = reply is not None
                if reply and not reply.startswith('ERR'):
                    vals = reply.split(',')
                    if len(vals) == len(STATS_COLS):
                        self.stats = dict(zip(STATS_COLS, vals))
                        self.stats_t = time.monotonic()
            time.sleep(1.0)

    def stats_view(self):
        st = self.stats
        if st is None:
            return None

        def f(k):
            try:
                return float(st[k])
            except (KeyError, ValueError):
                return None

        def deliv(p):
            d, l = f(p + '_delivered'), f(p + '_lost')
            return None if d is None or d + l == 0 else 100.0 * d / (d + l)
        links = {p: {'pct': deliv(p), 'max_gap': f(p + '_max_gap'), 'drops': f(p + '_drops')}
                 for p in ('acq', 'fus', 'est')}
        agents = []
        for name, key in AGENTS:
            v = f(key)
            agents.append({'name': name, 'state': AGENT_MODES[int(v)] if v is not None and 0 <= v < 5 else '?'})
        return {'run_s': f('t_run_s'), 'rss_kb': f('rss_kb'), 'cpu_s': f('cpu_s'), 'poses': f('poses'),
                'lat_p50': f('lat_p50_ms'), 'lat_p95': f('lat_p95_ms'), 'lat_max': f('lat_max_ms'),
                'int_p95': f('interval_p95_ms'), 'clock_scale': f('llc_clock_scale'), 'links': links,
                'gps_fixes': f('gps_fixes'), 'gps_valid': f('gps_valid'), 'log_drops': f('log_drops'),
                'agents': agents, 'raw': st}

    # -- page state ------------------------------------------------------
    def snapshot(self, q):
        def qi(k):
            try:
                return int(q.get(k, ['0'])[0])
            except ValueError:
                return 0
        with self.lock:
            now = self.now()
            marks = [mark_view(r, self.gps) for r in (self.marks.current() if self.live else self.replay_marks)]
            return {'live': self.live, 'source': self.source, 'ctrl_ok': self.ctrl_ok if self.live else False,
                    'stats': self.stats_view(),
                    'stats_age_s': (time.monotonic() - self.stats_t) if self.live and self.stats_t else None,
                    'pose': self.pose.snapshot(now, qi('pe'), qi('ps')),
                    'gps': self.gps.snapshot(now, qi('ge'), qi('gs')),
                    'marks': marks, 'log_dir': self.conf['log_dir'], 'marks_file': self.marks.path,
                    'pose_frame': self.conf['pose_frame']}

    def mark(self, label):
        label = ' '.join(label.split())[:60] or 'mark'
        reply = self.ctrl('MARK ' + label) if self.live else None
        with self.lock:
            row = make_mark(label, self.now(), self.gps, self.pose.last, time.monotonic_ns(),
                            time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), reply or 'no reply')
            err = self.marks.add(row)
        return {'ok': err is None, 'app': reply, 'error': err}

    # -- UDP inputs -------------------------------------------------------
    def udp_pose(self, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('0.0.0.0', port))
        size = struct.calcsize(POSE_FMT)
        while True:
            data = s.recv(2048)
            f = struct.unpack(POSE_FMT, data) if len(data) == size else None
            ok = f is not None and f[0] == b'OPE1' and binascii.crc32(data[:124]) & 0xFFFFFFFF == f[-1]
            with self.lock:
                p = self.pose
                if not ok:
                    p.bad += 1
                    continue
                p.rx += 1
                if p.last_seq is not None and f[1] != p.last_seq + 1:
                    p.lost += (f[1] - p.last_seq - 1) & 0xFFFFFFFF
                p.last_seq = f[1]
                p.add({'t': time.monotonic(), 'mode': f[5], 'status': f[6], 'x': f[7], 'y': f[8], 'theta': f[9],
                       'omega': f[10], 'bias': f[11], 'Pxx': f[12], 'Pyy': f[13], 's': f[16], 'delta': f[17]})

    def udp_gps(self, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('0.0.0.0', port))
        size = struct.calcsize(GPS_FMT)
        while True:
            data = s.recv(2048)
            f = struct.unpack(GPS_FMT, data) if len(data) == size else None
            ok = f is not None and f[0] == b'OGP1' and binascii.crc32(data[:76]) & 0xFFFFFFFF == f[-1]
            with self.lock:
                g = self.gps
                if not ok:
                    g.bad += 1
                    continue
                g.rx += 1
                if g.last_seq is not None and f[1] != g.last_seq + 1:
                    g.lost += (f[1] - g.last_seq - 1) & 0xFFFFFFFF
                g.last_seq = f[1]
                g.add({'t': time.monotonic(), 'lat': f[3], 'lon': f[4], 'alt': f[5], 'hdop': f[6], 'speed': f[7],
                       'course': f[8], 'utc_ms': f[9], 'q': f[10], 'sats': f[11], 'valid': bool(f[12])})


# ------------------------------------------------------------------ replay
def read_csv(path):
    try:
        with open(path, newline='') as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def gps_from_rows(rows, gps):
    for r in rows:
        try:
            gps.add({'t': int(r['t_rx_ns']) * 1e-9, 'utc_ms': int(r['utc_ms']), 'lat': float(r['lat_deg']),
                     'lon': float(r['lon_deg']), 'alt': float(r['alt_m']), 'q': int(r['quality']),
                     'sats': int(r['sats']), 'hdop': float(r['hdop']), 'speed': float(r['speed_mps']),
                     'course': float(r['course_deg']), 'valid': r['valid'].strip() == '1'})
            gps.rx += 1
        except (KeyError, ValueError):
            continue


def load_run(app, run):
    gps_from_rows(read_csv(os.path.join(run, 'gps.csv')), app.gps)
    poses = []
    for r in read_csv(os.path.join(run, 'pose.csv')):
        try:
            d = {'t': int(r['t_rx_ns']) * 1e-9, 'mode': int(r['mode']), 'status': int(r['status']),
                 'x': float(r['x_m']), 'y': float(r['y_m']), 'theta': float(r['theta_rad']),
                 'omega': float(r['omega_rad_s']), 'bias': float(r['bias_rad_s']), 'Pxx': float(r['P_xx']),
                 'Pyy': float(r['P_yy']), 's': float(r['s_m']), 'delta': float(r['delta'])}
        except (KeyError, ValueError):
            continue
        app.pose.add(d)
        app.pose.rx += 1
        poses.append(d)
    stats = read_csv(os.path.join(run, 'stats.csv'))
    if stats:
        app.stats = {k: stats[-1].get(k, '') for k in STATS_COLS}
    ends = [x for x in (poses[-1]['t'] if poses else None, app.gps.fixes[-1]['t'] if app.gps.fixes else None)
            if x is not None]
    app.t_end = max(ends) if ends else 0.0
    # test marks from the MARK events of the run, positioned with the same rule as live
    k = 0
    for r in read_csv(os.path.join(run, 'events.csv')):
        ev = r.get('event') or ''
        if r.get('agent') != 'operator' or not ev.startswith('MARK'):
            continue
        try:
            t = int(r['t_ns']) * 1e-9
        except (KeyError, ValueError):
            continue
        before = tail(poses, -1e18, lambda d: d['t'])
        pd = None
        for d in reversed(before):
            if d['t'] <= t:
                pd = d
                break
        k += 1
        row = make_mark(ev[4:].strip(), t, app.gps, pd, r['t_ns'], '', 'from events.csv')
        row['id'] = str(k)
        app.replay_marks.append(row)
    return len(poses), len(app.gps.fixes)


# --------------------------------------------------------------------- GPX
def run_utc_date(run_name):
    """UTC date of a run folder named YYYYmmdd-HHMMSS in the rover's local time."""
    try:
        t = time.mktime(time.strptime(run_name, '%Y%m%d-%H%M%S'))
    except ValueError:
        t = time.time()
    g = time.gmtime(t)
    return g, (g.tm_hour * 3600 + g.tm_min * 60 + g.tm_sec) * 1000


def gpx_text(name, fixes, marks, date):
    """date = (struct_time of the start, UTC ms of day at the start) or None."""
    def stamp(ms):
        if date is None:
            return ''
        start, start_ms = date
        day = time.mktime(start[:3] + (12, 0, 0, 0, 0, -1)) + (86400 if ms < start_ms - 3600000 else 0)
        d = time.localtime(day)
        return '<time>%04d-%02d-%02dT%02d:%02d:%06.3fZ</time>' % (d.tm_year, d.tm_mon, d.tm_mday, ms // 3600000,
                                                                 ms // 60000 % 60, ms % 60000 / 1000)
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<gpx version="1.1" creator="olympus-pose_gui.py" xmlns="http://www.topografix.com/GPX/1/1">']
    for m in marks:
        if m.get('gps_lat'):
            lab = (m.get('label') or '').replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
            out.append('<wpt lat="%s" lon="%s"><name>%s</name><desc>average of %s fixes, 2DRMS %s m</desc></wpt>'
                       % (m['gps_lat'], m['gps_lon'], lab, m.get('gps_n', ''), m.get('gps_spread_m', '')))
    out.append('<trk><name>%s</name><trkseg>' % name)
    prev = None
    for p in fixes:
        if not p['valid']:
            prev = p
            continue
        if prev is not None and not prev['valid']:
            out.append('</trkseg><trkseg>')         # a fix loss starts a new segment
        prev = p
        out.append('<trkpt lat="%.8f" lon="%.8f"><ele>%.2f</ele>%s<sat>%d</sat><hdop>%.2f</hdop></trkpt>'
                   % (p['lat'], p['lon'], p['alt'], stamp(p['utc_ms']), p['sats'], p['hdop']))
    out.append('</trkseg></trk>\n</gpx>\n')
    return '\n'.join(out)


def run_gpx(run_dir):
    tmp = App({'log_dir': '', 'pose_port': 0, 'gps_port': 0, 'ctrl_port': 0}, False, run_dir, None)
    load_run(tmp, run_dir)
    name = os.path.basename(os.path.normpath(run_dir))
    return gpx_text(name, tmp.gps.fixes, tmp.replay_marks, run_utc_date(name))


# -------------------------------------------------------------------- HTTP
def list_runs(log_dir):
    runs = []
    try:
        names = sorted((n for n in os.listdir(log_dir) if os.path.isdir(os.path.join(log_dir, n))), reverse=True)
    except OSError:
        return runs
    for n in names[:60]:
        d = os.path.join(log_dir, n)
        files = []
        for fn in sorted(os.listdir(d)):
            p = os.path.join(d, fn)
            if fn.endswith(RUN_FILES) and os.path.isfile(p):
                files.append({'name': fn, 'size': os.path.getsize(p)})
        runs.append({'name': n, 'files': files})
    return runs


def make_handler(app):
    class H(BaseHTTPRequestHandler):
        def _send(self, code, ctype, body, filename=None):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            if filename:
                self.send_header('Content-Disposition', 'attachment; filename="%s"' % filename)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, 'application/json', json.dumps(obj, separators=(',', ':')).encode())

        def _body(self):
            try:
                n = int(self.headers.get('Content-Length', '0'))
                return json.loads(self.rfile.read(min(n, 4096)) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return {}

        def do_GET(self):
            u = urlsplit(self.path)
            q = parse_qs(u.query)
            if u.path == '/':
                self._send(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
            elif u.path == '/api/state':
                self._json(app.snapshot(q))
            elif u.path == '/api/runs':
                self._json({'log_dir': app.conf['log_dir'], 'runs': list_runs(app.conf['log_dir']),
                            'marks_file': app.marks.path if app.live and app.marks.path
                            and os.path.exists(app.marks.path) else None})
            elif u.path == '/api/gpx':
                run = q.get('run', [''])[0]
                if run:
                    d = os.path.join(app.conf['log_dir'], run)
                    if run not in os.listdir(app.conf['log_dir']) or not os.path.isdir(d):
                        return self._send(404, 'text/plain', b'no such run')
                    body, name = run_gpx(d), run
                else:
                    with app.lock:
                        rows = app.marks.current() if app.live else app.replay_marks
                        body = gpx_text('olympus-pose session', list(app.gps.fixes), rows,
                                        run_utc_date(time.strftime('%Y%m%d-%H%M%S')) if app.live else None)
                    name = 'session'
                self._send(200, 'application/gpx+xml', body.encode(), name + '.gpx')
            elif u.path.startswith('/files/'):
                self._file(u.path[len('/files/'):])
            else:
                self._send(404, 'text/plain', b'not found')

        def _file(self, rel):
            parts = [unquote(p) for p in rel.split('/')]
            log_dir = app.conf['log_dir']
            try:
                if len(parts) == 1 and app.live and app.marks.path and parts[0] == os.path.basename(app.marks.path):
                    path = app.marks.path
                elif len(parts) == 2 and parts[0] in os.listdir(log_dir) and \
                        parts[1] in os.listdir(os.path.join(log_dir, parts[0])) and parts[1].endswith(RUN_FILES):
                    path = os.path.join(log_dir, parts[0], parts[1])
                else:
                    raise OSError
                size = os.path.getsize(path)
            except OSError:
                return self._send(404, 'text/plain', b'not found')
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(size))
            self.send_header('Content-Disposition', 'attachment; filename="%s"' % (
                parts[0] + '_' + parts[1] if len(parts) == 2 else parts[0]))
            self.end_headers()
            with open(path, 'rb') as f:
                shutil.copyfileobj(f, self.wfile)

        def do_POST(self):
            u = urlsplit(self.path)
            b = self._body()
            if not app.live:
                return self._json({'ok': False, 'error': 'replay: controls are off'}, 409)
            if u.path == '/api/ctrl':
                cmd = str(b.get('cmd', ''))
                word = cmd.split(' ', 1)[0]
                if word not in ('RESET_POSE', 'SUSPEND', 'RESUME', 'STATS'):
                    return self._json({'ok': False, 'error': 'command not allowed'}, 400)
                reply = app.ctrl(cmd)
                if word == 'RESET_POSE' and reply and reply.startswith('OK'):
                    with app.lock:
                        app.pose.track.clear()
                        app.pose.stored = None
                return self._json({'ok': bool(reply) and reply.startswith('OK'), 'reply': reply or
                                   'no reply from olympus-pose on 127.0.0.1:%d' % app.conf['ctrl_port']})
            if u.path == '/api/mark':
                return self._json(app.mark(str(b.get('label', ''))))
            if u.path == '/api/mark_delete':
                with app.lock:
                    return self._json({'ok': app.marks.delete(b.get('id'))})
            if u.path == '/api/clear':
                with app.lock:
                    app.gps.reset()
                    app.pose.track.clear()
                    app.pose.stored = None
                return self._json({'ok': True})
            self._send(404, 'text/plain', b'not found')

        def log_message(self, fmt, *args):
            pass
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-c', '--conf', default='/etc/olympus-pose/pose.conf', help='ports and log folder')
    ap.add_argument('--http', default='0.0.0.0:8080', help='address:port of the web page')
    ap.add_argument('--log-dir', help='run folders (default: io.log_dir of the configuration)')
    ap.add_argument('--marks', help='live test marks file (default: gui_marks.csv next to the run folders)')
    ap.add_argument('--run', help='replay a logged run folder instead of live data')
    ap.add_argument('--gpx', help='with --run: write its GPS track and test marks as GPX and exit')
    a = ap.parse_args()

    conf = load_conf(a.conf)
    if a.log_dir:
        conf['log_dir'] = a.log_dir
    if a.run:
        run = os.path.normpath(a.run)
        if a.gpx:
            with open(a.gpx, 'w', encoding='utf-8') as f:
                f.write(run_gpx(run))
            print('olympus-pose_gui: %s -> %s' % (run, a.gpx))
            return
        conf['log_dir'] = a.log_dir or os.path.dirname(run)
        # the run's own frame, from the parameter dump in its header.txt
        hdr = load_conf(os.path.join(run, 'header.txt'))
        conf['pose_frame'] = hdr['pose_frame']
        app = App(conf, False, os.path.basename(run), None)
        n_pose, n_gps = load_run(app, run)
        print('olympus-pose_gui: replay of %s: %d poses, %d GPS fixes, %d marks'
              % (run, n_pose, n_gps, len(app.replay_marks)))
    elif a.gpx:
        sys.exit('olympus-pose_gui: --gpx needs --run')
    else:
        marks = a.marks or os.path.join(os.path.dirname(os.path.normpath(conf['log_dir'])), 'gui_marks.csv')
        app = App(conf, True, 'live', marks)
        for target, port in ((app.udp_pose, conf['pose_port']), (app.udp_gps, conf['gps_port'])):
            threading.Thread(target=target, args=(port,), daemon=True).start()
        threading.Thread(target=app.poll_stats, daemon=True).start()
    host, _, port = a.http.rpartition(':')
    srv = ThreadingHTTPServer((host or '0.0.0.0', int(port)), make_handler(app))
    print('olympus-pose_gui: open http://%s:%s/ (Ctrl-C to stop)'
          % (host if host not in ('', '0.0.0.0') else '<rover-ip>', port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


PAGE = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Olympus Pose</title>
<style>
:root {
  color-scheme: light;
  --bg: #f4f4f1; --surface: #fcfcfb; --rule: #e2e1dc; --grid: #ebeae6;
  --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #7a7974;
  --series-1: #2a78d6; --series-2: #eb6834;
  --good: #0ca30c; --warning: #fab219; --critical: #d03b3b;
  --mono: ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --bg: #111110; --surface: #1a1a19; --rule: #33332f; --grid: #262624;
    --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #8f8e86;
    --series-1: #3987e5; --series-2: #d95926;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --bg: #111110; --surface: #1a1a19; --rule: #33332f; --grid: #262624;
  --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #8f8e86;
  --series-1: #3987e5; --series-2: #d95926;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text-primary);
  font: 15px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; padding: 16px; }
.wrap { max-width: 1180px; margin: 0 auto; }
header { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 16px; margin-bottom: 12px; }
h1 { font-size: 22px; margin: 0; }
.src { color: var(--text-secondary); font-family: var(--mono); font-size: 13px; }
.conn { margin-left: auto; font-size: 13px; color: var(--text-secondary); display: inline-flex; gap: 6px; align-items: center; }
nav.tabs { display: flex; gap: 4px; border-bottom: 1px solid var(--rule); margin-bottom: 14px; overflow-x: auto; }
nav.tabs button { border: 0; border-bottom: 2px solid transparent; border-radius: 0; background: none;
  padding: 8px 14px; font-weight: 600; color: var(--text-secondary); }
nav.tabs button[aria-selected="true"] { color: var(--text-primary); border-bottom-color: var(--series-1); }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(175px, 1fr)); gap: 10px; margin-bottom: 14px; }
.tile { background: var(--surface); border: 1px solid var(--rule); border-radius: 8px; padding: 10px 12px; min-width: 0; }
.tile .k { color: var(--text-secondary); font-size: 12.5px; }
.tile .v { font-size: 22px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 2px; overflow-wrap: anywhere; }
.tile .s { color: var(--text-muted); font-size: 12.5px; margin-top: 2px; overflow-wrap: anywhere; }
.status { display: inline-flex; align-items: center; gap: 8px; }
.dot { width: 14px; height: 14px; border-radius: 50%; flex: none; display: inline-grid; place-items: center;
  color: #fff; font-size: 10px; font-weight: 700; line-height: 1; }
.chips { display: flex; flex-wrap: wrap; gap: 6px; margin: -4px 0 14px; }
.chip { font-size: 12.5px; padding: 2px 8px; border-radius: 999px; border: 1px solid var(--rule);
  background: var(--surface); color: var(--text-secondary); }
.chip.on { color: var(--text-primary); font-weight: 600; }
.grid2 { display: grid; grid-template-columns: minmax(0, 1fr); gap: 14px; }
@media (min-width: 900px) { .grid2 { grid-template-columns: minmax(0, 1.25fr) minmax(0, 1fr); } }
.card { background: var(--surface); border: 1px solid var(--rule); border-radius: 8px; padding: 12px 14px; min-width: 0; position: relative; margin-bottom: 14px; }
.grid2 > .card, .side > .card { margin-bottom: 0; }
.card h2 { font-size: 15px; margin: 0 0 2px; }
.card .sub { color: var(--text-muted); font-size: 12.5px; margin: 0 0 8px; overflow-wrap: anywhere; }
.card h2 + .sub { padding-right: 96px; }
.card .acts { position: absolute; top: 10px; right: 12px; display: flex; gap: 6px; }
svg { display: block; width: 100%; height: auto; }
svg text { fill: var(--text-muted); font-size: 11px; font-family: system-ui, sans-serif; }
svg text.lab { fill: var(--text-primary); font-size: 12px; font-weight: 600; }
.gl { stroke: var(--grid); stroke-width: 1; }
.ax { stroke: var(--rule); stroke-width: 1; fill: none; }
.trk { fill: none; stroke: var(--series-1); stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }
.ln { fill: none; stroke: var(--series-1); stroke-width: 2; stroke-linejoin: round; }
.cur { fill: var(--series-1); stroke: var(--surface); stroke-width: 2; }
.head { stroke: var(--series-1); stroke-width: 2.5; stroke-linecap: round; }
.start { fill: var(--surface); stroke: var(--text-secondary); stroke-width: 2; }
.pin { fill: var(--series-2); stroke: var(--surface); stroke-width: 2; }
.pinring { fill: none; stroke: var(--series-2); stroke-width: 1; stroke-dasharray: 3 3; }
.hov { fill: none; stroke: var(--text-primary); stroke-width: 1.5; }
.xh { stroke: var(--text-muted); stroke-width: 1; stroke-dasharray: 3 3; }
.nofix { fill: var(--grid); }
.tip { position: absolute; pointer-events: none; background: var(--surface); color: var(--text-primary);
  border: 1px solid var(--rule); border-radius: 6px; padding: 6px 8px; font-size: 12.5px; white-space: nowrap;
  box-shadow: 0 2px 8px rgba(0,0,0,.12); display: none; z-index: 2; font-variant-numeric: tabular-nums; }
.side { display: grid; gap: 14px; align-content: start; }
table { border-collapse: collapse; width: 100%; font-size: 13px; font-variant-numeric: tabular-nums; }
th, td { text-align: right; padding: 5px 8px; border-bottom: 1px solid var(--rule); white-space: nowrap; }
th { color: var(--text-secondary); font-weight: 600; font-size: 12px; }
th:first-child, td:first-child, td.l, th.l { text-align: left; }
.tw { overflow-x: auto; }
.row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
.note { color: var(--text-muted); font-size: 13px; margin: 8px 0 0; max-width: 80ch; }
a { color: var(--series-1); }
button, input { font: inherit; font-size: 13.5px; padding: 6px 12px; border-radius: 6px; border: 1px solid var(--rule);
  background: var(--surface); color: var(--text-primary); }
button { cursor: pointer; }
button.primary { background: var(--series-1); border-color: var(--series-1); color: #fff; font-weight: 600; }
button.small { padding: 3px 8px; font-size: 12.5px; }
button:disabled { opacity: .5; cursor: not-allowed; }
input[type=text] { min-width: 0; flex: 1 1 220px; }
input[type=number] { width: 90px; }
button:focus-visible, input:focus-visible { outline: 2px solid var(--series-1); outline-offset: 2px; }
.msg { font-size: 13px; color: var(--text-secondary); font-family: var(--mono); overflow-wrap: anywhere; }
details summary { cursor: pointer; color: var(--text-secondary); font-size: 13px; }
[hidden] { display: none !important; }
</style>
</head>
<body>
<div class="wrap">
<header>
  <h1>Olympus Pose</h1>
  <span class="src" id="src">connecting…</span>
  <span class="conn" id="conn"></span>
</header>
<nav class="tabs" role="tablist">
  <button role="tab" data-tab="pose" aria-selected="true">Pose</button>
  <button role="tab" data-tab="gps" aria-selected="false">GPS</button>
  <button role="tab" data-tab="marks" aria-selected="false">Markers</button>
  <button role="tab" data-tab="ctrl" aria-selected="false">Controls &amp; files</button>
</nav>

<!-- ================================================================ POSE -->
<section id="tab-pose">
  <div class="tiles">
    <div class="tile"><div class="k">Estimator</div><div class="v status" id="p-mode">—</div><div class="s" id="p-mode-s"></div></div>
    <div class="tile"><div class="k">Position x, y</div><div class="v" id="p-xy">—</div><div class="s" id="p-sig"></div></div>
    <div class="tile"><div class="k">Heading θ</div><div class="v" id="p-th">—</div><div class="s" id="p-om"></div></div>
    <div class="tile"><div class="k">Distance travelled</div><div class="v" id="p-s">—</div><div class="s">encoder odometry</div></div>
    <div class="tile"><div class="k">Pose rate</div><div class="v" id="p-rate">—</div><div class="s" id="p-age"></div></div>
    <div class="tile"><div class="k">Latency p95 (OE5)</div><div class="v" id="p-lat">—</div><div class="s" id="p-lat-s"></div></div>
    <div class="tile"><div class="k">Delivery (OE4)</div><div class="v" id="p-del">—</div><div class="s" id="p-del-s"></div></div>
    <div class="tile"><div class="k">LLC clock scale</div><div class="v" id="p-clk">—</div><div class="s">1.00 = LLC tick counts real time</div></div>
  </div>
  <div class="chips" id="p-flags"></div>
  <div class="card" id="c-ptrk">
    <h2>Pose track</h2>
    <p class="sub" id="ptrk-sub">Metres in the pose frame. ○ start, ● now with heading; markers in orange.</p>
    <div class="acts"><button class="small" data-fig="ptrk">SVG</button><button class="small" data-fig="ptrk" data-png="1">PNG</button></div>
    <svg id="ptrk" viewBox="0 0 900 600" role="img" aria-label="Pose track in metres"></svg>
    <div class="tip" id="tip-ptrk"></div>
  </div>
</section>

<!-- ================================================================= GPS -->
<section id="tab-gps" hidden>
  <div class="tiles">
    <div class="tile"><div class="k">Fix</div><div class="v status" id="t-fix">—</div><div class="s" id="t-fix-s"></div></div>
    <div class="tile"><div class="k">Satellites</div><div class="v" id="t-sats">—</div><div class="s">in the solution</div></div>
    <div class="tile"><div class="k">HDOP</div><div class="v" id="t-hdop">—</div><div class="s" id="t-hdop-s"></div></div>
    <div class="tile"><div class="k">Fix rate (OE2)</div><div class="v" id="t-rate">—</div><div class="s" id="t-age"></div></div>
    <div class="tile"><div class="k">Valid fixes</div><div class="v" id="t-valid">—</div><div class="s" id="t-rx"></div></div>
    <div class="tile"><div class="k">Scatter, last 2 min (rover still)</div><div class="v" id="t-cep">—</div><div class="s" id="t-cep-s">needs ≥ 10 valid fixes</div></div>
  </div>
  <div class="grid2">
    <div class="card" id="c-gtrk">
      <h2>GPS track</h2>
      <p class="sub">East/north metres from the first valid fix. ○ start, ● latest; breaks where the fix was lost.</p>
      <div class="acts"><button class="small" data-fig="gtrk">SVG</button><button class="small" data-fig="gtrk" data-png="1">PNG</button></div>
      <svg id="gtrk" viewBox="0 0 600 600" role="img" aria-label="GPS track in metres"></svg>
      <div class="tip" id="tip-gtrk"></div>
    </div>
    <div class="side">
      <div class="card" id="c-sats">
        <h2>Satellites</h2>
        <p class="sub">Last 10 min. Grey bands: no valid fix.</p>
        <svg id="sats" viewBox="0 0 520 170" role="img" aria-label="Satellites over time"></svg>
        <div class="tip" id="tip-sats"></div>
      </div>
      <div class="card" id="c-hdop">
        <h2>HDOP</h2>
        <p class="sub">Last 10 min. Lower is better; below 2 is good.</p>
        <svg id="hdop" viewBox="0 0 520 170" role="img" aria-label="HDOP over time"></svg>
        <div class="tip" id="tip-hdop"></div>
      </div>
    </div>
  </div>
  <div class="card" style="margin-top:14px">
    <h2>Latest fixes</h2>
    <div class="tw"><table>
      <thead><tr><th>UTC</th><th>t (s)</th><th>Latitude</th><th>Longitude</th><th>Sats</th><th>HDOP</th><th>Quality</th><th>Valid</th></tr></thead>
      <tbody id="tbl"></tbody>
    </table></div>
    <p class="note" id="pos"></p>
  </div>
</section>

<!-- ============================================================= MARKERS -->
<section id="tab-marks" hidden>
  <div class="card">
    <h2>Mark a test point</h2>
    <p class="sub">Stop the rover first. A mark stores the average of the last 10 s of GPS fixes, the pose at that moment, and a MARK event in the olympus-pose log.</p>
    <form class="row" id="mark-form">
      <input type="text" id="mark-label" maxlength="60" placeholder="e.g. UMBmark CW run 1 start" aria-label="Marker label">
      <button class="primary" type="submit" id="mark-btn">Mark</button>
      <span class="msg" id="mark-msg"></span>
    </form>
  </div>
  <div class="card">
    <h2>Markers</h2>
    <p class="sub" id="marks-sub"></p>
    <div class="tw"><table>
      <thead><tr><th>#</th><th class="l">Label</th><th>UTC</th><th>GPS lat</th><th>GPS lon</th><th>fixes</th><th>2DRMS (m)</th>
        <th>pose x</th><th>pose y</th><th>θ (°)</th><th>Δ GPS (m)</th><th>Δ pose (m)</th><th></th></tr></thead>
      <tbody id="marks-tbl"></tbody>
    </table></div>
    <p class="note">Δ = straight distance from the previous marker. GPS scatters by a few metres, about the size of the tests:
      the GPS marks show where each test was done; distances are measured with the tape (OE6). The 10 s average removes
      the fast GPS noise but not its slow drift, so 2DRMS (the scatter of the averaged fixes) is shown, not a smaller "error of the mean".</p>
  </div>
</section>

<!-- ========================================================== CONTROLS -->
<section id="tab-ctrl" hidden>
  <div class="grid2">
    <div class="card">
      <h2>Estimator</h2>
      <p class="sub">Reset the pose to given values (default: origin). The tracks restart.</p>
      <div class="row">
        <label>x <input type="number" step="0.01" id="rx" value="0"></label>
        <label>y <input type="number" step="0.01" id="ry" value="0"></label>
        <label>θ (°) <input type="number" step="1" id="rt" value="0"></label>
        <button id="reset-btn" class="ctl">Reset pose</button>
      </div>
      <p class="msg" id="ctrl-msg"></p>
      <h2 style="margin-top:14px">CMAES agents</h2>
      <p class="sub">Suspend and resume go through the platform's AMS, as in the thesis. Communication cannot suspend itself.</p>
      <div class="tw"><table>
        <thead><tr><th class="l">Agent</th><th class="l">State</th><th></th></tr></thead>
        <tbody id="agents"></tbody>
      </table></div>
    </div>
    <div class="side">
      <div class="card">
        <h2>Run statistics</h2>
        <p class="sub" id="stats-sub">From the app's STATS command, once per second.</p>
        <div class="tw"><table><tbody id="stats-tbl"></tbody></table></div>
        <details style="margin-top:8px"><summary>Raw STATS line</summary><p class="msg" id="stats-raw"></p></details>
      </div>
      <div class="card">
        <h2>Session</h2>
        <div class="row">
          <button id="clear-btn" class="ctl">Clear tracks</button>
          <a id="gpx-session" href="/api/gpx">Download GPX of this session</a>
        </div>
        <p class="note">Clear tracks only clears this page; the logs and markers are kept.</p>
      </div>
    </div>
  </div>
  <div class="card" style="margin-top:14px">
    <h2>Logged runs</h2>
    <p class="sub" id="runs-sub"></p>
    <div class="tw"><table>
      <thead><tr><th class="l">Run</th><th class="l">Files</th><th class="l">GPX</th></tr></thead>
      <tbody id="runs"></tbody>
    </table></div>
    <button class="small" id="runs-btn" style="margin-top:8px">Refresh list</button>
  </div>
</section>
</div>

<script>
const QUAL = {0:'no fix',1:'GPS',2:'DGPS',3:'PPS',4:'RTK fixed',5:'RTK float',6:'estimated',7:'manual',8:'simulation'};
const $ = id => document.getElementById(id);
// HTML[id] = '...' rewrites an element only when its content changed, so a button
// is never replaced between the press and the release of a click.
const HTML = new Proxy({}, {set(_, id, html) {
  const e = $(id);
  if (e.__html !== html) { e.innerHTML = html; e.__html = html; }
  return true;
}});
const NS = 'http://www.w3.org/2000/svg';
let S = null, TAB = 'pose';
const PT = {track: [], epoch: 0, last: 0}, GT = {track: [], epoch: 0, last: 0};
const MAX_TRACK = 20000;

function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function utc(ms) {
  if (ms == null) return '—';
  const h = Math.floor(ms / 3600000), m = Math.floor(ms / 60000) % 60, s = (ms % 60000) / 1000;
  return String(h).padStart(2,'0') + ':' + String(m).padStart(2,'0') + ':' + s.toFixed(1).padStart(4,'0');
}
function nice(span, target) {
  const raw = span / target, p = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5, 10]) if (m * p >= raw) return m * p;
  return 10 * p;
}
function fmtStep(v, step) { const s = v.toFixed(step >= 1 ? 0 : step >= 0.1 ? 1 : 2); return Number(s) === 0 ? '0' : s; }
function fmtM(v) { return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2); }
function deg(r) { return r * 180 / Math.PI; }
function dot(cls, icon) { return '<span class="dot" style="background:var(--' + cls + ')">' + icon + '</span>'; }

// ------------------------------------------------------------- tabs
function showTab(t) {
  TAB = t;
  for (const b of document.querySelectorAll('nav.tabs button')) b.setAttribute('aria-selected', b.dataset.tab === t);
  for (const s of document.querySelectorAll('section')) s.hidden = s.id !== 'tab-' + t;
  try { localStorage.setItem('opg-tab', t); } catch (e) {}
  if (t === 'ctrl') loadRuns();
  if (S) render();
}
for (const b of document.querySelectorAll('nav.tabs button')) b.addEventListener('click', () => showTab(b.dataset.tab));

// --------------------------------------------- shared plot: equal-aspect metres
function planPlot(svg, W, H, pts, extra, minSpan) {
  const m = {l: 52, r: 16, t: 14, b: 40};
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  const xs = pts.map(p => p[0]).concat(extra.map(p => p[0])), ys = pts.map(p => p[1]).concat(extra.map(p => p[1]));
  let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const k = Math.min(pw / Math.max((x1 - x0) * 1.15, minSpan), ph / Math.max((y1 - y0) * 1.15, minSpan));
  const xc = (x0 + x1) / 2, yc = (y0 + y1) / 2;
  const X = v => m.l + pw / 2 + (v - xc) * k, Y = v => m.t + ph / 2 - (v - yc) * k;
  const step = nice(Math.max(pw, ph) / k, 7);
  for (let v = Math.ceil((xc - pw / 2 / k) / step) * step; v <= xc + pw / 2 / k; v += step) {
    el('line', {x1: X(v), x2: X(v), y1: m.t, y2: m.t + ph, class: 'gl'}, svg);
    el('text', {x: X(v), y: H - m.b + 16, 'text-anchor': 'middle'}, svg).textContent = fmtStep(v, step);
  }
  for (let v = Math.ceil((yc - ph / 2 / k) / step) * step; v <= yc + ph / 2 / k; v += step) {
    el('line', {x1: m.l, x2: m.l + pw, y1: Y(v), y2: Y(v), class: 'gl'}, svg);
    el('text', {x: m.l - 6, y: Y(v) + 4, 'text-anchor': 'end'}, svg).textContent = fmtStep(v, step);
  }
  el('rect', {x: m.l, y: m.t, width: pw, height: ph, class: 'ax'}, svg);
  return {X, Y, m, pw, ph, step, k};
}
function polySegments(svg, pts, X, Y, brkIdx) {
  let seg = [];
  const flush = () => { if (seg.length > 1) el('polyline', {points: seg.join(' '), class: 'trk'}, svg); seg = []; };
  for (const p of pts) { if (p[brkIdx]) flush(); seg.push(X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1)); }
  flush();
}
function drawPins(svg, list, P) {
  for (const mk of list) {
    const x = P.X(mk.a), y = P.Y(mk.b), right = x > P.m.l + P.pw * 0.6;   // labels stay inside the plot
    if (mk.ring) el('circle', {cx: x, cy: y, r: Math.max(mk.ring * P.k / 2, 6), class: 'pinring'}, svg);
    el('circle', {cx: x, cy: y, r: 6, class: 'pin'}, svg);
    el('text', {x: right ? x - 9 : x + 9, y: y - 8, class: 'lab', 'text-anchor': right ? 'end' : 'start'}, svg).textContent = mk.id + ' ' + mk.label;
  }
}

// ------------------------------------------------------------- pose view
let PTRK = null;
function renderPose(s) {
  const p = s.pose, L = p.last, st = s.stats;
  if (!L) {
    HTML['p-mode'] = dot('critical', '!') + 'No data';
    $('p-mode-s').textContent = s.live ? 'no pose datagrams (is olympus-pose running?)' : 'no poses in this run';
  } else {
    const stale = s.live && p.age_s > 1.0;
    const [cls, ic, label] = stale ? ['critical', '!', 'Stale'] : L.mode === 2 ? ['good', '✓', 'Running'] :
      L.mode === 1 ? ['warning', '…', 'Initializing'] : ['warning', '…', 'Waiting'];
    HTML['p-mode'] = dot(cls, ic) + label;
    $('p-mode-s').textContent = stale ? 'last pose ' + p.age_s.toFixed(1) + ' s ago' :
      L.mode === 1 ? 'keep the rover still (gyro bias)' : (L.flags.includes('VALID') ? 'pose valid' : 'pose not yet valid');
    $('p-xy').textContent = L.x.toFixed(3) + ', ' + L.y.toFixed(3) + ' m';
    $('p-sig').textContent = 'σ ' + Math.sqrt(Math.max(L.Pxx + L.Pyy, 0)).toFixed(3) + ' m';
    $('p-th').textContent = deg(L.theta).toFixed(1) + '°';
    $('p-om').textContent = 'ω ' + deg(L.omega).toFixed(1) + ' °/s · bias ' + deg(L.bias).toFixed(2) + ' °/s';
    $('p-s').textContent = L.s.toFixed(3) + ' m';
    const all = ['STILL', 'SLIP', 'STALE', 'GAP', 'IMU_INVALID', 'ROLLOVER', 'LLC_RESET', 'REREF'];
    HTML['p-flags'] = all.map(f => '<span class="chip' + (L.flags.includes(f) ? ' on' : '') + '">' +
      (L.flags.includes(f) ? '● ' : '○ ') + f + '</span>').join('');
  }
  $('p-rate').textContent = p.rate_hz ? p.rate_hz.toFixed(1) + ' Hz' : '—';
  $('p-age').textContent = s.live ? 'received ' + p.rx + ', lost ' + p.lost + ', bad ' + p.bad : 'poses in the log';
  if (st) {
    $('p-lat').textContent = st.lat_p95 != null ? st.lat_p95.toFixed(1) + ' ms' : '—';
    $('p-lat-s').textContent = 'p50 ' + (st.lat_p50 ?? 0).toFixed(1) + ' · max ' + (st.lat_max ?? 0).toFixed(1) + ' ms (req < 100)';
    const ls = st.links, pcts = ['acq', 'fus', 'est'].map(k => ls[k].pct).filter(v => v != null);
    $('p-del').textContent = pcts.length ? Math.min(...pcts).toFixed(2) + ' %' : '—';
    $('p-del-s').textContent = 'worst link; max gap ' + Math.max(...['acq', 'fus', 'est'].map(k => ls[k].max_gap || 0)) + ' (req ≥ 99 %, ≤ 4)';
    $('p-clk').textContent = st.clock_scale ? st.clock_scale.toFixed(3) : '—';
  } else {
    for (const id of ['p-lat', 'p-del', 'p-clk']) $(id).textContent = '—';
    $('p-lat-s').textContent = $('p-del-s').textContent = s.live ? 'no STATS reply' : 'no stats.csv';
  }
  // track
  $('ptrk-sub').textContent = 'Metres in the pose frame: ' + (s.pose_frame === 1
    ? 'y = where the rover faced at start, x = its right'
    : 'x = where the rover faced at start, y = its left') + '. ○ start, ● now with heading; markers in orange.';
  const svg = $('ptrk'); svg.replaceChildren();
  const W = 900, H = 600, pts = PT.track;
  const pins = s.marks.filter(m => m.x != null).map(m => ({a: m.x, b: m.y, id: m.id, label: m.label}));
  if (!pts.length) {
    el('text', {x: W / 2, y: H / 2, 'text-anchor': 'middle'}, svg).textContent = 'Waiting for poses…';
    PTRK = null; return;
  }
  const P = planPlot(svg, W, H, pts, pins.map(m => [m.a, m.b]), 1.0);
  el('text', {x: P.m.l + P.pw / 2, y: H - 6, 'text-anchor': 'middle'}, svg).textContent = 'x (m) · grid ' + fmtStep(P.step, P.step) + ' m';
  el('text', {x: 14, y: P.m.t + P.ph / 2, 'text-anchor': 'middle', transform: 'rotate(-90 14 ' + (P.m.t + P.ph / 2) + ')'}, svg).textContent = 'y (m)';
  polySegments(svg, pts, P.X, P.Y, 3);
  el('circle', {cx: P.X(pts[0][0]), cy: P.Y(pts[0][1]), r: 5, class: 'start'}, svg);
  drawPins(svg, pins, P);
  if (L) {
    const cx = P.X(L.x), cy = P.Y(L.y);
    // heading is relative to the start direction (CCW): along +x in frame 0, along +y in frame 1
    const hx = s.pose_frame === 1 ? -Math.sin(L.theta) : Math.cos(L.theta);
    const hy = s.pose_frame === 1 ? Math.cos(L.theta) : Math.sin(L.theta);
    el('line', {x1: cx, y1: cy, x2: cx + 22 * hx, y2: cy - 22 * hy, class: 'head'}, svg);
    el('circle', {cx, cy, r: 6, class: 'cur'}, svg);
  }
  PTRK = {pts, X: P.X, Y: P.Y, hov: el('circle', {r: 7, class: 'hov', visibility: 'hidden'}, svg)};
}

// -------------------------------------------------------------- GPS view
function gpsStatus(s) {
  const g = s.gps;
  if (!g.last) return ['critical', '!', 'No data', 'no GPS datagrams yet'];
  if (s.live && g.age_s > 3) return ['critical', '!', 'No data', 'last fix ' + g.age_s.toFixed(0) + ' s ago'];
  if (!g.last.valid) return ['warning', '?', 'No fix', 'receiving, ' + g.last.sats + ' sats'];
  return ['good', '✓', 'Fix', QUAL[g.last.q] || ('quality ' + g.last.q)];
}
let GTRK = null;
function renderGps(s) {
  const g = s.gps, [cls, icon, label, sub] = gpsStatus(s);
  HTML['t-fix'] = dot(cls, icon) + label;
  $('t-fix-s').textContent = sub;
  $('t-sats').textContent = g.last ? g.last.sats : '—';
  $('t-hdop').textContent = g.last && g.last.valid ? g.last.hdop.toFixed(2) : '—';
  $('t-hdop-s').textContent = g.last && g.last.valid ? (g.last.hdop < 2 ? 'good geometry' : g.last.hdop < 5 ? 'moderate' : 'poor') : '';
  $('t-rate').textContent = g.rate_hz ? g.rate_hz.toFixed(2) + ' Hz' : '—';
  $('t-age').textContent = s.live && g.age_s != null ? 'last fix ' + g.age_s.toFixed(1) + ' s ago' : (s.live ? '' : 'over the last 10 s of the log');
  $('t-valid').textContent = g.n ? g.n_valid + ' / ' + g.n : '—';
  $('t-rx').textContent = s.live ? 'datagrams ' + g.rx + ', lost ' + g.lost + ', bad ' + g.bad : 'fixes in the log';
  if (g.spread) {
    $('t-cep').textContent = fmtM(g.spread.cep50) + ' m';
    $('t-cep-s').textContent = 'CEP50 · 2DRMS ' + fmtM(g.spread.drms2) + ' m (' + g.spread.n + ' fixes)';
  } else { $('t-cep').textContent = '—'; $('t-cep-s').textContent = 'needs ≥ 10 valid fixes'; }
  $('pos').textContent = g.last && g.last.valid ? 'Latest: ' + g.last.lat.toFixed(7) + '°, ' + g.last.lon.toFixed(7) + '° · alt ' +
    g.last.alt.toFixed(1) + ' m · ' + (g.last.speed * 3.6).toFixed(1) + ' km/h' : 'No valid position.';
  HTML['tbl'] = g.recent.map(f => '<tr><td>' + utc(f.utc_ms) + '</td><td>' + f.t.toFixed(1) + '</td><td>' +
    (f.valid ? f.lat.toFixed(7) : '—') + '</td><td>' + (f.valid ? f.lon.toFixed(7) : '—') + '</td><td>' + f.sats +
    '</td><td>' + (f.valid ? f.hdop.toFixed(2) : '—') + '</td><td>' + (QUAL[f.q] || f.q) + '</td><td>' + (f.valid ? 'yes' : 'no') + '</td></tr>').join('');
  const svg = $('gtrk'); svg.replaceChildren();
  const W = 600, H = 600, pts = GT.track;
  const pins = s.marks.filter(m => m.e != null).map(m => ({a: m.e, b: m.n, id: m.id, label: m.label, ring: m.spread}));
  if (!pts.length) {
    el('text', {x: W / 2, y: H / 2, 'text-anchor': 'middle'}, svg).textContent = 'Waiting for a valid fix…';
    GTRK = null;
  } else {
    const P = planPlot(svg, W, H, pts, pins.map(m => [m.a, m.b]), 4.0);
    el('text', {x: P.m.l + P.pw / 2, y: H - 6, 'text-anchor': 'middle'}, svg).textContent = 'east (m) · grid ' + fmtStep(P.step, P.step) + ' m';
    el('text', {x: 14, y: P.m.t + P.ph / 2, 'text-anchor': 'middle', transform: 'rotate(-90 14 ' + (P.m.t + P.ph / 2) + ')'}, svg).textContent = 'north (m)';
    el('text', {x: P.m.l + P.pw - 8, y: P.m.t + 16, 'text-anchor': 'end'}, svg).textContent = 'N ↑';
    polySegments(svg, pts, P.X, P.Y, 5);
    el('circle', {cx: P.X(pts[0][0]), cy: P.Y(pts[0][1]), r: 5, class: 'start'}, svg);
    drawPins(svg, pins, P);
    const Lp = pts[pts.length - 1];
    el('circle', {cx: P.X(Lp[0]), cy: P.Y(Lp[1]), r: 6, class: 'cur'}, svg);
    GTRK = {pts, X: P.X, Y: P.Y, hov: el('circle', {r: 7, class: 'hov', visibility: 'hidden'}, svg)};
  }
  drawSeries('sats', s, 1, v => v.toFixed(0), 12);
  drawSeries('hdop', s, 2, v => v.toFixed(1), 3);
}

// one measure per chart (no dual axis)
const TS = {};
function drawSeries(id, s, idx, fmt, yMinSpan) {
  const svg = $(id); svg.replaceChildren();
  const ser = s.gps.series, W = 520, H = 170, m = {l: 36, r: 12, t: 10, b: 26};
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  if (!ser.length) { TS[id] = null; return; }
  const t1 = ser[ser.length - 1][0], t0 = Math.max(ser[0][0], t1 - 600);
  const X = t => m.l + (t - t0) / Math.max(t1 - t0, 1) * pw;
  const vals = ser.filter(r => idx === 1 || r[3]).map(r => r[idx]);
  const hi = Math.max(yMinSpan, ...vals) * 1.1;
  const Y = v => m.t + ph - v / hi * ph;
  let start = null;
  ser.forEach((r, i) => {
    if (!r[3] && start === null) start = r[0];
    if ((r[3] || i === ser.length - 1) && start !== null) {
      el('rect', {x: X(start), y: m.t, width: Math.max(X(r[0]) - X(start), 2), height: ph, class: 'nofix'}, svg);
      start = null;
    }
  });
  const ystep = nice(hi, 4);
  for (let v = 0; v <= hi; v += ystep) {
    el('line', {x1: m.l, x2: m.l + pw, y1: Y(v), y2: Y(v), class: 'gl'}, svg);
    el('text', {x: m.l - 6, y: Y(v) + 4, 'text-anchor': 'end'}, svg).textContent = fmt(v);
  }
  el('line', {x1: m.l, x2: m.l + pw, y1: m.t + ph, y2: m.t + ph, class: 'ax'}, svg);
  const xstep = nice(Math.max(t1 - t0, 1), 5);
  for (let t = Math.ceil(t0 / xstep) * xstep; t <= t1; t += xstep)
    el('text', {x: X(t), y: H - 8, 'text-anchor': 'middle'}, svg).textContent = t.toFixed(0) + ' s';
  let seg = [];
  const flush = () => { if (seg.length > 1) el('polyline', {points: seg.join(' '), class: 'ln'}, svg); seg = []; };
  for (const r of ser) {
    if (r[0] < t0) continue;
    if (idx === 2 && !r[3]) { flush(); continue; }
    seg.push(X(r[0]).toFixed(1) + ',' + Y(r[idx]).toFixed(1));
  }
  flush();
  TS[id] = {rows: ser.filter(r => r[0] >= t0), X, Y, idx, fmt,
            xh: el('line', {y1: m.t, y2: m.t + ph, class: 'xh', visibility: 'hidden'}, svg),
            dot: el('circle', {r: 4, class: 'cur', visibility: 'hidden'}, svg)};
}

// ------------------------------------------------------------ markers view
function renderMarks(s) {
  $('marks-sub').textContent = s.live ? (s.marks.length + ' marker(s) this session · saved in ' + (s.marks_file || '—')) :
    (s.marks.length + ' MARK event(s) in this run, positioned from its logs');
  $('mark-btn').disabled = !s.live;
  let prev = null;
  HTML['marks-tbl'] = s.marks.map(m => {
    const dg = prev && m.e != null && prev.e != null ? Math.hypot(m.e - prev.e, m.n - prev.n).toFixed(2) : '—';
    const dp = prev && m.x != null && prev.x != null ? Math.hypot(m.x - prev.x, m.y - prev.y).toFixed(3) : '—';
    prev = m;
    return '<tr><td>' + esc(m.id) + '</td><td class="l">' + esc(m.label) + '</td><td>' + esc((m.wall_utc || '').replace('T', ' ').replace('Z', '')) +
      '</td><td>' + (m.lat != null ? m.lat.toFixed(7) : '—') + '</td><td>' + (m.lon != null ? m.lon.toFixed(7) : '—') +
      '</td><td>' + m.gps_n + '</td><td>' + (m.spread != null ? m.spread.toFixed(2) : '—') +
      '</td><td>' + (m.x != null ? m.x.toFixed(3) : '—') + '</td><td>' + (m.y != null ? m.y.toFixed(3) : '—') +
      '</td><td>' + (m.theta != null ? deg(m.theta).toFixed(1) : '—') + '</td><td>' + dg + '</td><td>' + dp + '</td><td>' +
      (s.live ? '<button class="small" data-del="' + esc(m.id) + '" aria-label="Delete marker ' + esc(m.id) + '">Delete</button>' : '') + '</td></tr>';
  }).join('') || '<tr><td colspan="13" class="l">No markers yet.</td></tr>';
}
$('mark-form').addEventListener('submit', async ev => {
  ev.preventDefault();
  $('mark-btn').disabled = true;
  try {
    const r = await (await fetch('/api/mark', {method: 'POST', body: JSON.stringify({label: $('mark-label').value})})).json();
    $('mark-msg').textContent = (r.ok ? 'saved' : 'NOT saved to file (' + r.error + '), kept on this page') +
      (r.app ? ' · olympus-pose: ' + r.app : ' · olympus-pose did not answer (MARK not in its log)');
    $('mark-label').value = '';
  } catch (e) { $('mark-msg').textContent = 'failed: ' + e; }
  $('mark-btn').disabled = false;
  refresh();
});
$('marks-tbl').addEventListener('click', async ev => {
  const id = ev.target.dataset && ev.target.dataset.del;
  if (!id || !confirm('Delete marker ' + id + '? (the MARK event stays in the olympus-pose log)')) return;
  await fetch('/api/mark_delete', {method: 'POST', body: JSON.stringify({id})});
  refresh();
});

// ----------------------------------------------------------- controls view
async function ctrl(cmd, ask) {
  if (ask && !confirm(ask)) return;
  try {
    const r = await (await fetch('/api/ctrl', {method: 'POST', body: JSON.stringify({cmd})})).json();
    $('ctrl-msg').textContent = cmd + ' → ' + (r.reply || r.error);
  } catch (e) { $('ctrl-msg').textContent = cmd + ' → failed: ' + e; }
  refresh();
}
$('reset-btn').addEventListener('click', () => {
  const x = Number($('rx').value) || 0, y = Number($('ry').value) || 0, t = (Number($('rt').value) || 0) * Math.PI / 180;
  ctrl('RESET_POSE ' + x + ' ' + y + ' ' + t.toFixed(6), 'Reset the pose to x=' + x + ', y=' + y + ', θ=' + $('rt').value + '°? Do not do this in the middle of a test.');
});
$('agents').addEventListener('click', ev => {
  const b = ev.target.closest('button'); if (!b) return;
  const verb = b.dataset.verb, a = b.dataset.agent;
  ctrl(verb + ' ' + a, verb === 'SUSPEND' ? 'Suspend the ' + a + ' agent? The pose stops updating until it is resumed.' : null);
});
$('clear-btn').addEventListener('click', async () => {
  if (!confirm('Clear the tracks on this page? Logs and markers are kept.')) return;
  await fetch('/api/clear', {method: 'POST'});
  refresh();
});
function renderCtrl(s) {
  const st = s.stats, live = s.live;
  for (const b of document.querySelectorAll('.ctl')) b.disabled = !live || !s.ctrl_ok;
  $('gpx-session').hidden = !live;
  const states = st ? st.agents : [];
  HTML['agents'] = ['acquisition', 'fusion', 'estimation', 'communication', 'gps'].map(n => {
    const a = states.find(x => x.name === n), state = a ? a.state : '—';
    const cls = state === 'active' ? 'good' : state === 'suspended' ? 'warning' : 'critical';
    const btn = n === 'communication' || !live ? '' : state === 'suspended' ?
      '<button class="small ctl" data-verb="RESUME" data-agent="' + n + '"' + (s.ctrl_ok ? '' : ' disabled') + '>Resume</button>' :
      '<button class="small ctl" data-verb="SUSPEND" data-agent="' + n + '"' + (s.ctrl_ok ? '' : ' disabled') + '>Suspend</button>';
    return '<tr><td class="l">' + n + '</td><td class="l"><span class="status">' + (a ? dot(cls, cls === 'good' ? '✓' : '!') : '') + state + '</span></td><td>' + btn + '</td></tr>';
  }).join('');
  $('stats-sub').textContent = live ? (s.ctrl_ok ? 'From the app\'s STATS command, once per second.' :
    'olympus-pose is not answering on its control port: is it running?') : 'Last line of stats.csv of this run.';
  if (st) {
    const L = st.links, row = (k, v) => '<tr><td class="l">' + k + '</td><td>' + v + '</td></tr>';
    const f = (v, d, u) => v == null ? '—' : v.toFixed(d) + (u || '');
    HTML['stats-tbl'] = row('run time', f(st.run_s, 0, ' s')) + row('poses published', f(st.poses, 0)) +
      row('latency p50 / p95 / max', f(st.lat_p50, 1) + ' / ' + f(st.lat_p95, 1) + ' / ' + f(st.lat_max, 1) + ' ms') +
      row('pose interval p95', f(st.int_p95, 1, ' ms')) +
      ['acq', 'fus', 'est'].map(k => row('link ' + {acq: 'acquisition→fusion', fus: 'fusion→estimation', est: 'estimation→comm.'}[k],
        f(L[k].pct, 3, ' %') + ' · max gap ' + f(L[k].max_gap, 0) + ' · drops ' + f(L[k].drops, 0))).join('') +
      row('LLC clock scale', f(st.clock_scale, 3)) + row('GPS fixes (valid)', f(st.gps_fixes, 0) + ' (' + f(st.gps_valid, 0) + ')') +
      row('memory (RSS) · CPU', f(st.rss_kb, 0, ' kB') + ' · ' + f(st.cpu_s, 1, ' s')) + row('log drops', f(st.log_drops, 0));
    $('stats-raw').textContent = Object.entries(st.raw).map(([k, v]) => k + '=' + v).join('  ');
  } else { HTML['stats-tbl'] = ''; $('stats-raw').textContent = ''; }
}
async function loadRuns() {
  try {
    const r = await (await fetch('/api/runs')).json();
    $('runs-sub').textContent = 'Folder ' + r.log_dir + ' (newest first). Files download to this laptop.';
    const mk = r.marks_file ? '<tr><td class="l">markers</td><td class="l"><a href="/files/' + encodeURIComponent(r.marks_file.split(/[\\/]/).pop()) + '">' +
      esc(r.marks_file.split(/[\\/]/).pop()) + '</a></td><td></td></tr>' : '';
    HTML['runs'] = mk + (r.runs.map(run => '<tr><td class="l">' + esc(run.name) + '</td><td class="l">' +
      run.files.map(f => '<a href="/files/' + encodeURIComponent(run.name) + '/' + encodeURIComponent(f.name) + '">' + esc(f.name) +
        '</a> <span class="msg">' + (f.size / 1024).toFixed(0) + ' KB</span>').join(' · ') +
      '</td><td class="l">' + (run.files.some(f => f.name === 'gps.csv') ? '<a href="/api/gpx?run=' + encodeURIComponent(run.name) + '">GPX</a>' : '') +
      '</td></tr>').join('') || '<tr><td colspan="3" class="l">No runs yet.</td></tr>');
  } catch (e) { $('runs-sub').textContent = 'cannot list runs: ' + e; }
}
$('runs-btn').addEventListener('click', loadRuns);

// ------------------------------------------------------------- hover layers
function svgPoint(svg, ev) {
  const p = svg.createSVGPoint(); p.x = ev.clientX; p.y = ev.clientY;
  return p.matrixTransform(svg.getScreenCTM().inverse());
}
function placeTip(tip, card, ev, html) {
  tip.innerHTML = html; tip.style.display = 'block';
  const r = card.getBoundingClientRect();
  let x = ev.clientX - r.left + 12;
  if (x + tip.offsetWidth > r.width - 4) x = ev.clientX - r.left - tip.offsetWidth - 12;
  tip.style.left = x + 'px'; tip.style.top = (ev.clientY - r.top + 12) + 'px';
}
function trackHover(svgId, getT, card, tipId, html) {
  $(svgId).addEventListener('pointermove', ev => {
    const T = getT(); if (!T) return;
    const q = svgPoint($(svgId), ev);
    let best = null, bd = 1e9;
    for (const p of T.pts) { const d = Math.hypot(T.X(p[0]) - q.x, T.Y(p[1]) - q.y); if (d < bd) { bd = d; best = p; } }
    if (!best || bd > 30) { T.hov.setAttribute('visibility', 'hidden'); $(tipId).style.display = 'none'; return; }
    T.hov.setAttribute('cx', T.X(best[0])); T.hov.setAttribute('cy', T.Y(best[1])); T.hov.setAttribute('visibility', 'visible');
    placeTip($(tipId), $(card), ev, html(best));
  });
  $(svgId).addEventListener('pointerleave', () => { const T = getT(); if (T) T.hov.setAttribute('visibility', 'hidden'); $(tipId).style.display = 'none'; });
}
trackHover('ptrk', () => PTRK, 'c-ptrk', 'tip-ptrk', p => 't ' + p[2].toFixed(1) + ' s<br>x ' + p[0].toFixed(3) + ' m, y ' + p[1].toFixed(3) + ' m');
trackHover('gtrk', () => GTRK, 'c-gtrk', 'tip-gtrk', p => 'UTC ' + utc(p[4]) + ' · t ' + p[2].toFixed(1) + ' s<br>E ' + fmtM(p[0]) + ' m, N ' + fmtM(p[1]) + ' m · ' + p[3] + ' sats');
for (const [id, label] of [['sats', 'Satellites'], ['hdop', 'HDOP']]) {
  $(id).addEventListener('pointermove', ev => {
    const T = TS[id]; if (!T || !T.rows.length) return;
    const q = svgPoint($(id), ev);
    let best = T.rows[0];
    for (const r of T.rows) if (Math.abs(T.X(r[0]) - q.x) < Math.abs(T.X(best[0]) - q.x)) best = r;
    const x = T.X(best[0]), ok = T.idx === 1 || best[3];
    T.xh.setAttribute('x1', x); T.xh.setAttribute('x2', x); T.xh.setAttribute('visibility', 'visible');
    if (ok) { T.dot.setAttribute('cx', x); T.dot.setAttribute('cy', T.Y(best[T.idx])); }
    T.dot.setAttribute('visibility', ok ? 'visible' : 'hidden');
    placeTip($('tip-' + id), $('c-' + id), ev, 't ' + best[0].toFixed(1) + ' s<br>' + label + ' ' + (ok ? T.fmt(best[T.idx]) : '—') + (best[3] ? '' : ' · no fix'));
  });
  $(id).addEventListener('pointerleave', () => { const T = TS[id]; if (T) { T.xh.setAttribute('visibility', 'hidden'); T.dot.setAttribute('visibility', 'hidden'); } $('tip-' + id).style.display = 'none'; });
}

// ------------------------------------------------- figure export (SVG / PNG)
// Always in the light theme, with the styles inlined, so the file looks the same
// in the report as on screen.
const PROPS = ['fill', 'stroke', 'stroke-width', 'stroke-dasharray', 'stroke-linecap', 'stroke-linejoin',
               'font-size', 'font-family', 'font-weight', 'visibility', 'opacity'];
function exportFig(id, png) {
  const svg = $(id), root = document.documentElement, prev = root.getAttribute('data-theme');
  root.setAttribute('data-theme', 'light');
  const clone = svg.cloneNode(true), src = svg.querySelectorAll('*'), dst = clone.querySelectorAll('*');
  src.forEach((e, i) => { const cs = getComputedStyle(e); dst[i].setAttribute('style', PROPS.map(p => p + ':' + cs.getPropertyValue(p)).join(';')); });
  if (prev === null) root.removeAttribute('data-theme'); else root.setAttribute('data-theme', prev);
  const vb = svg.viewBox.baseVal;
  clone.setAttribute('xmlns', NS); clone.setAttribute('width', vb.width); clone.setAttribute('height', vb.height);
  clone.removeAttribute('style');
  const bg = document.createElementNS(NS, 'rect');
  bg.setAttribute('width', vb.width); bg.setAttribute('height', vb.height); bg.setAttribute('fill', '#ffffff');
  clone.insertBefore(bg, clone.firstChild);
  const text = new XMLSerializer().serializeToString(clone);
  const stamp = new Date().toISOString().slice(0, 19).replace(/[-:]/g, '').replace('T', '-');
  const name = 'olympus-' + {ptrk: 'pose-track', gtrk: 'gps-track'}[id] + '-' + stamp;
  const blob = new Blob([text], {type: 'image/svg+xml'});
  const save = (b, ext) => { const a = document.createElement('a'); a.href = URL.createObjectURL(b); a.download = name + ext; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 5000); };
  if (!png) return save(blob, '.svg');
  const img = new Image(), url = URL.createObjectURL(blob);
  img.onload = () => {
    const c = document.createElement('canvas'); c.width = vb.width * 2; c.height = vb.height * 2;
    const g = c.getContext('2d'); g.scale(2, 2); g.drawImage(img, 0, 0);
    URL.revokeObjectURL(url);
    c.toBlob(b => save(b, '.png'), 'image/png');
  };
  img.src = url;
}
for (const b of document.querySelectorAll('[data-fig]')) b.addEventListener('click', () => exportFig(b.dataset.fig, !!b.dataset.png));

// ---------------------------------------------------------------- refresh
function merge(T, part) {
  if (part.full) T.track = part.track; else for (const p of part.track) T.track.push(p);
  if (T.track.length > MAX_TRACK) T.track = T.track.slice(T.track.length - MAX_TRACK);
  T.epoch = part.epoch;
  T.last = T.track.length ? T.track[T.track.length - 1][T.track[0].length - 1] : 0;
}
function render() {
  const s = S;
  $('src').textContent = s.live ? 'live' : 'replay · ' + s.source;
  HTML['conn'] = !s.live ? '' : s.ctrl_ok ? dot('good', '✓') + 'olympus-pose answering' : dot('critical', '!') + 'olympus-pose not answering';
  if (TAB === 'pose') renderPose(s);
  else if (TAB === 'gps') renderGps(s);
  else if (TAB === 'marks') renderMarks(s);
  else renderCtrl(s);
}
async function refresh() {
  try {
    const r = await fetch('/api/state?pe=' + PT.epoch + '&ps=' + PT.last + '&ge=' + GT.epoch + '&gs=' + GT.last, {cache: 'no-store'});
    S = await r.json();
  } catch (e) { $('src').textContent = 'olympus-pose_gui.py not reachable'; return; }
  merge(PT, S.pose); merge(GT, S.gps);
  render();
}
try { const t = localStorage.getItem('opg-tab'); if (t && $('tab-' + t)) showTab(t); } catch (e) {}
refresh();
setInterval(() => { if (S && S.live) refresh(); }, 1000);
</script>
</body>
</html>
'''

if __name__ == '__main__':
    main()

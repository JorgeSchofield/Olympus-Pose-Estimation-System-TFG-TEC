#!/usr/bin/env python3
"""GPS viewer for olympus-pose: live web page, log replay and GPX export.

DESIGN.md §11.4. The GPS is NOT used by the estimator (thesis §1.6); this tool
only shows that the GPS agent works: fix state, satellites, HDOP, fix rate, the
track in metres and the scatter of a static receiver.

    # live, on the rover (olympus-pose sends one datagram per fix to UDP 47003)
    gps_view.py                      # then open http://<rover-ip>:8080 on a laptop
    # a logged run, same page
    gps_view.py --csv /var/lib/olympus-pose/runs/<run>/gps.csv
    # export a logged run for Google Earth / any GIS tool
    gps_view.py --csv gps.csv --gpx track.gpx [--date 2026-10-21]

The page needs no internet connection: the track is drawn in local east/north
metres from the first valid fix (equirectangular, fine over a few km). The
"OpenStreetMap" link only works if the viewing laptop is online.
"""
import argparse
import binascii
import csv
import json
import math
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

FMT = '<4sIQ6dI4BII'                 # 80 bytes, see src/core/pose_dgram.h
SIZE = struct.calcsize(FMT)
R_EARTH = 6371000.0
MAX_FIXES = 20000
QUALITY = {0: 'no fix', 1: 'GPS', 2: 'DGPS', 3: 'PPS', 4: 'RTK fixed', 5: 'RTK float',
           6: 'estimated', 7: 'manual', 8: 'simulation'}


def decode(data):
    if len(data) != SIZE:
        raise ValueError('length %d' % len(data))
    f = struct.unpack(FMT, data)
    if f[0] != b'OGP1':
        raise ValueError('magic %r' % f[0])
    if binascii.crc32(data[:76]) & 0xFFFFFFFF != f[-1]:
        raise ValueError('bad CRC')
    return {'seq': f[1], 't_rx_ns': f[2], 'lat': f[3], 'lon': f[4], 'alt': f[5], 'hdop': f[6],
            'speed': f[7], 'course': f[8], 'utc_ms': f[9], 'q': f[10], 'sats': f[11],
            'valid': bool(f[12]), 'n_valid': f[14]}


class State:
    """Fixes and the track, kept up to date as fixes arrive.

    A refresh of the page must not cost more as the run gets longer: the track
    (east/north, line breaks) is computed once per fix in add(), and snapshot()
    sends only the track points the page does not have yet, plus windows of
    fixed length (10 min of charts, 2 min of scatter, 10 table rows).
    """

    def __init__(self, source):
        self.lock = threading.Lock()
        self.source = source
        self.live = source.startswith('UDP')
        self.epoch = 0
        self.clear()
        self.rx = self.bad = self.lost = 0
        self.last_seq = None

    def clear(self):
        self.epoch += 1        # tells the page to drop the track it holds
        self.fixes = []        # dicts with 't' = seconds on one monotonic base
        self.track = []        # [e, n, t_rel, sats, utc_ms, brk, id] per valid fix
        self.origin = None
        self.t0 = None
        self.prev = None
        self.next_id = 1
        self.n = self.n_valid = 0

    def add(self, fx):
        with self.lock:
            if self.t0 is None:
                self.t0 = fx['t']
            self.n += 1
            if fx['valid']:
                self.n_valid += 1
                if self.origin is None:
                    self.origin = (fx['lat'], fx['lon'])
                e, n = self.en(fx['lat'], fx['lon'])
                # brk = 1: the line must not join this fix to the previous valid one
                p = self.prev
                brk = int(p is None or not p['valid'] or fx['t'] - p['t'] > 5.0)
                self.track.append([round(e, 3), round(n, 3), round(fx['t'] - self.t0, 1),
                                   fx['sats'], fx['utc_ms'], brk, self.next_id])
                self.next_id += 1
                if len(self.track) > MAX_FIXES + 1000:   # trim in blocks, not per fix
                    del self.track[:1000]
            self.prev = fx
            self.fixes.append(fx)
            if len(self.fixes) > MAX_FIXES + 1000:
                del self.fixes[:1000]

    def en(self, lat, lon):
        lat0, lon0 = self.origin
        k = math.pi / 180 * R_EARTH
        return ((lon - lon0) * k * math.cos(math.radians(lat0)), (lat - lat0) * k)

    def snapshot(self, epoch=0, since=0):
        """State for the page. Track points with id > since only, unless the page
        holds another epoch or points that were trimmed: then the full track."""
        with self.lock:
            fx = self.fixes
            out = {'source': self.source, 'live': self.live, 'rx': self.rx, 'bad': self.bad,
                   'lost': self.lost, 'n': self.n, 'n_valid': self.n_valid, 'origin': self.origin,
                   'epoch': self.epoch, 'full': True, 'last': None, 'age_s': None, 'rate_hz': 0.0,
                   'track': [], 'series': [], 'recent': [], 'spread': None}
            if not fx:
                return out
            now = time.monotonic() if self.live else fx[-1]['t']
            last = fx[-1]
            out['last'] = {k: last[k] for k in ('lat', 'lon', 'alt', 'hdop', 'speed', 'course', 'utc_ms',
                                                'q', 'sats', 'valid')}
            out['last']['quality'] = QUALITY.get(last['q'], str(last['q']))
            out['age_s'] = max(0.0, now - last['t'])

            # newest-first scans that stop at the edge of their window
            def tail(seq, t_from, key):
                i = len(seq)
                while i > 0 and key(seq[i - 1]) >= t_from:
                    i -= 1
                return seq[i:]

            recent = tail(fx, now - 10.0, lambda f: f['t'])
            if len(recent) > 1:
                out['rate_hz'] = (len(recent) - 1) / max(recent[-1]['t'] - recent[0]['t'], 1e-6)
            t0 = self.t0
            out['series'] = [[round(f['t'] - t0, 1), f['sats'], round(f['hdop'], 2), int(f['valid'])]
                             for f in tail(fx, last['t'] - 600.0, lambda f: f['t'])]
            out['recent'] = [{'t': round(f['t'] - t0, 1), 'utc_ms': f['utc_ms'], 'lat': f['lat'],
                              'lon': f['lon'], 'sats': f['sats'], 'hdop': f['hdop'], 'q': f['q'],
                              'valid': f['valid']} for f in fx[-10:]][::-1]

            trk = self.track
            if trk:
                # scatter over the last 2 min: the receiver's precision when the rover is still
                pts = tail(trk, last['t'] - t0 - 120.0, lambda q: q[2])
                if len(pts) >= 10:
                    me = sum(q[0] for q in pts) / len(pts)
                    mn = sum(q[1] for q in pts) / len(pts)
                    d = sorted(math.hypot(q[0] - me, q[1] - mn) for q in pts)
                    drms = math.sqrt(sum(x * x for x in d) / len(d))
                    out['spread'] = {'n': len(d), 'cep50': d[len(d) // 2], 'drms2': 2 * drms}
                if epoch == self.epoch and since >= trk[0][6] - 1:
                    out['full'] = False
                    out['track'] = tail(trk, since + 1, lambda q: q[6])
                else:
                    out['track'] = list(trk)
            return out


def udp_loop(state, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', port))
    while True:
        data = s.recv(2048)
        try:
            d = decode(data)
        except ValueError:
            with state.lock:
                state.bad += 1
            continue
        with state.lock:
            state.rx += 1
            if state.last_seq is not None and d['seq'] != state.last_seq + 1:
                state.lost += (d['seq'] - state.last_seq - 1) & 0xFFFFFFFF
            state.last_seq = d['seq']
        d['t'] = time.monotonic()
        state.add(d)


def load_csv(state, path):
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                state.add({'t': int(row['t_rx_ns']) * 1e-9, 'utc_ms': int(row['utc_ms']),
                           'lat': float(row['lat_deg']), 'lon': float(row['lon_deg']),
                           'alt': float(row['alt_m']), 'q': int(row['quality']), 'sats': int(row['sats']),
                           'hdop': float(row['hdop']), 'speed': float(row['speed_mps']),
                           'course': float(row['course_deg']), 'valid': row['valid'].strip() == '1'})
            except (KeyError, ValueError):
                continue
            state.rx += 1


def write_gpx(state, path, date):
    pts = [f for f in state.fixes if f['valid']]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n'
                '<gpx version="1.1" creator="olympus-pose gps_view.py" '
                'xmlns="http://www.topografix.com/GPX/1/1">\n<trk><name>%s</name><trkseg>\n' % state.source)
        prev = None
        for p in state.fixes:
            if not p['valid']:
                prev = p
                continue
            if prev is not None and not prev['valid']:
                f.write('</trkseg><trkseg>\n')        # a fix loss starts a new segment
            prev = p
            t = ''
            if date:
                ms = p['utc_ms']
                t = '<time>%sT%02d:%02d:%06.3fZ</time>' % (date, ms // 3600000, ms // 60000 % 60,
                                                          ms % 60000 / 1000)
            f.write('<trkpt lat="%.8f" lon="%.8f"><ele>%.2f</ele>%s<sat>%d</sat><hdop>%.2f</hdop></trkpt>\n'
                    % (p['lat'], p['lon'], p['alt'], t, p['sats'], p['hdop']))
        f.write('</trkseg></trk>\n</gpx>\n')
    print('gps_view: %d valid fixes -> %s' % (len(pts), path))


def make_handler(state):
    class H(BaseHTTPRequestHandler):
        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/?'):
                self._send(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
            elif self.path == '/state' or self.path.startswith('/state?'):
                q = parse_qs(urlsplit(self.path).query)
                try:
                    epoch, since = int(q.get('epoch', ['0'])[0]), int(q.get('since', ['0'])[0])
                except ValueError:
                    epoch = since = 0
                body = json.dumps(state.snapshot(epoch, since), separators=(',', ':')).encode()
                self._send(200, 'application/json', body)
            else:
                self._send(404, 'text/plain', b'not found')

        def do_POST(self):
            if self.path == '/clear' and state.live:
                with state.lock:
                    state.clear()
                self._send(200, 'application/json', b'{}')
            else:
                self._send(404, 'text/plain', b'not found')

        def log_message(self, fmt, *args):
            pass
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--udp-port', type=int, default=47003, help='io.udp_gps_port of pose.conf')
    ap.add_argument('--http', default='0.0.0.0:8080', help='address:port of the web page')
    ap.add_argument('--csv', help='show a logged run (gps.csv) instead of live data')
    ap.add_argument('--gpx', help='with --csv: write the valid fixes as GPX and exit')
    ap.add_argument('--date', help='with --gpx: UTC date YYYY-MM-DD for the point times')
    a = ap.parse_args()

    state = State('UDP %d' % a.udp_port if not a.csv else a.csv)
    if a.csv:
        load_csv(state, a.csv)
        if a.gpx:
            write_gpx(state, a.gpx, a.date)
            return
        print('gps_view: %d fixes (%d valid) from %s' % (len(state.fixes),
              sum(1 for f in state.fixes if f['valid']), a.csv))
    elif a.gpx:
        sys.exit('gps_view: --gpx needs --csv')
    else:
        threading.Thread(target=udp_loop, args=(state, a.udp_port), daemon=True).start()
    host, _, port = a.http.rpartition(':')
    srv = ThreadingHTTPServer((host or '0.0.0.0', int(port)), make_handler(state))
    print('gps_view: open http://%s:%s/ (Ctrl-C to stop)' % (host if host not in ('', '0.0.0.0') else '<this-host>', port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


PAGE = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Olympus GPS</title>
<style>
:root {
  color-scheme: light;
  --bg: #f4f4f1; --surface: #fcfcfb; --rule: #e2e1dc; --grid: #ebeae6;
  --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #7a7974;
  --series-1: #2a78d6;
  --good: #0ca30c; --warning: #fab219; --critical: #d03b3b;
  --mono: ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --bg: #111110; --surface: #1a1a19; --rule: #33332f; --grid: #262624;
    --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #8f8e86;
    --series-1: #3987e5;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --bg: #111110; --surface: #1a1a19; --rule: #33332f; --grid: #262624;
  --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #8f8e86;
  --series-1: #3987e5;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text-primary);
  font: 15px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; padding: 16px; }
.wrap { max-width: 1180px; margin: 0 auto; }
header { display: flex; flex-wrap: wrap; align-items: baseline; gap: 8px 16px; margin-bottom: 14px; }
h1 { font-size: 22px; margin: 0; }
.src { color: var(--text-secondary); font-family: var(--mono); font-size: 13px; }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin-bottom: 14px; }
.tile { background: var(--surface); border: 1px solid var(--rule); border-radius: 8px; padding: 10px 12px; min-width: 0; }
.tile .k { color: var(--text-secondary); font-size: 12.5px; }
.tile .v { font-size: 24px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 2px; }
.tile .s { color: var(--text-muted); font-size: 12.5px; margin-top: 2px; overflow-wrap: anywhere; }
.status { display: inline-flex; align-items: center; gap: 8px; }
.dot { width: 14px; height: 14px; border-radius: 50%; flex: none; display: inline-grid; place-items: center;
  color: #fff; font-size: 10px; font-weight: 700; line-height: 1; }
.grid2 { display: grid; grid-template-columns: minmax(0, 1fr); gap: 14px; }
@media (min-width: 900px) { .grid2 { grid-template-columns: minmax(0, 1.25fr) minmax(0, 1fr); } }
.card { background: var(--surface); border: 1px solid var(--rule); border-radius: 8px; padding: 12px 14px; min-width: 0; position: relative; }
.card h2 { font-size: 15px; margin: 0 0 2px; }
.card .sub { color: var(--text-muted); font-size: 12.5px; margin: 0 0 8px; }
svg { display: block; width: 100%; height: auto; }
svg text { fill: var(--text-muted); font-size: 11px; font-family: system-ui, sans-serif; }
.gl { stroke: var(--grid); stroke-width: 1; }
.ax { stroke: var(--rule); stroke-width: 1; }
.trk { fill: none; stroke: var(--series-1); stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }
.ln { fill: none; stroke: var(--series-1); stroke-width: 2; stroke-linejoin: round; }
.cur { fill: var(--series-1); stroke: var(--surface); stroke-width: 2; }
.start { fill: var(--surface); stroke: var(--text-secondary); stroke-width: 2; }
.hov { fill: none; stroke: var(--text-primary); stroke-width: 1.5; }
.xh { stroke: var(--text-muted); stroke-width: 1; stroke-dasharray: 3 3; }
.nofix { fill: var(--grid); }
.tip { position: absolute; pointer-events: none; background: var(--surface); color: var(--text-primary);
  border: 1px solid var(--rule); border-radius: 6px; padding: 6px 8px; font-size: 12.5px; white-space: nowrap;
  box-shadow: 0 2px 8px rgba(0,0,0,.12); display: none; z-index: 2; font-variant-numeric: tabular-nums; }
.side { display: grid; gap: 14px; }
table { border-collapse: collapse; width: 100%; font-size: 13px; font-variant-numeric: tabular-nums; }
th, td { text-align: right; padding: 5px 8px; border-bottom: 1px solid var(--rule); white-space: nowrap; }
th { color: var(--text-secondary); font-weight: 600; font-size: 12px; }
th:first-child, td:first-child { text-align: left; }
.tw { overflow-x: auto; }
.foot { display: flex; flex-wrap: wrap; gap: 10px 18px; align-items: center; margin-top: 14px; color: var(--text-secondary); font-size: 13.5px; }
.foot a { color: var(--series-1); }
button { font: inherit; font-size: 13.5px; padding: 6px 12px; border-radius: 6px; border: 1px solid var(--rule);
  background: var(--surface); color: var(--text-primary); cursor: pointer; }
button:focus-visible { outline: 2px solid var(--series-1); outline-offset: 2px; }
</style>
</head>
<body>
<div class="wrap">
<header>
  <h1>Olympus GPS</h1>
  <span class="src" id="src">connecting…</span>
</header>

<div class="tiles">
  <div class="tile"><div class="k">Fix</div><div class="v status" id="t-fix">—</div><div class="s" id="t-fix-s"></div></div>
  <div class="tile"><div class="k">Satellites</div><div class="v" id="t-sats">—</div><div class="s">in the solution</div></div>
  <div class="tile"><div class="k">HDOP</div><div class="v" id="t-hdop">—</div><div class="s" id="t-hdop-s"></div></div>
  <div class="tile"><div class="k">Fix rate</div><div class="v" id="t-rate">—</div><div class="s" id="t-age"></div></div>
  <div class="tile"><div class="k">Valid fixes</div><div class="v" id="t-valid">—</div><div class="s" id="t-rx"></div></div>
  <div class="tile"><div class="k">Scatter, last 2 min (rover still)</div><div class="v" id="t-cep">—</div><div class="s" id="t-cep-s">needs ≥ 10 valid fixes</div></div>
</div>

<div class="grid2">
  <div class="card" id="c-trk">
    <h2>Track</h2>
    <p class="sub" id="trk-sub">East/north metres from the first valid fix. ○ start, ● latest; the line breaks where the fix was lost.</p>
    <svg id="trk" viewBox="0 0 600 600" role="img" aria-label="GPS track in metres"></svg>
    <div class="tip" id="tip-trk"></div>
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
</div>

<div class="foot">
  <span id="pos">—</span>
  <a id="osm" href="#" target="_blank" rel="noopener" hidden>Open in OpenStreetMap</a>
  <button id="clear" type="button">Clear track</button>
</div>
</div>

<script>
const QUAL = {0:'no fix',1:'GPS',2:'DGPS',3:'PPS',4:'RTK fixed',5:'RTK float',6:'estimated',7:'manual',8:'simulation'};
const $ = id => document.getElementById(id);
const NS = 'http://www.w3.org/2000/svg';
let S = null;

function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
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
function fmtStep(v, step) {
  const s = v.toFixed(step >= 1 ? 0 : step >= 0.1 ? 1 : 2);
  return Number(s) === 0 ? '0' : s;
}
function fmtM(v) { return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2); }

function status(s) {
  if (!s.last) return ['critical', '!', 'No data', 'no GPS datagrams yet'];
  if (s.live && s.age_s > 3) return ['critical', '!', 'No data', 'last fix ' + s.age_s.toFixed(0) + ' s ago'];
  if (!s.last.valid) return ['warning', '?', 'No fix', 'receiving, ' + s.last.sats + ' sats'];
  return ['good', '✓', 'Fix', QUAL[s.last.q] || ('quality ' + s.last.q)];
}

function tiles(s) {
  $('src').textContent = (s.live ? 'live · ' : 'replay · ') + s.source;
  const [cls, icon, label, sub] = status(s);
  $('t-fix').innerHTML = '<span class="dot" style="background:var(--' + cls + ')">' + icon + '</span>' + label;
  $('t-fix-s').textContent = sub;
  $('t-sats').textContent = s.last ? s.last.sats : '—';
  $('t-hdop').textContent = s.last && s.last.valid ? s.last.hdop.toFixed(2) : '—';
  $('t-hdop-s').textContent = s.last && s.last.valid ? (s.last.hdop < 2 ? 'good geometry' : s.last.hdop < 5 ? 'moderate' : 'poor') : '';
  $('t-rate').textContent = s.rate_hz ? s.rate_hz.toFixed(2) + ' Hz' : '—';
  $('t-age').textContent = s.live && s.age_s != null ? 'last fix ' + s.age_s.toFixed(1) + ' s ago' : (s.live ? '' : 'over the last 10 s of the log');
  $('t-valid').textContent = s.n ? s.n_valid + ' / ' + s.n : '—';
  $('t-rx').textContent = s.live ? 'datagrams ' + s.rx + ', lost ' + s.lost + ', bad ' + s.bad : 'fixes in the log';
  if (s.spread) {
    $('t-cep').textContent = fmtM(s.spread.cep50) + ' m';
    $('t-cep-s').textContent = 'CEP50 · 2DRMS ' + fmtM(s.spread.drms2) + ' m (' + s.spread.n + ' fixes)';
  } else { $('t-cep').textContent = '—'; $('t-cep-s').textContent = 'needs ≥ 10 valid fixes'; }
  if (s.last && s.last.valid) {
    $('pos').textContent = s.last.lat.toFixed(7) + '°, ' + s.last.lon.toFixed(7) + '° · alt ' + s.last.alt.toFixed(1) + ' m · ' + (s.last.speed * 3.6).toFixed(1) + ' km/h';
    const o = $('osm');
    o.href = 'https://www.openstreetmap.org/?mlat=' + s.last.lat.toFixed(7) + '&mlon=' + s.last.lon.toFixed(7) + '#map=19/' + s.last.lat.toFixed(6) + '/' + s.last.lon.toFixed(6);
    o.hidden = false;
  } else { $('pos').textContent = 'no valid position'; $('osm').hidden = true; }
  $('clear').hidden = !s.live;
  $('tbl').innerHTML = s.recent.map(f => '<tr><td>' + utc(f.utc_ms) + '</td><td>' + f.t.toFixed(1) + '</td><td>' +
    (f.valid ? f.lat.toFixed(7) : '—') + '</td><td>' + (f.valid ? f.lon.toFixed(7) : '—') + '</td><td>' + f.sats +
    '</td><td>' + (f.valid ? f.hdop.toFixed(2) : '—') + '</td><td>' + (QUAL[f.q] || f.q) + '</td><td>' + (f.valid ? 'yes' : 'no') + '</td></tr>').join('');
}

// ---- track: equal-aspect east/north metres, nice grid, scale in the axis labels
let TRK = null;
function drawTrack(s) {
  const svg = $('trk'); svg.replaceChildren();
  const W = 600, H = 600, m = {l: 52, r: 16, t: 14, b: 40};
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  const pts = s.track;
  if (!pts.length) {
    el('text', {x: W / 2, y: H / 2, 'text-anchor': 'middle'}, svg).textContent = 'Waiting for a valid fix…';
    TRK = null; return;
  }
  let e0 = Math.min(...pts.map(p => p[0])), e1 = Math.max(...pts.map(p => p[0]));
  let n0 = Math.min(...pts.map(p => p[1])), n1 = Math.max(...pts.map(p => p[1]));
  let span = Math.max(e1 - e0, n1 - n0, 4) * 1.15;          // at least a 4 m box
  const ec = (e0 + e1) / 2, nc = (n0 + n1) / 2;
  const k = Math.min(pw, ph) / span;
  const X = e => m.l + pw / 2 + (e - ec) * k, Y = n => m.t + ph / 2 - (n - nc) * k;
  const step = nice(span, 6);
  for (let v = Math.ceil((ec - pw / 2 / k) / step) * step; v <= ec + pw / 2 / k; v += step) {
    el('line', {x1: X(v), x2: X(v), y1: m.t, y2: m.t + ph, class: 'gl'}, svg);
    el('text', {x: X(v), y: H - m.b + 16, 'text-anchor': 'middle'}, svg).textContent = fmtStep(v, step);
  }
  for (let v = Math.ceil((nc - ph / 2 / k) / step) * step; v <= nc + ph / 2 / k; v += step) {
    el('line', {x1: m.l, x2: m.l + pw, y1: Y(v), y2: Y(v), class: 'gl'}, svg);
    el('text', {x: m.l - 6, y: Y(v) + 4, 'text-anchor': 'end'}, svg).textContent = fmtStep(v, step);
  }
  el('rect', {x: m.l, y: m.t, width: pw, height: ph, fill: 'none', class: 'ax'}, svg);
  el('text', {x: m.l + pw / 2, y: H - 6, 'text-anchor': 'middle'}, svg).textContent = 'east (m) · grid ' + fmtStep(step, step) + ' m';
  const yl = el('text', {x: 14, y: m.t + ph / 2, 'text-anchor': 'middle', transform: 'rotate(-90 14 ' + (m.t + ph / 2) + ')'}, svg);
  yl.textContent = 'north (m)';
  el('text', {x: m.l + pw - 8, y: m.t + 16, 'text-anchor': 'end'}, svg).textContent = 'N ↑';
  // one polyline per run of consecutive valid fixes: a fix loss is a gap, not a straight line
  let seg = [];
  const flush = () => { if (seg.length > 1) el('polyline', {points: seg.join(' '), class: 'trk'}, svg); seg = []; };
  for (const p of pts) {
    if (p[5]) flush();
    seg.push(X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1));
  }
  flush();
  el('circle', {cx: X(pts[0][0]), cy: Y(pts[0][1]), r: 5, class: 'start'}, svg);
  const L = pts[pts.length - 1];
  el('circle', {cx: X(L[0]), cy: Y(L[1]), r: 6, class: 'cur'}, svg);
  TRK = {pts, X, Y, hov: el('circle', {r: 7, class: 'hov', visibility: 'hidden'}, svg)};
}

// ---- time series: one measure per chart (no dual axis)
const TS = {};
function drawSeries(id, s, idx, fmt, yMinSpan) {
  const svg = $(id); svg.replaceChildren();
  const W = 520, H = 170, m = {l: 36, r: 12, t: 10, b: 26};
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  const d = s.series.filter(r => idx === 1 || r[3]);       // HDOP only where the fix is valid
  if (!s.series.length) { TS[id] = null; return; }
  const t1 = s.series[s.series.length - 1][0], t0 = Math.max(s.series[0][0], t1 - 600);
  const X = t => m.l + (t - t0) / Math.max(t1 - t0, 1) * pw;
  const vals = d.map(r => r[idx]);
  let lo = 0, hi = Math.max(yMinSpan, ...vals) * 1.1;
  const Y = v => m.t + ph - (v - lo) / (hi - lo) * ph;
  // no-fix bands
  let start = null;
  s.series.forEach((r, i) => {
    if (!r[3] && start === null) start = r[0];
    if ((r[3] || i === s.series.length - 1) && start !== null) {
      el('rect', {x: X(start), y: m.t, width: Math.max(X(r[0]) - X(start), 2), height: ph, class: 'nofix'}, svg);
      start = null;
    }
  });
  const ystep = nice(hi - lo, 4);
  for (let v = 0; v <= hi; v += ystep) {
    el('line', {x1: m.l, x2: m.l + pw, y1: Y(v), y2: Y(v), class: 'gl'}, svg);
    el('text', {x: m.l - 6, y: Y(v) + 4, 'text-anchor': 'end'}, svg).textContent = fmt(v);
  }
  el('line', {x1: m.l, x2: m.l + pw, y1: m.t + ph, y2: m.t + ph, class: 'ax'}, svg);
  const xstep = nice(Math.max(t1 - t0, 1), 5);
  for (let t = Math.ceil(t0 / xstep) * xstep; t <= t1; t += xstep) {
    el('text', {x: X(t), y: H - 8, 'text-anchor': 'middle'}, svg).textContent = t.toFixed(0) + ' s';
  }
  // the line breaks where the fix is lost (HDOP) so a gap is never drawn as data
  let seg = [];
  const flush = () => { if (seg.length > 1) el('polyline', {points: seg.join(' '), class: 'ln'}, svg); seg = []; };
  s.series.forEach(r => {
    if (r[0] < t0) return;
    if (idx === 2 && !r[3]) { flush(); return; }
    seg.push(X(r[0]).toFixed(1) + ',' + Y(r[idx]).toFixed(1));
  });
  flush();
  TS[id] = {rows: s.series.filter(r => r[0] >= t0), X, Y, idx, fmt, m, ph,
            xh: el('line', {y1: m.t, y2: m.t + ph, class: 'xh', visibility: 'hidden'}, svg),
            dot: el('circle', {r: 4, class: 'cur', visibility: 'hidden'}, svg)};
}

function svgPoint(svg, ev) {
  const p = svg.createSVGPoint(); p.x = ev.clientX; p.y = ev.clientY;
  return p.matrixTransform(svg.getScreenCTM().inverse());
}
function placeTip(tip, card, ev, html) {
  tip.innerHTML = html; tip.style.display = 'block';
  const r = card.getBoundingClientRect();
  let x = ev.clientX - r.left + 12, y = ev.clientY - r.top + 12;
  if (x + tip.offsetWidth > r.width - 4) x = ev.clientX - r.left - tip.offsetWidth - 12;
  tip.style.left = x + 'px'; tip.style.top = y + 'px';
}
$('trk').addEventListener('pointermove', ev => {
  if (!TRK) return;
  const q = svgPoint($('trk'), ev);
  let best = null, bd = 1e9;
  for (const p of TRK.pts) { const d = Math.hypot(TRK.X(p[0]) - q.x, TRK.Y(p[1]) - q.y); if (d < bd) { bd = d; best = p; } }
  if (!best || bd > 30) { TRK.hov.setAttribute('visibility', 'hidden'); $('tip-trk').style.display = 'none'; return; }
  TRK.hov.setAttribute('cx', TRK.X(best[0])); TRK.hov.setAttribute('cy', TRK.Y(best[1])); TRK.hov.setAttribute('visibility', 'visible');
  placeTip($('tip-trk'), $('c-trk'), ev, 'UTC ' + utc(best[4]) + ' · t ' + best[2].toFixed(1) + ' s<br>E ' + fmtM(best[0]) + ' m, N ' + fmtM(best[1]) + ' m · ' + best[3] + ' sats');
});
$('trk').addEventListener('pointerleave', () => { if (TRK) TRK.hov.setAttribute('visibility', 'hidden'); $('tip-trk').style.display = 'none'; });
for (const [id, label] of [['sats', 'Satellites'], ['hdop', 'HDOP']]) {
  $(id).addEventListener('pointermove', ev => {
    const T = TS[id]; if (!T || !T.rows.length) return;
    const q = svgPoint($(id), ev);
    let best = T.rows[0];
    for (const r of T.rows) if (Math.abs(T.X(r[0]) - q.x) < Math.abs(T.X(best[0]) - q.x)) best = r;
    const x = T.X(best[0]);
    T.xh.setAttribute('x1', x); T.xh.setAttribute('x2', x); T.xh.setAttribute('visibility', 'visible');
    const ok = T.idx === 1 || best[3];
    if (ok) { T.dot.setAttribute('cx', x); T.dot.setAttribute('cy', T.Y(best[T.idx])); }
    T.dot.setAttribute('visibility', ok ? 'visible' : 'hidden');
    placeTip($('tip-' + id), $('c-' + id), ev, 't ' + best[0].toFixed(1) + ' s<br>' + label + ' ' + (ok ? T.fmt(best[T.idx]) : '—') + (best[3] ? '' : ' · no fix'));
  });
  $(id).addEventListener('pointerleave', () => { const T = TS[id]; if (T) { T.xh.setAttribute('visibility', 'hidden'); T.dot.setAttribute('visibility', 'hidden'); } $('tip-' + id).style.display = 'none'; });
}
$('clear').addEventListener('click', () => fetch('/clear', {method: 'POST'}).then(refresh));

// The page keeps the track; each refresh brings only the points it does not have
// (id > LAST_ID). A new epoch (track cleared on the rover) or a gap resends it all.
let TRACK = [], EPOCH = 0, LAST_ID = 0;
const MAX_TRACK = 20000;
async function refresh() {
  try {
    const r = await fetch('/state?epoch=' + EPOCH + '&since=' + LAST_ID, {cache: 'no-store'});
    S = await r.json();
  } catch (e) { $('src').textContent = 'gps_view.py not reachable'; return; }
  if (S.full) TRACK = S.track;
  else for (const p of S.track) TRACK.push(p);
  if (TRACK.length > MAX_TRACK) TRACK = TRACK.slice(TRACK.length - MAX_TRACK);
  EPOCH = S.epoch;
  LAST_ID = TRACK.length ? TRACK[TRACK.length - 1][6] : 0;
  S.track = TRACK;
  tiles(S);
  drawTrack(S);
  drawSeries('sats', S, 1, v => v.toFixed(0), 12);
  drawSeries('hdop', S, 2, v => v.toFixed(1), 3);
}
refresh();
setInterval(() => { if (S && S.live) refresh(); }, 1000);
</script>
</body>
</html>
'''

if __name__ == '__main__':
    main()

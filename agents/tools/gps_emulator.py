#!/usr/bin/env python3
"""GPS emulator: NMEA RMC + GGA for testing the GPS agent and gps_view.py indoors.

    # NMEA into a pty, read by olympus-pose (io.gps_port = /tmp/gps_emu)
    gps_emulator.py --link /tmp/gps_emu
    # skip the app: send the datagrams olympus-pose would send, straight to gps_view.py
    gps_emulator.py --udp 127.0.0.1:47003
    # a 20 s fix loss after 90 s, faster than real time into a file
    gps_emulator.py --out /tmp/gps.nmea --no-fix 90:110 --duration 300

Scenario: 60 s at rest (a static receiver scatters by ~1-3 m: that cloud is what
the "Scatter" tile measures), then a 20 m square walked at 0.5 m/s, repeated.
The position error is a slow random walk plus white noise, like a real
single-frequency receiver; satellites and HDOP drift slowly.
"""
import argparse
import math
import os
import random
import socket
import struct
import sys
import time
import binascii

R_EARTH = 6371000.0


def nmea(body):
    cs = 0
    for c in body.encode():
        cs ^= c
    return '$%s*%02X\r\n' % (body, cs)


def latlon_fields(lat, lon):
    def dm(v, deg_digits):
        a = abs(v)
        d = int(a)
        return '%0*d%08.5f' % (deg_digits, d, (a - d) * 60)
    return dm(lat, 2), 'N' if lat >= 0 else 'S', dm(lon, 3), 'E' if lon >= 0 else 'W'


def path(t, side=20.0, v=0.5, rest=60.0):
    """East/north metres of the true position at time t."""
    if t < rest:
        return 0.0, 0.0
    s = ((t - rest) * v) % (4 * side)
    k, r = int(s // side), s % side
    return [(r, 0.0), (side, r), (side - r, side), (0.0, side - r)][k]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--link', help='create a pty and symlink it here (live NMEA)')
    ap.add_argument('--out', help='write NMEA to this file as fast as possible')
    ap.add_argument('--udp', help='HOST:PORT: send OGP1 datagrams directly (live, no app)')
    ap.add_argument('--origin', default='9.85560,-83.91250', help='LAT,LON of the start (default: TEC Cartago)')
    ap.add_argument('--alt', type=float, default=1435.0)
    ap.add_argument('--rate', type=float, default=1.0, help='fixes per second')
    ap.add_argument('--duration', type=float, default=0.0, help='s (0 = until Ctrl-C; --out needs it)')
    ap.add_argument('--no-fix', default='', help='START:END s without a fix (sentences keep coming)')
    ap.add_argument('--speedup', type=float, default=1.0, help='live modes: play the scenario N times faster')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    if sum(bool(x) for x in (a.link, a.out, a.udp)) != 1:
        sys.exit('gps_emulator: use exactly one of --link, --out, --udp')
    if a.out and a.duration <= 0:
        sys.exit('gps_emulator: --out needs --duration')
    lat0, lon0 = (float(x) for x in a.origin.split(','))
    nofix = tuple(float(x) for x in a.no_fix.split(':')) if a.no_fix else None
    rnd = random.Random(a.seed)

    out = sock = None
    if a.link:
        import pty
        import tty
        master, slave = pty.openpty()
        tty.setraw(slave)
        if os.path.lexists(a.link):
            os.remove(a.link)
        os.symlink(os.ttyname(slave), a.link)
        out = os.fdopen(master, 'wb', buffering=0)
        print('gps_emulator: %s -> %s, NMEA at %.1f Hz' % (a.link, os.ttyname(slave), a.rate))
    elif a.out:
        out = open(a.out, 'wb')
    else:
        host, _, port = a.udp.rpartition(':')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = (host, int(port))
        print('gps_emulator: OGP1 datagrams to %s:%d at %.1f Hz' % (host, int(port), a.rate))

    dt = 1.0 / a.rate
    k_lat = 180 / math.pi / R_EARTH
    k_lon = k_lat / math.cos(math.radians(lat0))
    walk_e = walk_n = 0.0
    sats, hdop = 9.0, 1.0
    t_wall0 = time.monotonic()
    utc0 = (time.time() % 86400)
    k = n_valid = 0
    try:
        while a.duration <= 0 or k * dt < a.duration:
            t = k * dt
            e, n = path(t)
            walk_e = 0.995 * walk_e + rnd.gauss(0, 0.12)     # correlated error, ~1.2 m
            walk_n = 0.995 * walk_n + rnd.gauss(0, 0.12)
            e += walk_e + rnd.gauss(0, 0.4)
            n += walk_n + rnd.gauss(0, 0.4)
            sats = min(14.0, max(4.0, sats + rnd.gauss(0, 0.15)))
            hdop = min(3.0, max(0.7, hdop + rnd.gauss(0, 0.02)))
            fix = not (nofix and nofix[0] <= t < nofix[1])
            lat, lon = lat0 + n * k_lat, lon0 + e * k_lon
            e1, n1 = path(t + 1.0)
            e0, n0 = path(t)
            speed = math.hypot(e1 - e0, n1 - n0)
            course = math.degrees(math.atan2(e1 - e0, n1 - n0)) % 360 if speed > 0 else 0.0
            utc_s = (utc0 + t) % 86400
            ms = int(round(utc_s * 1000)) % 86400000
            hhmmss = '%02d%02d%05.2f' % (ms // 3600000, ms // 60000 % 60, ms % 60000 / 1000)
            ns = int(round(sats)) if fix else rnd.randint(0, 3)
            if a.link:
                target = t_wall0 + t / a.speedup
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)
            if sock:
                if fix:
                    n_valid += 1
                d = struct.pack('<4sIQ6dI4BI', b'OGP1', k + 1, time.monotonic_ns(), lat if fix else 0.0,
                                lon if fix else 0.0, a.alt if fix else 0.0, hdop if fix else 99.9,
                                speed, course, ms, 1 if fix else 0, ns, 1 if fix else 0, 0, n_valid)
                sock.sendto(d + struct.pack('<I', binascii.crc32(d) & 0xFFFFFFFF), dest)
                target = t_wall0 + (t + dt) / a.speedup
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)
            else:
                la, ns_, lo, ew = latlon_fields(lat, lon)
                date = time.strftime('%d%m%y', time.gmtime())
                if fix:
                    rmc = 'GPRMC,%s,A,%s,%s,%s,%s,%.2f,%.1f,%s,,,A' % (hhmmss, la, ns_, lo, ew,
                                                                     speed / 0.514444, course, date)
                    gga = 'GPGGA,%s,%s,%s,%s,%s,1,%02d,%.2f,%.1f,M,-1.0,M,,' % (hhmmss, la, ns_, lo, ew,
                                                                               ns, hdop, a.alt)
                else:
                    rmc = 'GPRMC,%s,V,,,,,,,%s,,,N' % (hhmmss, date)
                    gga = 'GPGGA,%s,,,,,0,%02d,99.99,,,,,,' % (hhmmss, ns)
                out.write((nmea(rmc) + nmea(gga) + nmea('GPGSA,A,%d,,,,,,,,,,,,,,%.2f,' % (3 if fix else 1,
                                                                                         hdop))).encode())
            k += 1
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        if out:
            out.close()
        if a.link and os.path.islink(a.link):
            os.remove(a.link)
    print('gps_emulator: %d epochs' % k)


if __name__ == '__main__':
    main()

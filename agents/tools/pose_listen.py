#!/usr/bin/env python3
"""Prints the olympus-pose UDP datagrams (DESIGN.md §11.1, version "OPE1").

    pose_listen.py [--port 47001] [--all]

By default prints one summary line per second; --all prints every datagram.
Checks the magic and the CRC-32 of each datagram.
"""
import argparse
import math
import socket
import struct
import time
import binascii

FMT = '<4sIQQIHH3d2d4d2dII'          # 128 bytes, see pose_dgram.h
SIZE = struct.calcsize(FMT)
FLAGS = ['VALID', 'STILL', 'SLIP', 'STALE', 'GAP', 'IMU_INVALID', 'ROLLOVER', 'LLC_RESET',
         'RESUMED', 'REREF']
MODES = ['WAIT_DATA', 'INITIALIZING', 'RUNNING']


def decode(data):
    if len(data) != SIZE:
        raise ValueError('length %d' % len(data))
    f = struct.unpack(FMT, data)
    if f[0] != b'OPE1':
        raise ValueError('magic %r' % f[0])
    if binascii.crc32(data[:124]) & 0xFFFFFFFF != f[-1]:
        raise ValueError('bad CRC')
    keys = ['magic', 'seq', 't_pub_ns', 't_rx_ns', 't_llc_ms', 'mode', 'status',
            'x', 'y', 'theta', 'omega', 'bias', 'Pxx', 'Pyy', 'Pxy', 'Ptt', 's', 'delta', 'res', 'crc']
    return dict(zip(keys, f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=47001)
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', a.port))
    print('listening on UDP %d' % a.port)
    last_print, n, bad, last_seq, lost = 0.0, 0, 0, None, 0
    while True:
        data = s.recv(2048)
        try:
            d = decode(data)
        except ValueError as e:
            bad += 1
            print('bad datagram: %s' % e)
            continue
        n += 1
        if last_seq is not None and d['seq'] != last_seq + 1:
            lost += (d['seq'] - last_seq - 1) & 0xFFFFFFFF
        last_seq = d['seq']
        now = time.monotonic()
        if a.all or now - last_print >= 1.0:
            last_print = now
            flags = '|'.join(f for i, f in enumerate(FLAGS) if d['status'] >> i & 1) or '-'
            mode = MODES[d['mode']] if d['mode'] < len(MODES) else str(d['mode'])
            age_ms = (time.monotonic_ns() - d['t_rx_ns']) / 1e6
            print('#%-7d x %8.4f y %8.4f th %8.2f deg  s %7.3f m  bias %.5f  sigma_xy %.3f m  '
                  '%s %s  age %.1f ms  (rx %d, lost %d, bad %d)'
                  % (d['seq'], d['x'], d['y'], math.degrees(d['theta']), d['s'], d['bias'],
                     math.sqrt(max(d['Pxx'] + d['Pyy'], 0)), mode, flags, age_ms, n, lost, bad))


if __name__ == '__main__':
    main()

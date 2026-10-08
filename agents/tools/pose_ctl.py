#!/usr/bin/env python3
"""Sends a command to the olympus-pose control port (DESIGN.md §11.3).

    pose_ctl.py MARK start of UMBmark CW run 1
    pose_ctl.py RESET_POSE 0 0 0
    pose_ctl.py SUSPEND estimation        (through the CMAES AMS)
    pose_ctl.py RESUME estimation
    pose_ctl.py STATS

MARK writes an event into the logs, so test runs can be cut out afterwards.
"""
import socket
import sys


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    port = 47002
    cmd = ' '.join(sys.argv[1:])
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    s.sendto(cmd.encode(), ('127.0.0.1', port))
    try:
        print(s.recv(2048).decode().rstrip())
    except socket.timeout:
        print('no reply from olympus-pose on 127.0.0.1:%d (is it running?)' % port)
        sys.exit(1)


if __name__ == '__main__':
    main()

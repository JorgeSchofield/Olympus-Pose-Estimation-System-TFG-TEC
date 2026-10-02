#!/usr/bin/env python3
"""Spike S-1 (DESIGN.md §18): can rover_bridge talk to the LLC through a pty?

The llcmux design (DESIGN.md §6) hands olympus_hlc a pseudo-terminal instead of
the real Arduino port. This script checks, on the RPi 5 image, that the
unmodified rover_bridge extension can open a pty and exchange MSM traffic over
it. It does NOT touch the real Arduino port, so it is safe to run at any time.

It creates a pty pair, plays a fake LLC on the master side and drives the slave
side with rover_bridge.Rover exactly as olympus_hlc does:

  1. open    - Rover(<pty>, 115200) succeeds (TIOCEXCL, flock, termios, baud)
  2. command - send_command("PING") returns "PONG", skipping RAW:/TLM: noise
  3. tlm     - recv_tlm() returns an unsolicited TLM: line
  4. excl    - a second Rover() on the same pty is refused (exclusive access)

Usage (on the rover, as root):  python3 spike_pty_check.py
Exit code 0 when every check passes.
"""

import os
import select
import sys
import threading
import time

LINK = "/tmp/llc_spike"
TLM_LINE = b"TLM:NORMAL:0:12345ms:15800mV:420mA\n"


class FakeLlc(threading.Thread):
    """Answers on the master side like firmware v2.20 would."""

    def __init__(self, master_fd):
        super().__init__(daemon=True)
        self.fd = master_fd
        self.rx = b""
        self.commands = []
        self.stop = False

    def write(self, data):
        os.write(self.fd, data)

    def run(self):
        while not self.stop:
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, 256)
            except OSError:
                # EIO while no slave is open; keep waiting
                time.sleep(0.05)
                continue
            self.rx += chunk
            while b"\n" in self.rx:
                line, self.rx = self.rx.split(b"\n", 1)
                line = line.strip(b"\r")
                self.commands.append(line)
                if line == b"PING":
                    # Async noise first, as the real link interleaves it
                    self.write(b"RAW:1000:12:-3:16384:1:-2:5:2400:-2398\n")
                    self.write(TLM_LINE)
                    self.write(b"PONG\n")
                else:
                    self.write(b"ERR:UNKNOWN\n")


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))
    return ok


def main():
    try:
        import rover_bridge
    except ImportError as exc:
        print(f"rover_bridge not importable here ({exc}); run this on the rover image.")
        return 2

    master, slave = os.openpty()
    slave_name = os.ttyname(slave)
    if os.path.lexists(LINK):
        os.remove(LINK)
    os.symlink(slave_name, LINK)
    print(f"pty slave {slave_name} linked as {LINK}")

    llc = FakeLlc(master)
    llc.start()
    results = []

    # 1. open (Rover() sleeps 2 s internally, like after a real Arduino reset)
    rover = None
    try:
        rover = rover_bridge.Rover(LINK, 115200)
        results.append(check("open", True))
    except Exception as exc:  # noqa: BLE001 - report whatever the bridge raises
        results.append(check("open", False, repr(exc)))

    if rover is not None:
        # 2. command round trip through the noise
        try:
            resp = rover.send_command("PING")
            results.append(check("command", resp == "PONG", f"got {resp!r}"))
        except Exception as exc:  # noqa: BLE001
            results.append(check("command", False, repr(exc)))

        # 3. unsolicited telemetry
        llc.write(TLM_LINE)
        tlm = None
        for _ in range(5):
            tlm = rover.recv_tlm()
            if tlm:
                break
        results.append(check("tlm", bool(tlm) and tlm.startswith("TLM:"), f"got {tlm!r}"))

        # 4. exclusivity: a second opener must be refused
        try:
            rover_bridge.Rover(LINK, 115200)
            results.append(check("excl", False, "second open was accepted"))
        except Exception as exc:  # noqa: BLE001
            results.append(check("excl", True, f"refused: {exc}"))

    llc.stop = True
    os.remove(LINK)
    print(f"commands seen by the fake LLC: {llc.commands}")
    ok = all(results) and len(results) == 4
    print("SPIKE S-1:", "PASS - the llcmux pty approach is viable" if ok else "FAIL - see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

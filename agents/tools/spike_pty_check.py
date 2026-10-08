#!/usr/bin/env python3
"""Spike S-1 (DESIGN.md §18): can rover_bridge talk to the LLC through a pty?

The llcmux design (DESIGN.md §6) hands olympus_hlc a pseudo-terminal instead of
the real Arduino port. This script checks, on the RPi 5 image, that the
unmodified rover_bridge extension can open a pty and exchange MSM traffic over
it. It does NOT touch the real Arduino port, so it is safe to run at any time,
on the rover or on any RPi 5 with the same image.

It creates a pty pair, plays a fake LLC on the master side and drives the slave
side with rover_bridge.Rover exactly as olympus_hlc does:

  1. open    - Rover(<pty>, 115200) succeeds (TIOCEXCL, flock, termios, baud)
  2. command - send_command("PING") returns "PONG", skipping RAW:/TLM: noise
  3. tlm     - recv_tlm() returns an unsolicited TLM: line
  4. excl    - a second Rover() on the same pty is refused (exclusive access)

The fake LLC runs in a separate PROCESS, like the real llcmux. It cannot be a
thread: rover_bridge is a Rust extension that keeps Python's GIL while
send_command() waits for the reply, so a fake LLC thread in the same process
would only get to answer after the bridge had already timed out (that was the
failure mode of the first version of this script).

Usage (as root):  python3 spike_pty_check.py
Exit code 0 when every check passes.
"""

import os
import select
import signal
import sys
import time

LINK = "/tmp/llc_spike"
TLM_LINE = b"TLM:NORMAL:0:12345ms:15800mV:420mA\n"
TLM_PERIOD_S = 0.2        # unsolicited telemetry, like the firmware (faster, to keep the test short)


def fake_llc(master_fd, log_fd):
    """Child process: answers like firmware v2.20 and emits periodic TLM."""
    rx = b""
    next_tlm = time.monotonic() + TLM_PERIOD_S
    while True:
        timeout = max(0.0, next_tlm - time.monotonic())
        ready, _, _ = select.select([master_fd], [], [], timeout)
        if time.monotonic() >= next_tlm:
            try:
                os.write(master_fd, TLM_LINE)
            except OSError:
                pass
            next_tlm += TLM_PERIOD_S
        if not ready:
            continue
        try:
            chunk = os.read(master_fd, 256)
        except OSError:
            time.sleep(0.05)          # EIO while no slave is open
            continue
        rx += chunk
        while b"\n" in rx:
            line, rx = rx.split(b"\n", 1)
            line = line.strip(b"\r")
            os.write(log_fd, line + b"\n")
            if line == b"PING":
                # Async noise first, as the real link interleaves it
                os.write(master_fd, b"RAW:1000:12:-3:16384:1:-2:5:2400:-2398\n")
                os.write(master_fd, TLM_LINE)
                os.write(master_fd, b"PONG\n")
            else:
                os.write(master_fd, b"ERR:UNKNOWN\n")


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

    log_r, log_w = os.pipe()
    pid = os.fork()
    if pid == 0:                      # child: the fake LLC
        os.close(log_r)
        os.close(slave)
        try:
            fake_llc(master, log_w)
        finally:
            os._exit(0)
    os.close(log_w)
    os.close(master)                  # the parent only uses the slave side

    results = []
    rover = None
    try:
        # 1. open (Rover() sleeps 2 s internally, like after a real Arduino reset)
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

            # 3. unsolicited telemetry (recv_tlm reads one line per call, 50 ms max)
            tlm = None
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not tlm:
                tlm = rover.recv_tlm()
            results.append(check("tlm", bool(tlm) and tlm.startswith("TLM:"), f"got {tlm!r}"))

            # 4. exclusivity: a second opener must be refused
            try:
                rover_bridge.Rover(LINK, 115200)
                results.append(check("excl", False, "second open was accepted"))
            except Exception as exc:  # noqa: BLE001
                results.append(check("excl", True, f"refused: {exc}"))
    finally:
        os.kill(pid, signal.SIGTERM)
        os.waitpid(pid, 0)
        os.remove(LINK)

    seen = b""
    while True:
        ready, _, _ = select.select([log_r], [], [], 0)
        if not ready:
            break
        chunk = os.read(log_r, 4096)
        if not chunk:
            break
        seen += chunk
    print(f"commands seen by the fake LLC: {seen.split()}")

    ok = all(results) and len(results) == 4
    print("SPIKE S-1:", "PASS - the llcmux pty approach is viable" if ok else "FAIL - see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

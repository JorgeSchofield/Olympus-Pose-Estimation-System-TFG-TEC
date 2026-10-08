#!/usr/bin/env python3
"""Spike S-1 (DESIGN.md Â§18): can rover_bridge talk to the LLC through a pty?

The llcmux design (DESIGN.md Â§6) hands olympus_hlc a pseudo-terminal instead of
the real Arduino port. This script checks, on the RPi 5 image, that the
unmodified rover_bridge extension can open a pty and exchange MSM traffic over
it. It does NOT touch the real Arduino port, so it is safe to run at any time,
on the rover or on any RPi 5 with the same image.

A fake LLC runs in a separate process on the pty master and answers like
firmware v2.20. The checks:

  0. pyserial - baseline round trip with plain pyserial (no rover_bridge)
  1. open     - rover_bridge.Rover(<pty>, 115200) succeeds
  2. command  - send_command("PING") returns "PONG", skipping RAW:/TLM: noise
  3. tlm      - recv_tlm() returns an unsolicited TLM: line
  4. excl     - a second Rover() on the same pty is refused (exclusive access)

Every event is timestamped (ms since start) on both sides, the slave's termios
flags are printed after the bridge opens the port, and anything left unread in
the pty after a failure is dumped. Paste the whole output when reporting.

Usage (as root):  python3 spike_pty_check.py
Exit code 0 when checks 1-4 pass.
"""

import os
import select
import signal
import sys
import termios
import time

VERSION = "v4 (diagnostic)"
LINK = "/tmp/llc_spike"
TLM_LINE = b"TLM:NORMAL:0:12345ms:15800mV:420mA\n"
TLM_PERIOD_S = 1.0        # unsolicited telemetry, like the firmware (~1 Hz)
T0 = time.monotonic()


def ms():
    return f"{(time.monotonic() - T0) * 1000:8.1f} ms"


def fake_llc(master_fd, log_fd):
    """Child process: answers like firmware v2.20 and emits periodic TLM."""
    def log(text):
        os.write(log_fd, f"{ms()}  [fake LLC] {text}\n".encode())

    rx = b""
    next_tlm = time.monotonic() + TLM_PERIOD_S
    while True:
        timeout = max(0.0, next_tlm - time.monotonic())
        ready, _, _ = select.select([master_fd], [], [], timeout)
        if time.monotonic() >= next_tlm:
            try:
                os.write(master_fd, TLM_LINE)
                log("sent periodic TLM")
            except OSError as exc:
                log(f"periodic TLM write failed: {exc}")
            next_tlm += TLM_PERIOD_S
        if not ready:
            continue
        try:
            chunk = os.read(master_fd, 256)
        except OSError as exc:
            log(f"read error {exc}")
            time.sleep(0.05)
            continue
        log(f"read {chunk!r}")
        rx += chunk
        while b"\n" in rx:
            line, rx = rx.split(b"\n", 1)
            line = line.strip(b"\r")
            if line == b"PING":
                os.write(master_fd, b"RAW:1000:12:-3:16384:1:-2:5:2400:-2398\n")
                os.write(master_fd, TLM_LINE)
                os.write(master_fd, b"PONG\n")
                log("replied RAW + TLM + PONG")
            elif line:
                os.write(master_fd, b"ERR:UNKNOWN\n")
                log(f"replied ERR:UNKNOWN to {line!r}")


def check(name, ok, detail=""):
    print(f"{ms()}  [{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))
    return ok


def termios_summary(fd):
    iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(fd)
    flags = []
    for name, value, field in (("ICANON", termios.ICANON, lflag), ("ECHO", termios.ECHO, lflag),
                               ("ISIG", termios.ISIG, lflag), ("ICRNL", termios.ICRNL, iflag),
                               ("IXON", termios.IXON, iflag), ("OPOST", termios.OPOST, oflag),
                               ("CRTSCTS", termios.CRTSCTS, cflag), ("CLOCAL", termios.CLOCAL, cflag)):
        flags.append(f"{name}={'on' if field & value else 'off'}")
    flags.append(f"VMIN={cc[termios.VMIN] if isinstance(cc[termios.VMIN], int) else ord(cc[termios.VMIN])}")
    flags.append(f"VTIME={cc[termios.VTIME] if isinstance(cc[termios.VTIME], int) else ord(cc[termios.VTIME])}")
    return " ".join(flags)


def drain(fd, label):
    """Print whatever is still queued on fd (non-blocking)."""
    data = b""
    while True:
        ready, _, _ = select.select([fd], [], [], 0)
        if not ready:
            break
        try:
            chunk = os.read(fd, 4096)
        except OSError:
            break
        if not chunk:
            break
        data += chunk
    print(f"{ms()}  {label}: {data!r}")


def main():
    print(f"spike_pty_check {VERSION}")
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
    print(f"{ms()}  pty slave {slave_name} linked as {LINK}")

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
        # 0. baseline with pyserial: proves the pty and the fake LLC on their own
        try:
            import serial
            with serial.Serial(LINK, 115200, timeout=1.0, exclusive=True) as port:
                port.reset_input_buffer()
                t = time.monotonic()
                port.write(b"PING\n")
                port.flush()
                reply = b""
                while b"PONG" not in reply and time.monotonic() - t < 1.0:
                    reply += port.readline()
                check("pyserial", b"PONG" in reply,
                      f"round trip {(time.monotonic() - t) * 1000:.1f} ms, got {reply!r}")
        except Exception as exc:  # noqa: BLE001
            check("pyserial", False, repr(exc))

        # 1. open (Rover() sleeps 2 s internally, like after a real Arduino reset)
        try:
            print(f"{ms()}  calling Rover()")
            rover = rover_bridge.Rover(LINK, 115200)
            results.append(check("open", True))
            print(f"{ms()}  slave termios after open: {termios_summary(slave)}")
        except Exception as exc:  # noqa: BLE001
            results.append(check("open", False, repr(exc)))

        if rover is not None:
            # 2. command round trip through the noise
            try:
                print(f"{ms()}  calling send_command('PING')")
                resp = rover.send_command("PING")
                results.append(check("command", resp == "PONG", f"got {resp!r}"))
            except Exception as exc:  # noqa: BLE001
                results.append(check("command", False, repr(exc)))
                drain(slave, "unread in the pty after the failed command")

            # 3. unsolicited telemetry, read the way olympus_hlc does it: once per
            #    control cycle, so a TLM line has usually arrived in full before
            #    recv_tlm() is called. Calling recv_tlm() back to back does not
            #    work even on the real port: it gives itself 50 ms but each read
            #    waits up to 100 ms, so a line arriving in the second half of a
            #    call loses its first byte and the rest is then discarded (v3
            #    showed this; it is a rover_bridge issue, not a pty one).
            tlm = None
            for _ in range(3):
                time.sleep(TLM_PERIOD_S * 1.2)     # let at least one TLM line arrive
                for _ in range(10):                # drain queued lines, one per call
                    line = rover.recv_tlm()
                    if line:
                        tlm = line
                        break
                if tlm:
                    break
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

    log = b""
    while True:
        ready, _, _ = select.select([log_r], [], [], 0)
        if not ready:
            break
        chunk = os.read(log_r, 65536)
        if not chunk:
            break
        log += chunk
    print("---- fake LLC log ----")
    print(log.decode(errors="replace"), end="")
    print("----------------------")

    ok = all(results) and len(results) == 4
    print("SPIKE S-1:", "PASS - the llcmux pty approach is viable" if ok else "FAIL - see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

# olympus-pose — pose estimation on the Olympus HLC

CMAES multi-agent application that estimates the planar pose of the Olympus rover on the
Raspberry Pi 5, plus `llcmux`, the daemon that shares the Arduino link with the existing
`olympus_hlc`. Design, decisions and validation plan: [DESIGN.md](DESIGN.md) (ARQ-PE-003).

```
LLC (Arduino) ── /dev/arduino_mega_hw ── llcmux ─┬─ /dev/arduino_mega  → olympus_hlc (RAW: lines removed)
                                                 └─ /run/olympus/llc_raw → olympus-pose
olympus-pose:  acquisition → fusion → estimation → communication ← GPS (/dev/ttyAMA0)
                                                         └─ UDP 127.0.0.1:47001, logs, control port 47002
```

## Layout

| Path | What |
|---|---|
| `src/main.c` | Supervisor: configuration, logs, CMAES platform, shutdown |
| `src/agent_*.c` | The five agents (DESIGN.md §7) |
| `src/core/` | Portable C99: RAW and NMEA parsers, `pose.conf` parser, statistics, UDP datagram |
| `src/core/gen/` | **Generated** by MATLAB Coder (`codegen_hlc_core.m` in the simulation repo). Never edit by hand |
| `src/port/` | The only Linux-specific code: serial/pty, UDP, async log writer, memory locking |
| `tools/llcmux/` | LLC link multiplexer (DESIGN.md §6) |
| `tools/llc_emulator.py` | 50 Hz RAW: generator (pty or file) for tests without the rover |
| `tools/pose_listen.py`, `tools/pose_ctl.py` | UDP pose viewer and control-port client |
| `config/pose.conf` | Configuration: model parameters (from `export_pose_conf.m`) + application settings |
| `tests/` | Host unit tests and the MATLAB↔C equivalence vectors |

## Building

**For the rover** — the `olympus-pose` recipe in `CMAES/CMAES_Yocto_layer/meta-olympus-pose`:

```bash
bitbake olympus-pose
```

It installs `/usr/bin/olympus-pose`, `/usr/bin/llcmux`, the Python tools,
`/etc/olympus-pose/pose.conf` and two sysvinit scripts (`llcmux` at S90, `olympus-pose` at
S91). The udev change that renames the real port to `/dev/arduino_mega_hw` is in the same
layer; no file of `olympus-hlc-rpi5` is modified.

**Unit tests on any PC** (no Linux needed — only the portable core):

```bash
cmake -S agents -B build-core -DPE_CORE_ONLY=ON
cmake --build build-core
./build-core/pe_tests
```

Expected: `OK: 68 checks passed`, including the RAW parser fuzz (10⁶ mutations) and the
generated C reproducing MATLAB on 2591 frames with zero difference.

**After changing the model or its parameters** (in MATLAB, simulation repo,
`Simulation Model v2.0`):

```matlab
codegen_hlc_core('<repo>\agents\src\core\gen')     % only if the algorithm changed
export_pose_conf                                    % then paste codegen/pose_model.conf
                                                    % into section 1 of config/pose.conf
```

and regenerate the test vectors with `tests/matlab/export_core_vectors.m`.

## Running

```bash
/etc/init.d/llcmux start          # usually started at boot
/etc/init.d/olympus-pose start
olympus-pose -c /etc/olympus-pose/pose.conf --check     # validate a configuration
pose_listen.py                    # live pose, one line per second
pose_ctl.py MARK start of run 3   # event marker in the logs
pose_ctl.py STATS
```

`olympus_hlc` is started by hand as before; it finds `/dev/arduino_mega` (now the
`llcmux` pty) and needs no change.

Logs: `/var/lib/olympus-pose/runs/<YYYYmmdd-HHMMSS>/` (persistent; `/var/log` is in RAM on the
rover image)

| File | Content |
|---|---|
| `header.txt` | Version, mode, every parameter and the configuration hash |
| `pose.csv` | Every estimate: timestamps, state, covariance, slip indicators, latency, link counters |
| `stats.csv` | Once per second: RSS, CPU, latency p50/p95/max, pose interval, LLC clock scale, per-link delivery and worst gap, agent states |
| `gps.csv` | Every GPS fix (HLC time) |
| `events.csv` | Mode changes, STALE, LLC resets, operator marks, AMS replies |

`llcmux --record-dir DIR` also writes `llc_record.txt` (every LLC line with its HLC arrival
time). Replay it through the app, without drops and reproducibly:

```bash
olympus-pose -c /etc/olympus-pose/pose.conf -r DIR/<run>/llc_record.txt
```

## Testing on a spare Raspberry Pi (no rover)

Run as root (real-time priorities). Each step only needs the previous ones.

1. **Replay** — the whole chain on a file, fast and deterministic:
   ```bash
   llc_emulator.py --out /tmp/umb.txt --prefix --scenario umbmark --loss 0.005 --corrupt-at 100
   olympus-pose -c /etc/olympus-pose/pose.conf -r /tmp/umb.txt
   ```
   Check `events.csv` (INITIALIZING → RUNNING, end of replay) and the last line of
   `pose.csv`: the 2 m square should close to roughly 1–3 % of 8 m, `s_m` = 8.000.
   In replay the `latency_ms` column is not meaningful (it compares recorded arrival times
   with the current clock); latency is measured live (step 2).
2. **Live, OE4 delivery test** (V-OE4-2, 15 min at 50 Hz). In `pose.conf` set
   `io.raw_input = /tmp/llc_emu`, then:
   ```bash
   llc_emulator.py --link /tmp/llc_emu --prefix --scenario umbmark --duration 900 &
   olympus-pose -c /etc/olympus-pose/pose.conf
   ```
   In the last line of `stats.csv`: `acq_*`, `fus_*`, `est_*` delivered vs lost ≥ 99 % and
   `*_max_gap` ≤ 4; `lat_p95_ms` and `interval_p95_ms` < 100; `rss_kb` flat.
3. **llcmux without the Arduino** — `spike_pty_check.py` already showed `rover_bridge` works
   through a pty. With an Arduino (or a USB-serial adapter in loopback) on
   `/dev/arduino_mega_hw`, run `llcmux --stats-every 2` and open `/dev/arduino_mega` with
   `olympus_hlc --mode manual`: commands must still be answered, and `RAW filtered` must count
   up while `olympus_hlc` never sees a `RAW:` line.

## On the rover

`/etc/init.d/llcmux` and `/etc/init.d/olympus-pose` start at boot. Then follow the OE5 and
OE6 tests in DESIGN.md §16 (latency over 15 min with vision running, kill/restart checks,
IMU unplugged, UMBmark and straight-line runs with `pose_ctl.py MARK` at each start/stop).

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
| `tools/llcmux/` | LLC link multiplexer (DESIGN.md §6). Writes each line to `olympus_hlc` whole, with a leading `\n` (§6.1) |
| `tools/odom_calib.py` | Per-side encoder calibration from recorded straight runs and spins (DESIGN.md §9.5) |
| `tools/llc_emulator.py` | RAW: generator (pty or file) for tests without the rover: firmware v3.0 or v2.20 frames, IMU loss, slow LLC clock |
| `tools/capture_to_replay.py` | Turns a `raw_*.csv` of the LLC repo's `capture_rover_data.py` into a replay file |
| `tools/pose_listen.py`, `tools/pose_ctl.py` | UDP pose viewer and control-port client |
| `tools/olympus-pose_gui.py` | Web GUI: pose, GPS, test markers, controls, logs, figures and GPX (DESIGN.md §11.4) |
| `tools/gps_emulator.py` | GPS emulator (NMEA into a pty, or datagrams straight to the GUI) |
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

Expected: `OK: … checks passed, 0 failed`, including the RAW parser fuzz (10⁶ mutations
each for the v2.20 and v3.0 frames), 300 real v3.0 frames (LLC clock scale ≈ 1.86), the
generated C reproducing MATLAB on 2591 frames with zero difference, and the two
**model/app parity** tests (DESIGN.md §9.6): the C parser and the model's parser agree on
5000 mutated lines, and a recording run through the app's chain gives the same poses as
the model's `hlc_replay.m`, with both time bases.

**The model and the app estimate the same way.** The model's HLC (`hlc_step.m`) uses the
same parser rules, the same fusion inputs (`hlc_fusion_inputs.m` = `src/core/pipeline.c`),
event-driven estimation and the same parameters, including `est.dt_source` and
`est.gyro_yaw_axis`, which are now exported by the model. To reproduce a rover run in
MATLAB, replay its `llcmux` recording:

```matlab
R = hlc_replay('llc_record.txt');        % one row per pose, as pose.csv
% columns: line t_rx_ns tick t_in S x y theta omega bias d1 d2 d3 still imu_ok
```

**After changing the model or its parameters** (in MATLAB, simulation repo,
`Simulation Model v2.0`):

```matlab
codegen_hlc_core('<repo>\agents\src\core\gen')     % only if the algorithm changed
export_pose_conf                                    % then paste codegen/pose_model.conf
                                                    % into section 1 of config/pose.conf
```

and regenerate the test vectors with `tests/matlab/export_core_vectors.m`,
`export_replay_vectors.m` and (if the parser changed) `export_parser_vectors.m`. A
change to the parser or to the fusion inputs must be made on both sides
(`raw_frame_from_bytes.m` / `raw_parser.c`, `hlc_fusion_inputs.m` / `pipeline.c`); the
parity tests fail otherwise.

## Running

```bash
/etc/init.d/llcmux start          # usually started at boot
/etc/init.d/olympus-pose start
olympus-pose -c /etc/olympus-pose/pose.conf --check     # validate a configuration
olympus-pose_gui.py               # web page: http://<rover-ip>:8080 (see "Web GUI" below)
```

Console alternatives: `pose_listen.py` (live pose, one line per second; not at the same
time as the GUI, both listen on UDP 47001), `pose_ctl.py MARK start of run 3`,
`pose_ctl.py STATS`.

`olympus_hlc` is started by hand as before; it finds `/dev/arduino_mega` (now the
`llcmux` pty) and needs no change.

**Pose axes** (DESIGN.md §11.0): the origin is the rover's position when the estimator
starts. With the shipped `io.pose_frame = 1`, **y = where the front pointed, x = to its
right**. θ is the heading relative to that start direction, counter-clockwise positive, so
a drift to the left gives a negative x and a positive θ. `io.pose_frame = 0` publishes
the thesis/estimator frame instead (x forward, y left). It only changes what is published
(datagram, `pose.csv`, GUI, `RESET_POSE`), never the estimate.

**IMU mounting**: only the vertical axis matters. Set `gyro_yaw_axis` in the model's
`sim_params.m` to the IMU axis that points up (0 x, 1 y, 2 z) and re-export; it reaches
`pose.conf` as `est.gyro_yaw_axis` and the same axis is used for the rollover tilt. If it
points down, flip the sign of `ag.gyro_scale` in the model and re-export.

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

## Web GUI

`olympus-pose_gui.py` runs on the rover next to `olympus-pose` and serves one page to any
laptop on the rover's network, with no internet needed (DESIGN.md §11.4):

```bash
olympus-pose_gui.py                                      # live: http://<rover-ip>:8080
olympus-pose_gui.py --run /var/lib/olympus-pose/runs/<run>                 # replay a run
olympus-pose_gui.py --run /var/lib/olympus-pose/runs/<run> --gpx run.gpx   # GPX, no page
```

| Tab | What it shows / does |
|---|---|
| Pose | Estimator state, x, y, θ, distance, pose rate, status flags, the pose track; latency p95 (OE5), worst-link delivery (OE4) and LLC clock scale from the app's `STATS` |
| GPS | Fix state, satellites, HDOP, fix rate (OE2 ≥ 1 Hz), scatter at rest (CEP50/2DRMS), the track in metres, satellite/HDOP charts, latest fixes |
| Markers | **Mark** with a label (stop the rover first): stores the 10 s average GPS position and its 2DRMS, the pose at that moment, and writes `MARK <label>` into the run's `events.csv`. Table with the distances between markers. Saved in `/var/lib/olympus-pose/gui_marks.csv` |
| Controls & files | Reset pose, suspend/resume agents (through the CMAES AMS), run statistics, the logged runs with every file and a GPX per run to download |

Each plot has **SVG** and **PNG** buttons: the figure is saved in the light theme, ready
for the report and the slides. A GPX holds the track and the markers as named points, for
Google Earth or any GIS tool.

There is **no password**: anyone on the rover's network can use the controls; reset pose
and suspend ask for confirmation. The rover is never driven from the GUI. It is a separate,
normal-priority process: the pose threads (real-time priority) cannot be delayed by it, and
a refresh costs the same after 5 min or 5 h. To keep it off the rover entirely during the
OE5 latency tests, just don't start it.

For the thesis evidence: keep the rover still for 2 min (GPS scatter), then mark the start
and end of each test with a label, and save the figures.

**Indoors, without the GPS module:** `gps_emulator.py --link /tmp/gps_emu` produces NMEA (set
`io.gps_port = /tmp/gps_emu` and restart `olympus-pose`) to test the whole chain;
`gps_emulator.py --udp 127.0.0.1:47003 --speedup 10` feeds the GPS tab alone.

## LLC firmware versions

Both `RAW:` frames are accepted; the version of each frame is logged (`frame_version` in
`pose.csv`):

| Firmware | Frame | IMU failure |
|---|---|---|
| v2.20 | `RAW:tick:ax:ay:az:gx:gy:gz:encL:encR` | no frames at all → `STALE` |
| v3.0 (MPU-9250) | `…:encR:temp:imu_ok` | frames keep coming with `imu_ok = 0` → odometry only, `IMU_INVALID`, `imu_bad_frames` counts them |

**Time base.** Both firmwares advance `tick` by 20 per loop, but the v3.0 loop really takes
≈ 37 ms (LLC clock scale ≈ 1.87, column `llc_clock_scale` of `stats.csv`). With the default
`est.dt_source = 0` (LLC tick) the encoder turn rate is then overestimated and the heading
is wrong (DESIGN.md §10.4). **Use `dt_source = 1` (HLC arrival time from `llcmux`) until
the firmware loop is timer-driven:** set it in the model's `sim_params.m`, re-export, and
it reaches `pose.conf` as `est.dt_source` (the model's full UMBmark: 45 % error with 0,
1.34 %/0.50 % with 1; DESIGN.md §9.6). Replays of `llcmux` recordings and of converted
captures carry the arrival times, so both settings can be compared on the same file.

Bench captures from `capture_rover_data.py` can be replayed directly:

```bash
capture_to_replay.py raw_20261006_183344.csv -o /tmp/cap.txt
olympus-pose -c /etc/olympus-pose/pose.conf -r /tmp/cap.txt
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
   **Firmware v3.0 conditions** — slow LLC clock and a 10 s IMU loss:
   ```bash
   llc_emulator.py --out /tmp/slow.txt --prefix --rate 26.7 --tick-ms 20 --imu-off 60:70
   ```
   Replay it once with `est.dt_source = 0` and once with `1`: with `0` the square does not
   close (heading off by ~170°), with `1` it closes to a few cm. Between 60 and 70 s
   `pose.csv` must show `IMU_INVALID` (status bit) and `imu_bad_frames` rising by ≈ 267,
   with no `GAP`.
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

**Calibrate the encoder scale first** (DESIGN.md §9.5). The model's per-wheel constants
(including FR = 2.47 × FL) are not trusted for distance until this is done:

1. Add `--record-dir /var/lib/olympus-pose/cal` to `LLCMUX_ARGS` in `/etc/default/llcmux`
   and reboot (or restart `llcmux`, then `olympus_hlc`).
2. In one session: three straight runs of 2 m forward and one spin of one turn CCW in
   place, with ≥ 3 s at rest before each and between them. Measure each straight run
   with a tape between the same point of the chassis.
3. ```bash
   R=/var/lib/olympus-pose/cal/<run>/llc_record.txt
   odom_calib.py --list $R                  # numbers the motions: #1, #2, ...
   odom_calib.py --straight $R#1:2.00 --straight $R#2:2.00 --straight $R#3:2.00 --spin $R#4:1
   ```
4. Copy the printed `k_cal_R`, `k_cal_L` (and `chi`) into `sep_geo_params.m`, run
   `export_pose_conf`, paste section 1 into `pose.conf`, restart `olympus-pose`, and
   repeat one straight run as the check (error ≤ 2 %).

### Final test session: one run for every pending indicator

What is still open is OE2 (rate), OE4 (15-min delivery), OE5 (15-min latency) and OE6
(accuracy against markers and against the model). Done in this order, one continuous
session records all of them (test IDs from DESIGN.md §16):

**Before.** Image built from `main` with the latest `SRCREV`. IMU fixed in its final place,
and `gyro_yaw_axis` / the sign of `gyro_scale` matching it. `dt_source = 1` while the LLC
loop is software-timed. Both are set in `sim_params.m`, then re-exported. `llcmux` recording
on (step 1 above). Floor markers: a 5 m line and a 2 m square, with a tape point on the
chassis. The GUI (`http://<rover>:8080`) open.

1. **Calibration** (V-CAL-1, V-CAL-2): steps 1–4 above, then restart `olympus-pose`.
2. **Start the timed session** with `olympus_hlc` and vision running, as on a real mission
   (latency under load). Keep `olympus-pose` running for **≥ 15 min** from here on. Rest
   ≥ 3 s before every motion, and press **Mark** in the GUI at every start and stop.
3. **5 m straight line** ×3 (V-OE6-1): measure the end point against the markers;
   ≤ 15 cm.
4. **360° spin in place** ×3 (V-OE6-2): final heading error ≤ 10.8°.
5. **UMBmark 2 m**, clockwise and counter-clockwise; ideally 5 each, as in Borenstein's
   method (V-OE6-3, general objective). Final position ≤ 24 cm; pose ≥ 10 Hz.
6. **Stop** after ≥ 15 min. Keep the run folder and the `llcmux` recording.

**Reading the results:**

| Indicator | Where |
|---|---|
| OE2, LLC rate ≥ 50 Hz (V-OE2-1) | `llcmux` stats line (`RAW interval … (… Hz)`) and `llc_clock_scale` in `stats.csv`. With the current firmware this reads ≈ 26.7 Hz and fails; it needs the timer-driven LLC loop |
| OE4, delivery ≥ 99 %, ≤ 4 consecutive (V-OE4-2) | last line of `stats.csv`: `*_delivered`, `*_lost`, `*_max_gap` per link |
| OE5, latency < 100 ms for 15 min (V-OE5-1…3) | `stats.csv`: `lat_p95_ms`, `lat_max_ms`, `interval_p95_ms`, `rss_kb` over the whole run; `events.csv` without restarts |
| OE6 against markers (V-OE6-1…3) | `pose.csv` at each Mark (the GUI's Markers tab and `gui_marks.csv`) against the tape measurements |
| OE6 against the model (V-OE6-4) | MATLAB: `R = hlc_replay('<run>/llc_record.txt')` gives the same poses as `pose.csv`; then run the model's simulated scenario for the same path and compare |

Fault tests (V-OE5-5 kill/restart, V-OE5-6 IMU unplugged) go **after** the timed session,
because they would break its 15 minutes. The GPS test (V-OE2-2) is outdoors, on its own.

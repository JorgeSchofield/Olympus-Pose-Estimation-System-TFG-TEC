# Design of the HLC multi-agent pose-estimation application

| Field | Value |
|---|---|
| Document | ARQ-PE-003 — Agent architecture and LLC→HLC data path (draft) |
| Version | v0.6 — 2026-10-06 (week 10). v0.6: CMAES library items L1–L3, L5, L6, Y6, Y9 done; `rps_stress` OE3 test demo. v0.5: blocking layer items Y1–Y3, Y5 fixed and committed. v0.4: review of `meta-olympus-pose` (§15.4), init system unconfirmed so both init flavours are shipped, app builds CMAES from the same commit. v0.3: gateway-agent alternative rejected, systemd units, pose layer carries the udev change, Coder license confirmed. v0.2 (2026-10-01): pty naming decided (udev rename), GPS on the GPIO UART, simulation model v2.0 reviewed, C core generated with MATLAB Coder from unchanged core files, RAW filtering in `llcmux` |
| Covers | Deliverable "Documento de arquitectura multiagente y protocolo serial LLC-HLC" (OE2, OE4; milestone H2) and the design basis for the OE4/OE5/OE6 validation |
| Sources | Thesis report *Olympus_pose_TFG.pdf* (ch. 1, 2.1.4–2.1.8, 3.1–3.4, 4); `requirements/DRT-SEP-001.md` v0.2; `requirements/ICD-PE-002.md` v1.0; LLC firmware v2.20 (`Alonso11/rover-low-level-controller`, `src/main.rs`); HLC image (`Alonso11/olympus-hlc-rpi5`, `olympus_hlc` v3.x, `rover_bridge`); CMAES pthreads port (`CMAES/libCMAES_pthreads`); simulation model v2.0 (`JorgeSchofield/olympus-pose-estimation-simulation`, folder `Simulation Model v2.0`) |
| Status | Design only — no code yet. Section 17 lists the changes this design implies for the thesis report |

> **Requirement numbering.** This document uses the numbering of thesis §3.4.1
> (PE-RF-006 … PE-RF-013, PE-RNF-001 … PE-RNF-008). The copy of DRT-SEP-001 in
> `requirements/` (v0.2) uses an older numbering; see §17.13.

---

## 1. Purpose and scope

This document specifies the software that runs on the High-Level Controller (Raspberry
Pi 5, Yocto Linux) to estimate the planar pose of Olympus from the LLC sensor frames:

- the **`llcmux` daemon**, which shares the single LLC serial link between the existing
  `olympus_hlc` software and the pose application;
- the **CMAES application `olympus-pose`**, made of five agents (acquisition, data fusion,
  estimation, communication, GPS) plus a supervisor thread;
- the **inter-agent messages**, the **output interfaces** (UDP pose datagram, logs,
  control port), timing, configuration and fault handling;
- the **changes required in the CMAES pthreads port** before the application is built;
- the **validation plan** traced to objectives OE2–OE6 and to the requirements.

Out of scope: the LLC firmware (only its observable behaviour is used here), the IMU
electrical interface (OE1), and the estimator equations themselves, which are taken
unchanged from thesis §3.2 except for the input formulation in §9.

---

## 2. What the application must demonstrate (objectives)

| Objective | Indicator (thesis §1.3.3) | What this design provides for it |
|---|---|---|
| **Goal / general objective** | Pose at ≥ 10 Hz; final position error ≤ 3 % (24 cm on the 2 m UMBmark) | Event-driven pipeline publishing every accepted LLC frame (≈ 33–50 Hz); full-rate logs for offline error analysis |
| **OE2** | LLC sends six encoders + IMU at ≥ 50 Hz **measured with the HLC clock**; GPS valid fixes at ≥ 1 Hz | `llcmux` timestamps every frame with `CLOCK_MONOTONIC` and can record the raw stream; the GPS agent logs fix rate and validity; the LLC clock-scale regression (§10.4) checks the 65 % prediction of §3.3.6 |
| **OE3** | Library compiles on the RPi 5 Yocto image; 45 000 messages at 50 Hz with register/suspend/resume, no failures, no memory growth | Library changes in §14 must be regression-tested against this same indicator |
| **OE4** | ≥ 4 functional agents exchanging messages over the CMAES bus; 15 min at 50 Hz with delivery ≥ 99 % and ≤ 4 consecutive losses | Five agents; per-link sequence numbers and loss counters in every message; a 50 Hz LLC emulator (§16.1) so OE4 can be tested even if the firmware still runs at ≈ 33 Hz |
| **OE5** | End-to-end latency < 100 ms sustained for ≥ 15 min without failures | Timestamps propagated through every message; latency computed per pose; RSS/health stats once per second |
| **OE6** | Straight line 5 m, 360° spin, 2 m UMBmark: error ≤ 3 % against markers; same bounds against the simulation model | Bit-exact replay mode (recorded raw stream → same binary) so rover runs can be re-run in the app and in the MATLAB model on identical inputs |

---

## 3. Context, constraints and findings

### 3.1 The sensor frame actually emitted by firmware v2.20

```
RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>\n
```

Findings from reading `src/main.rs` and `src/sensors/mpu6050.rs`:

| Item | Observation | Consequence for the HLC |
|---|---|---|
| Link | Sent on **USART0**, the same USB-CDC link (`/dev/arduino_mega`, 115 200 8N1) as TLM and the MSM commands | The port must be shared (§3.2) |
| Emission condition | Only emitted when `mpu.read_raw()` succeeds | **If the IMU fails, the encoder data stops too.** No odometry-only fallback through `RAW:` (§13) |
| `tick` | `elapsed_ms`, incremented by a fixed 20 per loop iteration (software count, not a timer) | Δtick is always a multiple of 20 ms; it does not measure real time (thesis §3.3.6). The HLC estimates the real scale (§10.4) |
| Encoders | `encL = FL+CL+RL`, `encR = FR+CR+RR`, int32 wrap-around, ×2 decoding | Unwrap in fusion; per-side constant λ_side (thesis eq. 3.1) |
| IMU | MPU-6050 driver (accepts the MPU-9250 WHO_AM_I), gyro ±250 °/s → 131 LSB/(°/s), accel ±2 g → 16 384 LSB/g, DLPF ≈ 44 Hz. Sent axes: ax ay az gx gy gz (temperature is read but **not** sent) | Scale factors are configuration parameters; gyro temperature compensation is impossible without a frame change (ICD issue A-02) |
| Timing inside the cycle | Encoders are read in the stall block, IMU read separately; both go out in the same frame | As modelled in §3.3.2 of the thesis |
| Length | ≤ 82 bytes incl. `\n`; no checksum | Syntax check in acquisition, physical plausibility checks in fusion (§7.2) |
| Link load | ≈ 82 B × 10 bit × 50 Hz ≈ 41 kbit/s ≈ 36 % of 115 200 baud, plus TLM | Acceptable; on the LLC this costs ≈ 7 ms of blocking TX per cycle (OE2 firmware topic) |

### 3.2 The serial port is owned exclusively by `olympus_hlc`

`rover_bridge` (Rust/PyO3, `serialport` 4.8.1) opens `/dev/arduino_mega` with `TIOCEXCL` +
`flock` and discards `RAW:` lines. Two readers on one tty would each receive an arbitrary
subset of the bytes. The pose application also cannot own the port by itself during driving
tests: the LLC watchdog enters FAULT after 2 s without a `PING`, and the PINGs and drive
commands come from `olympus_hlc`.

**Decision D1 (taken 2026-10-01): a separate serial multiplexer daemon, `llcmux`** (§6).

**Finding: `RAW:` frames on USART0 already interfere with `olympus_hlc`, mux or not.**
The `olympus_hlc` engine calls `rover_bridge.recv_tlm()` **once per control cycle**, and
`recv_tlm()` consumes exactly one line: it returns it if it starts with `TLM:`, otherwise
it discards it and returns `None`. A cycle lasts at least 50 ms in vision mode (plus
inference time, up to about 2 s). Once the IMU is connected, firmware v2.20 emits 33–50
`RAW:` lines per second, so `olympus_hlc` would consume roughly one line per cycle while
dozens arrive. The `TLM:` lines would be delayed or lost in the backlog, and with them the
energy, thermal and stall monitors that depend on TLM. This breaks the no-interference
requirement (PE-RNF-006) as soon as the IMU is enabled, whatever the pose application does.
**Decision D13:** `llcmux` removes the `RAW:` lines from the stream it gives to
`olympus_hlc` (§6, rule 1). `olympus_hlc` then sees exactly the stream it saw before the
IMU was enabled.

**pty naming (decided, "least code"):** the udev rule renames the real Arduino port to
`/dev/arduino_mega_hw`, and `llcmux` publishes the pty as `/dev/arduino_mega`.
`olympus_hlc`, `olympus_controller.py` and the test scripts that hard-code
`/dev/arduino_mega` (`test_bridge.py`, `test_rover.py`, …) all keep working with **zero
code changes**. The rename is two `SYMLINK+=` values in `99-arduino.rules`, supplied from
the pose layer through a `.bbappend` (§15.3), so no file of the rover's repository is
edited. During
the spike and bench work, before the image is rebuilt, `olympus_hlc --port /run/olympus/llc`
gives the same result without touching any file.

Two details of `rover_bridge` that the mux must respect:

- **Arduino reset on open.** Opening the real port toggles DTR, which resets the
  ATmega2560 (the bridge then sleeps 2 s). When the bridge opens a pty instead, no reset
  happens. `llcmux` reproduces this by pulsing DTR on the real port when the
  `olympus_hlc` side of the pty is opened (configurable, on by default) so the observable
  behaviour of `olympus_hlc` does not change.
- **Opening a pty.** `serialport` 4.8.1 `TTYPort::open` only needs `TIOCEXCL`, `flock`,
  termios and the baud-rate call, and ignores DTR failures. All of these work on a Linux
  pty slave, so it should open a pty. **This must be confirmed on the rover before
  implementing the mux** (spike S-1, §18; script `tools/spike_pty_check.py`).

### 3.3 Nothing consumes a pose yet

`olympus_hlc`'s `OdometryTracker` (used by SLAM and `WaypointTracker`) only reads the
1 Hz TLM line. There is no input socket. **Decision D7:** the communication agent
publishes a binary datagram over UDP to localhost (§11.1). A consumer inside `olympus_hlc`
is future work and is not needed to satisfy the pose-output requirement (PE-RF-012):
the pose must be readable by an external consumer.

### 3.4 There is no GPS software on the HLC

**Decision D6:** a fifth agent reads NMEA and timestamps fixes with the same HLC clock as
the pose (PE-RF-013, OE2).

### 3.5 HLC platform

| Item | Value | Consequence |
|---|---|---|
| Kernel | Mainline RPi kernel, **not PREEMPT_RT**; `CONFIG_CPU_FREQ_DEFAULT_GOV_POWERSAVE=y`, `arm_freq=1500` | Wake-up latency is the risk, not compute. Measure before tuning (§10.3) |
| Init | **Unconfirmed.** The configuration in the HLC repository gives **sysvinit**: `local.conf` sets no `INIT_MANAGER`, and poky scarthgap defaults to `POKY_INIT_MANAGER = "sysvinit"`. The `meta-olympus-pose` README agrees. The HLC decision log (2026-03-18) says the opposite ("Scarthgap usa systemd por defecto"), and some rover recipes ship only systemd units. Check on the board with `ps -p 1 -o comm=`. `olympus_hlc` is launched by hand (`python3 -m olympus_hlc --mode … [--port …]`) | `llcmux` and `olympus-pose` ship **both** a SysV init script and a systemd unit (`inherit update-rc.d systemd`), so they start whichever init the image uses. `llcmux` sets its own priority and affinity in code instead of relying on unit options |
| Privileges | `debug-tweaks`, root login | `SCHED_FIFO` (CAP_SYS_NICE) available when run as root |
| Toolchain | No compiler in the image | Build with a Yocto recipe or the Yocto SDK (§15.3) |
| CPU load | YOLOv8n-seg vision on the same SoC | The pose threads run at real-time priority, pinned (§10) |

### 3.6 CMAES pthreads port — properties that shape the design

- Every agent has **one single-slot mailbox**. Sending with timeout 0 to a full mailbox
  drops the **new** message. Messages carry a **pointer** to their content.
- AMS replies (CONFIRM/REFUSE) are sent **to the requester's data mailbox** with timeout 0.
  A sensor-path agent must therefore never issue AMS requests.
- Suspension is **cooperative**: it takes effect at the next `MAES_CheckSuspend()`, at the
  top of the behaviour loop.
- `send0()` (broadcast) always blocks forever, so it is **never used** in this application.
- The main thread may call platform functions directly (`caller_is_setup_thread`) but
  **cannot send CMAES messages**, because it is not registered in `env` and
  `isRegistered()` would dereference NULL.
- The defects listed in §14 must be fixed first.

---

## 4. Architecture overview

```
                         Raspberry Pi 5 (HLC)
 ┌──────────────────────────────────────────────────────────────────────────────┐
 │                                                                              │
 │  olympus_hlc (unchanged code)        olympus-pose (CMAES process)            │
 │  /dev/arduino_mega (pty)  ◄──┐       ┌────────────────────────────────────┐  │
 │                              │       │ AMS (CMAES)      Supervisor (main) │  │
 │                              │       │                                    │  │
 │   llcmux ── pty A ───────────┘       │ ┌───────────┐ raw  ┌───────────┐   │  │
 │   (daemon)                           │ │Acquisition├─────►│  Fusion   │   │  │
 │      │  ── pty B (RAW:/TLM:, ───────►│ └───────────┘ msg  └─────┬─────┘   │  │
 │      │     prefixed @t_rx_ns)        │                   odom   │cumul.   │  │
 │      │  ── record file (optional)    │ ┌───────────┐ pose ┌─────▼─────┐   │  │
 │      │                               │ │Communicat.│◄─────┤Estimation │   │  │
 │  /dev/arduino_mega_hw                │ └─▲───┬───┬─┘      └─────▲─────┘   │  │
 │      │ (exclusive owner)             │   │gps│   │ ctrl (RESET)─┘         │  │
 │      │                               │ ┌─┴───┴┐  │                        │  │
 │      │                               │ │ GPS  │  │                        │  │
 │      │                               │ └──▲───┘  │                        │  │
 │      │                               └────┼──────┼────────────────────────┘  │
 │      │                          NMEA UART │      │ UDP 127.0.0.1 (pose)      │
 └──────┼───────────────────────────────────┼──────┼───────────────────────────┘
        │ USB CDC 115 200                   │      ├─► logs /var/log/olympus-pose
   LLC (ATmega2560, fw v2.20)          GPS module  └─◄ UDP control port
```

| Component | Kind | Responsibility | Requirements |
|---|---|---|---|
| `llcmux` | Separate process (C99 + POSIX) | Owns the LLC port; forwards the stream to `olympus_hlc` without `RAW:` lines and its commands to the LLC unchanged; copies `RAW:`/`TLM:` lines with an arrival timestamp to the pose app; optional recording | PE-RNF-006, OE2 |
| Acquisition agent | CMAES agent | Frames lines, accepts `RAW:`, parses fields, delivers **unconverted** samples | PE-RF-008 |
| Data fusion agent | CMAES agent | Unwraps accumulators, plausibility checks, converts to **cumulative** S, Θ_enc, Ω; rest and rollover detection | PE-RF-009, PE-RF-008 |
| Estimation agent | CMAES agent | Differences cumulatives; 5-state EKF (§3.2 of the thesis); slip detection | PE-RF-010, PE-RF-011 |
| Communication agent | CMAES agent | UDP pose output; logs; control port; AMS requests on operator command | PE-RF-012, PE-RF-013 |
| GPS agent | CMAES agent | Reads NMEA, validates, timestamps, forwards fixes | PE-RF-013, OE2 |
| Supervisor | Main thread (not an agent) | Builds the platform, signals, shutdown, health stats (RSS, CPU) | PE-RNF-005 |

---

## 5. Design decisions

| ID | Decision | Alternatives considered | Rationale |
|---|---|---|---|
| D1 | Share the LLC link with a **mux daemon** (`llcmux`) | Dedicated Channel 2 UART (ICD-PE-002 v1.0); tee hook in `rover_bridge`; pose app owns the port | No firmware or `olympus_hlc` code change; `olympus_hlc` sees the same stream as before the IMU was enabled (D13); if the mux dies, the LLC watchdog fails safe |
| D2 | Fusion→estimation messages carry **cumulative** S, Θ_enc, Ω and sample counters; the **estimator differences them** | Increments Δs/Δθ (thesis §3.4.3); increments with carry-over | Generalises the §3.3.6 rule ("convert to increments in the last step") to the second mailbox: a dropped message costs time resolution only, never distance; ω_enc and δ stay consistent across gaps; a corrupted frame that slips through is undone by the next frame instead of becoming a permanent offset |
| D3 | **Event-driven** chain: each agent blocks on its input | Estimation on a 20 ms timer (as in model 2.0) | Removes up to 20 ms of phase delay; avoids the "equal rates" mailbox problem of §3.3.6 once the LLC clock is fixed |
| D4 | All data sends use **timeout 0** (drop-new), per-producer rotation of **≥ 3 static buffers**, index advanced **only after a successful send** | Blocking sends; `MAES_QueueOverwrite` | Thesis §3.3.6 recommendation; with D2 drops are harmless; overwrite postponed (§14, item L7) |
| D5 | **Fusion owns every geometry- or sensor-dependent conversion** (λ, b_eff, signs, gyro scale and axis); estimation sees only physical units | Conversions in estimation | Mirrors the split `sep_odometry` / `sep_ekf_step` (thesis §3.2.10) |
| D6 | **GPS as a fifth agent** | Separate process; out of scope | One clock and one log for pose and reference (PE-RF-013) |
| D7 | Pose output: **fixed-size little-endian datagram, UDP to 127.0.0.1** | Unix socket; file only | Non-blocking, no consumer required, reachable by `olympus_hlc` or the ground station later |
| D8 | Arrival time stamped **by `llcmux`** at the serial read and carried in every message | Stamping in acquisition | Makes the measured HLC-side latency include the mux and the pty hop |
| D9 | Main thread = supervisor; **operator commands enter through the communication agent** | Supervisor sends messages | The main thread cannot send CMAES messages (§3.6); AMS requests from comm demonstrate the AMS as the thesis describes |
| D10 | `SCHED_FIFO` priorities **below the kernel IRQ threads (FIFO 50)**, all pipeline threads pinned to one core | Priorities 70–99 | Avoids starving the threaded USB/xHCI interrupt handler that delivers the frames |
| D11 | All parameters in one text file, parsed at start-up and written to the log header | Compile-time constants | Calibration changes (χ, λ, δ_th) without rebuilding; traceable runs |
| D12 | Application code split into **`core/` (pure C99, no OS)** and **`port/` (Linux)** | Single layer | PE-RNF-007: only `port/` and CMAES depend on Linux; `core/` is unit-testable on any host and comparable with MATLAB |
| D13 | `llcmux` **removes `RAW:` lines** from the stream given to `olympus_hlc`; publishes the pty as `/dev/arduino_mega` (udev renames the real port) | Byte-exact copy including `RAW:`; `--port` argument | `recv_tlm()` reads one line per cycle, so `RAW:` lines would starve `TLM:` (§3.2); udev rename needs zero code changes in `olympus_hlc` and in the legacy scripts |
| D14 | The C estimator is **generated with MATLAB Coder** from the four unchanged core files plus two new codegen-safe wrappers (`sep_fusion_step`, `sep_estimation_step`). **Parameters are entry-point arguments**, filled at run time from `pose.conf`, which is exported from `sep_geo_params`/`sep_ekf_params` | Hand-written C mirroring the `.m` files | One implementation of the algorithm (the model), already written codegen-safe for this purpose; the parameter source stays single (§12) |

---

## 6. `llcmux` — LLC link multiplexer

**Process:** standalone, C99 + POSIX, about 400 lines, started at boot before `olympus_hlc`.
`SCHED_FIFO` 45, pinned to the pose core.

| Endpoint | Path | Direction | Content |
|---|---|---|---|
| Real port | `/dev/arduino_mega_hw` (udev symlink, renamed) | ↔ | Raw termios, 115 200 8N1 |
| pty A | `/dev/arduino_mega` (symlink created by `llcmux`; also `/run/olympus/llc`) | ↔ | The LLC stream **minus `RAW:` lines** (D13); commands from `olympus_hlc` forwarded to the LLC unchanged |
| pty B | `/run/olympus/llc_raw` | → | Complete lines whose prefix is in the tee set (default `RAW:`, `TLM:`), each prefixed `@<t_rx_ns> ` |
| Record file | `--record <file>` | → | Every complete line as `<t_rx_ns>\t<line>` (written by a low-priority writer thread from a ring buffer) |

Because `/dev/arduino_mega` only exists while `llcmux` runs, `llcmux` is started at boot
by its init script or systemd unit and becomes required infrastructure. If an older image without the
udev change is used, `llcmux --hw /dev/arduino_mega --pty-link /run/olympus/llc` and
`olympus_hlc --port /run/olympus/llc` give the same behaviour.

**Rules**

1. `poll()` on the real port and both pty masters. Serial bytes are forwarded to pty A as
   soon as they are received, except at the start of a line: the first up to 4 bytes are
   held until the prefix is known. If the prefix is `RAW:`, the whole line is withheld from
   pty A (filter list configurable). Otherwise the held bytes and the rest of the line flow
   through immediately. The added delay for `olympus_hlc` is at most 4 byte times
   (≈ 0.35 ms).
2. The `t_rx_ns` of a line is `CLOCK_MONOTONIC` taken when its `\n` is read.
3. All writes to pty masters are **non-blocking**. On `EAGAIN` (reader absent or slow) the
   data is dropped and counted. The serial side is never blocked by a consumer.
4. When pty A's slave goes from closed to open (master stops reporting `POLLHUP`), pulse
   DTR on the real port (configurable) to reproduce the original reset-on-open.
5. If the real port disappears (USB re-enumeration), close the ptys' data path, retry
   opening every 500 ms, and log it.
6. Statistics once per second on stderr/syslog: bytes in/out, lines teed, drops,
   inter-arrival p50/p99 of `RAW:` lines (OE2 indicator on the HLC clock).

**Failure behaviour:** if `llcmux` dies, `olympus_hlc` gets EIO and stops sending PINGs.
The LLC watchdog enters FAULT within 2 s and cuts the motors (fail-safe). If
`olympus-pose` dies, the mux keeps serving `olympus_hlc` unaffected.

---

## 7. Agent specifications

Common pattern: every agent is a `CyclicBehaviour`. `setup()` initialises its `Agent_Msg`
(`msg.Agent_Msg()` sets the caller), pins the thread and loads its parameters.
`action()` processes **one** input and returns, so suspension is checked every iteration.
Data are sent with `msg.send(&msg, aid, 0)`, never with `send0()`. Each consumer copies the
payload into a local struct right after `receive()`.

### 7.1 Acquisition agent

| | |
|---|---|
| Trigger | `poll()` on pty B (or a real serial port, or a replay file) with a 100 ms timeout |
| Input | Byte stream; lines `[@<t_rx_ns> ]RAW:…` (and `TLM:…`, counted, otherwise ignored) |
| Output | `pe_raw_msg_t` → fusion, timeout 0 |
| State | Line buffer (128 B), per-link sequence counter, buffer rotation index, counters |
| Priority / core | FIFO 44, core 3 |

Steps:

1. Read the available bytes and assemble lines. Lines longer than 127 bytes are discarded
   up to the next `\n` (counted `frames_bad`).
2. Parse the optional `@<t_rx_ns>` prefix. If absent (direct serial or bench use), stamp
   `CLOCK_MONOTONIC` locally and set `RX_TS_LOCAL`.
3. Accept only `RAW:`. Require exactly 9 numeric fields after the prefix, base-10, no
   trailing characters, `tick` within uint32, IMU fields within int16, encoders within
   int32. Any violation → discard and count. No physical plausibility checks here: those
   belong in fusion.
4. Fill the next free buffer, `hdr.seq = ++seq`, and send with timeout 0. On failure,
   count `drops_out` and **keep the same buffer** for the next frame.
5. A binary-frame parser (ICD-PE-002, `0xA5 0x5A` + CRC-16) is a compile-time option, so
   a future firmware frame needs no agent change.

Failure hooks: `failure_detection()` returns true when no valid frame has arrived for
> 200 ms. `failure_recovery()` reopens the input after 1 s of silence. Data loss is
signalled downstream by the time gap (flag `STALE` raised by estimation).

### 7.2 Data fusion agent

| | |
|---|---|
| Trigger | `receive(MAES_MAX_DELAY)` on its mailbox |
| Input | `pe_raw_msg_t` |
| Output | `pe_odom_msg_t` → estimation, timeout 0 |
| State | Last accepted raw accumulators (int32) and unwrapped totals C̃_L, C̃_R (int64); continuous LLC time t_cum (uint64 ms); Ω sum; sample counters; rest counter; rejection counter |
| Priority / core | FIFO 43, core 3 |

Steps for each accepted message:

1. **Link accounting:** gap = `hdr.seq − last_seq − 1` → update received, lost and
   max-consecutive-lost counters for link acquisition→fusion.
2. **Time plausibility:** `d_tick = tick − tick_prev` (uint32 arithmetic).
   - `0 < d_tick ≤ dtick_max` (default 2000 ms) → normal.
   - Tick went backwards **and** both |encL|, |encR| < `reset_window_counts` → **LLC reset**:
     re-reference the accumulators, add the HLC arrival difference to t_cum, set
     `LLC_RESET`. Cumulative S, Θ_enc and Ω stay **continuous** (nothing is integrated
     across the reset).
   - Otherwise reject (possible corrupted byte). After `K_rereference` (default 3)
     consecutive rejections that are mutually consistent (Δtick ≈ 20 ms between them),
     adopt the new timeline as above.
3. **Count plausibility:** `dC_s = (int32)(enc_s − enc_s_prev)` (wrap-safe, via uint32
   subtraction). Reject the frame if `|dC_s| > rate_max_counts_s × d_tick/1000 + margin`.
   The limit is a parameter because counts per revolution are still an open anomaly
   (thesis §3.3.6).
4. **Convert and accumulate:** call the unchanged model function
   `[ds, dth, moving] = sep_odometry(dC_R, dC_L, geo)` on the per-frame increments, then
   `S += ds`, `Θ_enc += dth`. The signed per-side constants `m_per_tick_R` (negative: the
   right motors are mirror-mounted) and `m_per_tick_L` carry the sign convention exactly
   as in `sep_geo_params`. Because no mailbox sits between the increment and the sum, this
   is mathematically the same as applying the linear conversion to the totals:
   `S = (λ_R·C̃_R + λ_L·C̃_L)/2`, `Θ_enc = (λ_R·C̃_R − λ_L·C̃_L)/B_eff`.
   `t_cum += d_tick`; `n_samples += 1`.
5. **Gyro:** `ω_m = gyr[yaw_axis] × gyro_scale` (rad/s, sign included in `gyro_scale`,
   as `ag.gyro_scale` in the model). `Ω += ω_m`; `n_gyro += 1` (only when `IMU_VALID`).
   Also forward the latest ω_m for logging.
6. **Rest (µ):** from `moving` (`|dC_R| + |dC_L| > stop_ticks`, as in the model): if not
   moving then `still_frames++`, else `still_frames = 0`. Flag `STILL` when
   `still_frames ≥ N_still`. The default `N_still = 1` reproduces the model exactly; a
   larger value adds hysteresis if Hall-sensor bounce produces isolated counts at rest
   (open anomaly R-2).
7. **Rollover:** `tilt = acos(az/|a|)` from the accelerometer → flag `ROLLOVER` above
   `tilt_rollover` (default 30°). The accelerometer is **not** used in the filter
   (thesis §3.2.3).
8. Send `pe_odom_msg_t` with timeout 0. On failure, count and keep the buffer. **No
   carry-over logic is needed**: the next message holds the totals.

### 7.3 Estimation agent

| | |
|---|---|
| Trigger | `receive(100 ms)` on its mailbox |
| Inputs | `pe_odom_msg_t` (fusion); `pe_ctrl_msg_t` (communication: `RESET_POSE`, `SET_STATE`) |
| Output | `pe_pose_msg_t` → communication, timeout 0 |
| State | x̂ = [p_x p_y θ ω b_ω]ᵀ, P (5×5), last processed (t_cum, S, Θ_enc, Ω, n_gyro), mode |
| Priority / core | FIFO 42, core 3 |

Mode machine:

```
 WAIT_DATA ──first odom msg (reference only)──► INITIALIZING ──still ≥ T_init (3 s)──► RUNNING
     ▲                                                │                                  │
     └─────────── RESET_POSE (ctrl) ──────────────────┴──────────────────────────────────┤
                                   no input 100 ms → STALE flag (stays in mode, keeps x̂, P)
```

The first message after start, reset or resume **only sets the reference**. In
INITIALIZING the filter runs normally, but the pose is flagged as not yet valid, because the
gyro bias has not been observed at rest yet (thesis §3.2.8).

Per message, through the generated `sep_estimation_step` (§9.4), which calls the
unchanged `sep_ekf_step`:

1. **Link accounting** for fusion→estimation (same as §7.2, step 1).
2. **Differences:** Δt, Δs, Δθ_enc, Δn_g and the gyro mean z (§9.2).
   `moving = !(STILL && Δs == 0 && Δθ_enc == 0)`.
3. **Normal step** (Δt ≤ `dt_max` = 500 ms): one call
   `sep_ekf_step(x, P, Δs, Δθ_enc, z, Δt, moving, prm)`. Inside it: ω_enc = Δθ_enc/Δt,
   slip metric δ and Q gain γ (eqs. 3.10–3.11), prediction (eqs. 3.3–3.6), gyro update in
   Joseph form with H = [0 0 0 1 µ] (eq. 3.7), ZARU when at rest (eq. 3.9). θ is wrapped
   to [−π, π) and P is symmetrised, exactly as in the model.
4. **Long interval** (Δt > `dt_max`, after a link interruption or an AMS resume without
   re-reference): split it into m = ⌈Δt/dt_max⌉ sub-steps, each with Δs/m, Δθ_enc/m,
   Δt/m and the same z, and set `GAP`. `sep_ekf_step` never receives a Δt it would clamp,
   so ω_enc stays correct and **no distance is discarded**. Intervals longer than
   `t_gap_max` (default 10 s) only re-reference and set `GAP`.
5. If Δn_g = 0 (no valid gyro sample in the step), the wrapper passes a copy of `prm`
   with `r_gyro` set to a very large value, which disables the gyro correction without
   touching `sep_ekf_step`, and sets `IMU_INVALID`: odometry only, P grows.
6. A test build checks positive definiteness of P every step (PE-RF-010).
7. Fill `pe_pose_msg_t` (state, P, δ, γ, flags, input timestamps, upstream link
   statistics) and send with timeout 0.

`failure_detection()` → no input for 100 ms (sets `STALE`). On resume after an AMS
suspension, the next message only re-references (no integration of the suspended
interval), and `RESUMED` is reported once.

### 7.4 Communication agent

| | |
|---|---|
| Trigger | `receive(100 ms)`; also polls the control socket (non-blocking) on every wake-up, so at ≥ 10 Hz |
| Inputs | `pe_pose_msg_t`, `pe_gps_msg_t`; AMS replies (CONFIRM/REFUSE, logged); UDP control datagrams |
| Outputs | UDP pose datagram (§11.1); log records to the async writer (§11.2); `pe_ctrl_msg_t` → estimation (timeout 20 ms, not on the sensor path); AMS requests (`msg.suspend/resume`) |
| Priority / core | FIFO 41, core 3 |

Steps:

1. Pose: compute latency `t_pub − t_rx` (HLC side) and update the histogram. Send the
   datagram every `pose_decimation`-th message (default 1, i.e. every estimate). Push the
   full record to the estimate log.
2. GPS: push to the GPS log and keep the latest fix for the stats line.
3. Control: `RESET_POSE x y θ` → ctrl message to estimation; `SUSPEND <agent>` /
   `RESUME <agent>` → AMS request. The reply may be lost if the mailbox is full; the
   request is still executed and the next stats line reports the agent state via
   `get_state()`. `MARK <text>` writes an event marker to all logs (start/stop of a test
   run, marker positions).
4. Once per second: stats record (per-link delivery, max consecutive losses, latency
   p50/p95/max, pose rate, LLC clock-scale estimate, GPS rate, agent states).

The log writer is a low-priority `port/` thread fed by a static ring buffer, so an SD-card
stall never blocks the agent. If the ring overflows, records are dropped and counted.

### 7.5 GPS agent

| | |
|---|---|
| Trigger | `poll()` on the GPS UART with a 50 ms timeout |
| Input | NMEA 0183 (`$GxRMC`, `$GxGGA`), 9600 baud default (NEO-series GY-GPSV3) |
| Port | RPi 5 GPIO header UART0: GPS TX → GPIO15/RXD (pin 10), GPS RX ← GPIO14/TXD (pin 8, optional, only to reconfigure the module), GND (pin 6). Device `/dev/ttyAMA0` |
| Output | `pe_gps_msg_t` → communication, timeout 0; if the send fails, keep the fix as "pending" and retry on the next wake-up (≤ 50 ms later; GPS is not on the latency path) |
| Priority / core | FIFO 30, core 2 |

**GPIO UART checks before connecting** (one-time, on the rover):
- **Voltage:** the RPi 5 GPIO pins are 3.3 V and not 5 V tolerant. Measure the module's TX
  high level first (thesis §3.5.2). If it is 5 V, add a divider or level shifter on GPS TX.
- **UART enabled:** the image already sets `enable_uart=1` and `dtoverlay=disable-bt`.
  Confirm that `/dev/ttyAMA0` exists (`ls -l /dev/ttyAMA* /dev/serial*`). If it does not,
  add `dtparam=uart0=on` to `RPI_EXTRA_CONFIG` (one configuration line).
- **No console on it:** the RPi 5 serial console normally uses the dedicated debug
  connector (`ttyAMA10`). Check `cat /proc/cmdline` for `console=ttyAMA0` or
  `console=serial0`; if it points to the GPIO UART, the console must be moved, otherwise
  kernel messages and a login getty would share the line with the GPS.

Validates the checksum and fix status (RMC `A`, GGA quality > 0) and timestamps at the end
of the sentence with `CLOCK_MONOTONIC`. It combines RMC+GGA of the same epoch into one fix
message, counts the fix rate (OE2: ≥ 1 Hz) and never feeds the filter (PE-CON-001 /
thesis §1.6).

### 7.6 Supervisor (main thread)

1. Load the parameters; `mlockall()`; block SIGINT/SIGTERM/SIGUSR1 in the main thread
   **before** creating agents, so they inherit the mask.
2. Construct the platform and agents, `agent_init` ×5, `boot()`.
3. Loop on `sigtimedwait(1 s)`: every second, sample RSS (`/proc/self/statm`) and CPU time
   into the supervisor log stream. On SIGINT/SIGTERM: flush and close the logs, then
   `kill_agent` each agent (allowed from the setup thread), then exit with status 0.

---

## 8. Messages

### 8.1 Conventions

- Fixed-size C99 structs, defined once in `include/pe_msgs.h`, with `_Static_assert` on
  their sizes (C11 keyword is avoided in C99 builds through a typedef-array assertion
  macro).
- CMAES `MsgObj.type = INFORM` for data, `REQUEST` for control. `MsgObj.content` points to
  the struct in the producer's buffer pool.
- **Buffer pools:** `N_BUF = 4` per producer (≥ 3 required by thesis §3.1.3, plus one of
  margin). Advance the index only after a successful send. Pools are static (no `malloc`
  after initialisation).
- **Per-link sequence:** `hdr.seq` increases by 1 per **successful** send. A drop at send
  time therefore appears as a counted producer drop, and any loss elsewhere appears as a
  sequence gap at the consumer. Delivery rate = delivered / produced per link (OE4).

```c
typedef enum { PE_MSG_RAW = 1, PE_MSG_ODOM, PE_MSG_POSE, PE_MSG_GPS, PE_MSG_CTRL } pe_msg_kind_t;

typedef struct {                 /* 16 B, first member of every message */
    uint16_t kind;               /* pe_msg_kind_t */
    uint16_t version;            /* struct layout version */
    uint32_t seq;                /* per-link sequence, +1 per successful send */
    uint64_t t_rx_ns;            /* HLC CLOCK_MONOTONIC arrival of the newest LLC frame */
} pe_hdr_t;

typedef struct {                 /* link statistics, cumulative, 16 B */
    uint32_t delivered;          /* messages received on this link */
    uint32_t lost;               /* sum of sequence gaps */
    uint32_t max_consec_lost;    /* worst gap observed */
    uint32_t producer_drops;     /* sends that failed (reported by the producer) */
} pe_link_stats_t;
```

### 8.2 Acquisition → fusion: `pe_raw_msg_t` (unconverted, PE-RF-008)

```c
typedef struct {
    pe_hdr_t hdr;
    uint32_t t_llc_ms;           /* tick as sent by the LLC */
    int32_t  enc_l, enc_r;       /* raw side accumulators, unconverted */
    int16_t  acc[3], gyr[3];     /* raw IMU LSB */
    uint8_t  source;             /* PE_SRC_RAW_ASCII | PE_SRC_BIN_ICD | PE_SRC_REPLAY */
    uint8_t  flags;              /* IMU_VALID, RX_TS_LOCAL */
    uint16_t line_len;
    uint32_t frames_ok, frames_bad, producer_drops; /* cumulative acquisition counters */
} pe_raw_msg_t;
```

### 8.3 Fusion → estimation: `pe_odom_msg_t` (cumulative, decision D2)

```c
typedef struct {
    pe_hdr_t hdr;
    uint64_t t_cum_ms;           /* continuous LLC time (unwrapped, continuous across LLC resets) */
    uint32_t t_llc_ms;           /* raw tick of the newest frame (logging) */
    uint32_t n_samples;          /* cumulative accepted frames */
    double   s_m;                /* S: cumulative distance, m */
    double   theta_enc_rad;      /* Θ_enc: cumulative encoder heading, rad (not wrapped) */
    double   omega_sum;          /* Ω = Σ ω_m,k over valid gyro samples, rad/s */
    uint32_t n_gyro;             /* cumulative number of gyro samples in Ω */
    float    gyro_last_rad_s;    /* newest ω_m (logging) */
    float    tilt_rad;
    uint32_t still_frames;       /* consecutive still frames up to this one */
    uint16_t flags;              /* STILL, ROLLOVER, LLC_RESET, IMU_INVALID, FRAME_REJECTED */
    uint16_t reserved;
    pe_link_stats_t link_acq;    /* acquisition→fusion link, measured by fusion */
    uint32_t frames_ok, frames_bad; /* copied from acquisition */
} pe_odom_msg_t;
```

### 8.4 Estimation → communication: `pe_pose_msg_t`

```c
typedef struct {
    pe_hdr_t hdr;                /* t_rx_ns of the newest input frame */
    uint64_t t_est_ns;           /* end of the EKF step */
    uint64_t t_cum_ms;
    double   x[5];               /* px, py, theta, omega, b_omega */
    double   P[15];              /* upper triangle of P, row-major */
    double   s_m;                /* distance travelled (from S) */
    double   delta, gamma;       /* slip indicator and adaptation factor */
    double   z_gyro, omega_enc;  /* inputs of this step (logging/analysis) */
    float    dt_s;
    uint16_t mode;               /* WAIT_DATA, INITIALIZING, RUNNING */
    uint16_t status;             /* VALID, STILL, SLIP, STALE, GAP, IMU_INVALID, ROLLOVER, LLC_RESET, RESUMED */
    pe_link_stats_t link_acq, link_fus;
    uint32_t frames_ok, frames_bad;
} pe_pose_msg_t;
```

### 8.5 GPS → communication: `pe_gps_msg_t`; communication → estimation: `pe_ctrl_msg_t`

```c
typedef struct {
    pe_hdr_t hdr;                /* t_rx_ns = end of the last sentence of the epoch */
    double   lat_deg, lon_deg, alt_m;
    float    hdop, speed_mps, course_deg;
    uint32_t utc_ms_of_day;
    uint8_t  fix_quality, n_sats, valid, reserved;
} pe_gps_msg_t;

typedef struct {
    pe_hdr_t hdr;
    uint16_t cmd;                /* PE_CTRL_RESET_POSE, PE_CTRL_SET_BIAS */
    uint16_t reserved;
    double   arg[3];
} pe_ctrl_msg_t;
```

All structs are below 400 bytes; the pools take a few kilobytes in total.

---

## 9. Cumulative formulation (decision D2)

### 9.1 Fusion (sender side)

With C̃_L and C̃_R the unwrapped, signed side totals since the last re-reference
(int64), λ_L and λ_R the signed per-side constants (`m_per_tick_L`, `m_per_tick_R`,
eq. 3.1) and B_eff = χ·B_nom:

```
S       = Σ_k ds_k   = (λ_R·C̃_R + λ_L·C̃_L) / 2
Θ_enc   = Σ_k dth_k  = (λ_R·C̃_R − λ_L·C̃_L) / B_eff
Ω       = Σ_k ω_m,k        (sum of gyro rates, one term per valid frame)
n_g     = number of terms in Ω
t_cum   = Σ Δtick          (continuous LLC time)
```

ds_k and dth_k come from the unchanged `sep_odometry`. Because the conversion is linear
and there is no mailbox between the increment and the sum, S and Θ_enc carry exactly the
same information as the raw counts, in physical units.

### 9.2 Estimation (receiver side)

Against the last values it processed itself:

```
Δt      = (t_cum − t_cum,prev) / 1000          (sub-stepped if > dt_max, §7.3)
Δs      = S − S_prev
Δθ_enc  = Θ_enc − Θ_enc,prev
Δn_g    = n_g − n_g,prev
z       = (Ω − Ω_prev) / Δn_g                  mean gyro rate over the step (if Δn_g > 0)
```

These feed `sep_ekf_step(x, P, Δs, Δθ_enc, z, Δt, moving, prm)` **unchanged**. If no
message is lost, Δn_g = 1, and the call is identical to the one the model makes today.
When Δn_g > 1 the true noise of z is σ_g²/Δn_g, but `r_gyro` is kept as is. This is
slightly conservative during the rare gaps, and it keeps `sep_ekf_step` untouched. If
the equivalence tests show gaps are frequent, an optional `r_gyro` scaling can be added
later in the wrapper, without touching the core.

### 9.3 Behaviour when a message is lost

| Case | Increments (thesis §3.4.3) | Cumulative (this design) |
|---|---|---|
| Fusion→estimation message k dropped | Δs_k and Δθ_k lost forever: always-short distance (≈ 0.6 mm per drop at 2.9 cm/s, ≈ 8 cm at 1 % loss over 8 m) | Message k+1 gives the exact Δs, Δθ over k−1→k+1 |
| ω_enc across the gap | Wrong (20 ms of rotation over 40 ms, or the gap is never integrated) | Exact average over the same Δt as the gyro mean z |
| Slip indicator δ across the gap | False discrepancy | Consistent: both rates averaged over the same interval |
| Corrupted frame that passes the checks | Permanent offset | Spike undone by the next frame (totals are recomputed from absolute counts) |
| Re-start, resume or reset of the estimator | Needs extra logic | First message only re-references |

**Precision:** doubles give a resolution of about 10⁻¹³ m at 1 km; the int64 totals never
overflow in practice.

### 9.4 Changes to the MATLAB reference model (Simulation Model v2.0)

The four codegen-safe core files stay **unchanged**: `sep_geo_params`, `sep_ekf_params`,
`sep_odometry`, `sep_ekf_step`. Two codegen-safe files are added; they are the code that
MATLAB Coder turns into the fusion and estimation cores (D14):

| New file | Agent | Content | Signature (proposal) |
|---|---|---|---|
| `sep_fusion_step.m` | Data fusion | Wrap-safe differences (today's `wrap_i32`), plausibility with **last-accepted** reference, LLC-reset handling, `sep_odometry` per frame, accumulation of S, Θ_enc, Ω, n_g, t_cum, rest counter | `[fst, odom] = sep_fusion_step(fst, tick, encL, encR, gz_lsb, imu_ok, geo, fprm)` |
| `sep_estimation_step.m` | Estimation | Differences against the last processed values, sub-stepping of long intervals, Δn_g = 0 handling, calls `sep_ekf_step` | `[est, x_est, diag] = sep_estimation_step(est, odom, prm, eprm)` |

`fst`/`est` are fixed-size state structs, `odom` is the struct mirror of `pe_odom_msg_t`,
and `fprm`/`eprm` hold the agent thresholds that today live in `ag` (`dt_reject`,
`dcount_reject`, `gyro_scale`) plus the new ones (§12).

`hlc_step.m` changes:
- The estimation branch calls `sep_fusion_step` on mailbox 1 and deposits `odom` in a
  **second depth-1 mailbox** with the same three policies. A new branch consumes mailbox 2
  through `sep_estimation_step`. This closes the difference listed in thesis §3.3.7.
- `ag.est_mode = 'periodic' | 'event'` selects between today's 20 ms period and the
  event-driven activation of D3, so both can be compared.

New tests in `test_sep_model.m`:
- *(11)* With forced drops in mailbox 2, the final S is identical to the no-drop run, and
  the final position difference stays below the time-resolution bound.
- *(12)* A corrupted but plausible frame does not change the final S.
- *(13)* `sep_estimation_step` with Δt > `dt_max` sub-steps instead of clamping, so the
  distance over a 2 s gap is preserved.

Two findings from reading the model (also listed in §17.14):
- `hlc_step` sets the reference to a frame **even when it is rejected as implausible**.
  The next good frame is usually rejected too (it is differenced against the corrupted
  values), and the distance between the last accepted frame and the next accepted one,
  about two frame intervals, is lost permanently. `sep_fusion_step` keeps the last **accepted** reference instead.
- `sep_ekf_params.slip_thresh = 0.10 rad/s` (marked TBD), while thesis §3.2.9 argues for
  0.006 rad/s at 2.9 cm/s. Update the parameter or the text so they agree.

---

## 10. Timing, scheduling and latency

### 10.1 Priorities and CPU affinity

| Thread | Policy / priority | CPU | Notes |
|---|---|---|---|
| Kernel IRQ threads (USB/xHCI on RP1) | FIFO 50 (kernel default) | — | Must not be starved (D10) |
| CMAES AMS | FIFO **46** | 3 | Library currently uses the maximum (99); make it configurable (§14, L5) |
| `llcmux` | FIFO 45 | 3 | |
| Acquisition | FIFO 44 | 3 | |
| Fusion | FIFO 43 | 3 | |
| Estimation | FIFO 42 | 3 | |
| Communication | FIFO 41 | 3 | |
| GPS | FIFO 30 | 2 | |
| Log writer, supervisor | SCHED_OTHER, nice 10 | any | |

CMAES maps an agent priority p to `sched_get_priority_min(SCHED_FIFO) + p`, so FIFO 44 is
p = 43. The whole chain runs on one core, so a frame flows through without cross-core
wake-ups. Isolating core 3 (`isolcpus=3` in the RPi cmdline) and setting the
`performance` governor are **optional** image changes, applied only if V-OE5-1 shows a
tail-latency problem. They are image changes and must be agreed with the rover's owner.

### 10.2 Latency budget (update of thesis Table 3.8)

| Stage | Budget | Basis |
|---|---|---|
| Sampling in the LLC | ≤ 20 ms | Nominal LLC cycle |
| IMU read and frame build | ≤ 5 ms | PE-RNF-008 |
| Transmission of the frame | ≤ 7.1 ms | ≤ 82 B at 115 200 baud |
| USB CDC + `llcmux` (**t_rx stamped here**) | ≤ 2 ms | 1 ms USB full-speed frames + one wake-up |
| pty + acquisition | ≤ 5 ms | One FIFO wake-up + parse |
| Extra age when a frame is dropped at a full mailbox | ≤ 20 ms | The next frame carries the totals |
| Fusion agent | ≤ 3 ms | A few dozen flops |
| Estimation agent | ≤ 10 ms | Modelled cost ≈ 3 ms |
| Communication agent (publication = end point) | ≤ 5 ms | Modelled cost ≈ 2 ms |
| **Total** | **≤ 77 ms** | **Margin ≈ 23 ms** |

### 10.3 What is measured, and how

- **HLC side (exact):** `t_pub − t_rx` for every pose, from `CLOCK_MONOTONIC` in two
  processes on the same machine. Histogram, p50/p95/max per second and over the run.
- **LLC side (bounded):** sampling-to-arrival = (cycle phase ≤ 20 ms) + frame build +
  transmission time computed from the line length. Reported as a bound until the LLC
  clock is fixed.
- **Direct end-to-end (optional, strongest evidence for OE5):** in a test build the LLC
  toggles a pin at the IMU read and `olympus-pose` toggles an RPi GPIO at publication. An
  oscilloscope measures the delay over N samples. This needs a small firmware test feature
  and a GPIO helper in `port/`.
- **Endpoint definition:** OE5 says "until the estimate", PE-RNF-002 says "until
  publication". This design measures **until publication** (the stricter one) and also
  logs `t_est_ns`, so both are available.

### 10.4 LLC clock-scale estimation (OE2 support, thesis §4.4 step 2)

`llcmux`/communication fit the regression t_rx ≈ a + k·tick over a sliding window
(default 60 s, robust to outliers), excluding gaps and LLC resets. k ≈ 1.0 means a correct
LLC clock; the model predicts k ≈ 1/0.655 ≈ 1.53 for firmware v2.20. The value is logged
every second. Parameter `dt_source = LLC_TICK | LLC_TICK_SCALED | HLC_RX` selects which
Δt the estimator uses (default `LLC_TICK`, as decided in thesis §3.2.4), which lets the
clock hypothesis be tested on recorded data without changing the firmware.

---

## 11. External interfaces

### 11.1 Pose datagram (UDP, little-endian, 128 bytes, version 1)

| Offset | Type | Field |
|---|---|---|
| 0 | char[4] | magic `"OPE1"` |
| 4 | u32 | seq |
| 8 | u64 | t_pub_ns (HLC CLOCK_MONOTONIC) |
| 16 | u64 | t_rx_ns (arrival of the newest input frame) |
| 24 | u32 | t_llc_ms |
| 28 | u16 | mode |
| 30 | u16 | status bits (as §8.4) |
| 32 | f64 ×3 | x_m, y_m, theta_rad (wrapped to [−π, π), as in the model) |
| 56 | f64 ×2 | omega_rad_s, bias_rad_s |
| 72 | f64 ×4 | P_xx, P_yy, P_xy, P_θθ |
| 104 | f64 | s_m (distance travelled) |
| 112 | f64 | delta (slip indicator) |
| 120 | u32 | reserved |
| 124 | u32 | crc32 (of bytes 0–123; lets file captures be checked too) |

Destination `udp_pose_dest` (default `127.0.0.1:47001`), optional second destination (for
example the ground station). Serialisation is field by field into a byte buffer, so it
does not depend on compiler padding. A Python decoder is provided in `tools/`.

### 11.2 Logs (directory `log_dir`, default `/var/log/olympus-pose/<run-id>/`)

| File | Rate | Content |
|---|---|---|
| `header.txt` | once | Software version, git hash, full parameter list + hash, `dt_source`, start time |
| `pose.csv` | every estimate | seq, t_rx, t_est, t_pub, t_llc, x̂, P (upper triangle), δ, γ, z, ω_enc, Δt, mode, status, link stats |
| `gps.csv` | every fix | t_rx, UTC, lat, lon, alt, quality, sats, HDOP, speed, course |
| `stats.csv` | 1 Hz | per-link delivery and max consecutive loss, latency p50/p95/max, pose rate, clock scale k, RSS, CPU, agent states, writer drops |
| `events.csv` | on event | MARK, RESET, SUSPEND/RESUME, LLC_RESET, STALE begin/end, start/stop |

The raw input stream for replay is recorded by `llcmux --record` (complete, including
frames the app rejected). Both share `t_rx_ns`, so they join exactly.

### 11.3 Control port (UDP, text, localhost only, default `127.0.0.1:47002`)

`MARK <text>`, `RESET_POSE [x y θ]`, `SUSPEND <agent>`, `RESUME <agent>`, `STATS`
(replies with the latest stats line).

---

## 12. Configuration parameters (`/etc/olympus-pose/pose.conf`)

**Single source of truth:** the model files stay the master copy. A MATLAB script
`tools/export_params.m` calls `sep_geo_params`, `sep_ekf_params` and the agent parameters,
and writes `pose.conf` with the status tag of each value (`MED`, `DER`, `PROV`, `TBD`, as in
`rover_params`). The generated C code takes `geo`, `prm`, `fprm` and `eprm` as entry-point
arguments (D14), so the app fills them from `pose.conf` at start-up. Recalibration means
editing the model parameter file, re-exporting and restarting, with no rebuild. Key names
are the model field names.

| Group | Keys (model names) | Default | Status |
|---|---|---|---|
| Geometry `geo.*` | `m_per_tick_R` (negative, mirror-mounted side), `m_per_tick_L` | From `ticks_per_rev` and `R_wheel` (2026-09-14 campaign) | DER (counts/rev anomaly open) |
| | `B_nom` | 0.622 m | MED |
| | `chi`, `B_eff` | 1.00, χ·B_nom | TBD (360° spin) / DER |
| | `stop_ticks` | 0 | PROV (raise if Hall bounce is confirmed) |
| EKF `prm.*` | `k_rho`, `s2_ds_floor`, `s2_theta`, `s2_w_enc`, `s2_bg`, `r_gyro`, `r_zaru`, `P0` | `sep_ekf_params` values | PROV |
| | `slip_thresh`, `slip_gain`, `slip_cap` | 0.10 rad/s (model) vs 0.006 rad/s (thesis §3.2.9), 20, 100 | TBD; reconcile (§9.4) |
| | `dt_min`, `dt_max` | 1 ms, 500 ms | — |
| Fusion `fprm.*` | `gyro_scale` (sign included), `yaw_axis`, `acc_scale` | (π/180)/131, z, 9.80665/16384 | DER (firmware ±250 °/s, ±2 g); axis/sign TBD by hand rotation |
| | `dt_reject` (`dtick_max`), `dcount_reject` (`rate_max_counts_s`), `reset_window_counts`, `K_rereference`, `N_still`, `tilt_rollover` | 2000 ms, *TBD*, 1000, 3, 1, 30° | PROV |
| Estimation `eprm.*` | `T_init`, `t_gap_max`, `dt_source` | 3 s, 10 s, `LLC_TICK` | PROV |
| I/O | `raw_input`, `gps_port`, `gps_baud`, `udp_pose_dest`, `udp_ctrl_port`, `pose_decimation`, `log_dir` | `/run/olympus/llc_raw`, `/dev/ttyAMA0`, 9600, `127.0.0.1:47001`, 47002, 1, `/var/log/olympus-pose` | — |
| RT | `prio_*`, `cpu_pipeline`, `cpu_gps` | §10.1 | — |

Unknown keys or missing required keys are a start-up error. The parsed set is written to
`header.txt` with a hash.

---

## 13. Faults and degraded modes

| Fault | Detection | Behaviour | Flag / log |
|---|---|---|---|
| Corrupted byte in a frame | Syntax (acquisition), time/count plausibility (fusion) | Frame rejected; the next frame carries the totals | `frames_bad`, `FRAME_REJECTED` |
| Lost frame on the serial link | Δtick > 20 ms | Longer step, no distance lost | link stats from Δtick |
| Message dropped at a mailbox | Producer send fails / sequence gap | Next message carries the totals | `producer_drops`, `lost` |
| LLC reset (power, DTR, watchdog reboot) | Tick backwards + counts near 0 | Re-reference, no jump in the pose | `LLC_RESET` |
| IMU failure (fw v2.20) | No `RAW:` frames at all | Pose held, `STALE` after 100 ms; resumes on data | `STALE` |
| IMU failure (future frame with `imu_ok`) | `IMU_VALID` = 0 | Odometry only, P grows | `IMU_INVALID` |
| Long link interruption | Δt > `dt_max` (500 ms) | Integrate the exact Δs, Δθ in sub-steps of ≤ `dt_max`; only re-reference beyond `t_gap_max` | `GAP` |
| Wheel slip on one side | δ > δ_th | Q inflated by γ (thesis §3.2.9) | `SLIP` |
| Equal slip on all six wheels | Not detectable (thesis §3.2.9) | Documented limitation | — |
| Rollover | Tilt > threshold | Pose flagged | `ROLLOVER` |
| `llcmux` dies | — | LLC watchdog FAULT ≤ 2 s (fail-safe) | — |
| `olympus-pose` dies | — | `olympus_hlc` unaffected; the init script or systemd unit restarts it | — |
| Log storage slow or full | Writer ring overflow | Records dropped and counted; pipeline unaffected | `writer_drops` |

---

## 14. Required changes to the CMAES pthreads port (before the app)

| ID | Change | Why | Verification |
|---|---|---|---|
| L1 | Mailbox timed waits on **`CLOCK_MONOTONIC`** | `CLOCK_REALTIME` jumps when the time is set at boot or by NTP | **Done (671bf4a)**; on-target check V-LIB-2 pending |
| L2 | Absolute-deadline wait **`MAES_DelayUntil`** + `MAES_GetTickCount` | `agent_wait` drifts | **Done (671bf4a)**: no catch-up bursts after an overrun or a suspension; used by `rps_stress` |
| L3 | CPU affinity **`MAES_SetAffinity(aid, cpu)`** | Thesis §3.4.5 relies on core pinning | **Done (671bf4a)**; check with V-LIB-3 on target |
| L4 | ~~Fix `CMAES/CMakeLists.txt` path~~ — **done upstream** (current `main`); only the layer's `SRCREV` must follow (§15.4, Y2) | — | — |
| L5 | Configurable AMS priority (`MAES_SetAMSPriority`, default 46); agents always below the AMS | D10 | **Done (671bf4a)** |
| L6 | Header/implementation mismatches (27 `void*` → `void`), `get_AP_description` by-value return, double decrement in `kill_agent`, `failure_identification` wiring, `agent_init` taking `void*`, `MAX_RECEIVERS`, `ConstructorUSER_DEF_COND` prototype | Undefined behaviour; errors under GCC 14 | **Done (6358206, 671bf4a)**: strict C99, zero warnings under Clang 20; the recipe needs no `-Wno-error` flags any more |
| L7 | (Optional, later) `MAES_QueueOverwrite` | Not needed with D2+D4 | — |
| L8 | Keep: send/receive/AMS semantics unchanged | API compatibility with the FreeRTOS version | Existing demos |
| L9 | ~~`install()` rules~~ — **done upstream**. `olympus-pose` builds the library with `add_subdirectory` from the same commit (§15.3), so it does not depend on them | — | — |

After L1–L6, re-run the OE3 indicator with `cmaes_rps_stress_demo 45000 20 3` (as root on the
RPi 5): players at 50 Hz, AMS suspend/resume every round, checksummed payloads, RSS sampled
after warm-up. Pass = exit code 0 and `RESULT : PASS`.

---

## 15. Code organisation, build and deployment

### 15.1 Layout (in this `agents/` folder)

```
agents/
├── DESIGN.md                 this document
├── CMakeLists.txt            builds olympus-pose, llcmux, tests; links CMAES
├── include/
│   ├── pe_msgs.h             §8 message structs + static size asserts
│   └── pe_params.h           parameter struct
├── src/
│   ├── main.c                supervisor (§7.6)
│   ├── agent_acquisition.c   §7.1
│   ├── agent_fusion.c        §7.2
│   ├── agent_estimation.c    §7.3
│   ├── agent_comm.c          §7.4
│   ├── agent_gps.c           §7.5
│   ├── core/                 pure C99, no OS, unit-tested on host
│   │   ├── raw_parser.c      RAW: and binary ICD frame
│   │   ├── nmea_parser.c
│   │   ├── gen/              MATLAB Coder output, committed, never edited by hand:
│   │   │                     sep_fusion_step, sep_estimation_step (+ sep_odometry,
│   │   │                     sep_ekf_step they call); regenerated by tools/codegen.m
│   │   ├── params.c          key=value parser → geo/prm/fprm/eprm structs
│   │   ├── msg_pool.c        buffer rotation
│   │   └── stats.c           link statistics, latency histogram, clock-scale regression
│   └── port/                 Linux portability layer of the application
│       ├── port_serial.c     termios / pty / replay file
│       ├── port_time.c       CLOCK_MONOTONIC
│       ├── port_udp.c
│       ├── port_log.c        async ring-buffer file writer
│       └── port_rt.c         mlockall, affinity helpers, RSS
├── tools/
│   ├── llcmux/               §6
│   ├── llc_emulator.py       50 Hz RAW: generator into a pty, scenario files, fault injection
│   ├── pose_listen.py        UDP datagram decoder
│   ├── spike_pty_check.py    spike S-1 (§18)
│   ├── codegen.m             MATLAB Coder configuration (C99, no dynamic memory, structs as args)
│   ├── export_params.m       model parameters → pose.conf (§12)
│   └── replay/               scripts: record → replay into app and into MATLAB
└── tests/                    unit + integration tests (§16)
```

### 15.2 Language and coding rules (PE-RNF-007)

`core/` and the agents are C99 (`-std=c99 -Wall -Wextra -Werror`), with no dynamic memory
after start-up and no OS calls outside `port/` and CMAES. Host builds use
`-fsanitize=address,undefined` for the tests. Doubles are used throughout the estimator,
matching the MATLAB reference.

MATLAB Coder settings for `core/gen/` (in `tools/codegen.m`): target language C,
`TargetLangStandard = 'C99 (ISO)'`, `EnableDynamicMemoryAllocation = false`, entry points
`sep_fusion_step` and `sep_estimation_step` with fixed-size struct arguments, no
`persistent` state (all state lives in `fst`/`est`, owned by the agents), and
`GenerateReport = true` so the code-generation report is kept as evidence for PE-RNF-007.
Generated files are compiled with the same warning flags. Warnings in generated code are
reported, not edited.

### 15.3 Build and deployment

- **Development:** native build on an x86 Ubuntu host for `core/` tests and SIL runs
  (§16.1).
- **Target:** the layer `meta-olympus-pose` (in this repository at `CMAES/CMAES_Yocto_layer/`, reviewed in §15.4) already
  builds CMAES into the rover image. Add a recipe `olympus-pose` (classes `cmake`,
  `update-rc.d`, `systemd`) to it. It fetches **this same repository at one SRCREV** and
  builds `agents/`, whose `CMakeLists.txt` pulls the library in with
  `add_subdirectory(../CMAES/libCMAES_pthreads)`. The application and the library therefore
  always come from the same commit, and the app inherits the library's `_GNU_SOURCE`
  definition and `Threads` link without needing an exported CMake package. The existing
  `cmaes` recipe stays for the demos (OE3 indicator). In the layer's
  `olympus-image.bbappend`, `cmaes-demos` is replaced by `olympus-pose` once the
  application runs (keep the demos until milestone H3 is closed). The udev
  rename of §3.2 is also done from that layer, with a `custom-udev-rules.bbappend`
  (`FILESEXTRAPATHS:prepend := "${THISDIR}/files:"`) that supplies a replacement
  `99-arduino.rules`. That file is identical to the original except for the symlink name
  and an explicit systemd tag:

  ```
  SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="arduino_mega_hw", TAG+="systemd", MODE="0666"
  SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino_mega_hw", TAG+="systemd", MODE="0666"
  ```

  So **no file in `olympus-hlc-rpi5` changes at all**; removing the layer from
  `bblayers.conf` restores the original rover image. The recipe installs:
  - `/usr/bin/olympus-pose`, `/usr/bin/llcmux`, `/etc/olympus-pose/pose.conf`;
  - **for sysvinit** (what the repository configuration gives, §3.5): init scripts
    `llcmux` (start priority 90, before any manual `olympus_hlc` launch) and
    `olympus-pose` (91), each relaunching its daemon if it exits;
  - **for systemd** (if the board turns out to use it): `llcmux.service` (`Restart=always`)
    and `olympus-pose.service` (`Wants=`/`After=llcmux.service`, `Restart=on-failure`,
    `LimitRTPRIO=50`, `LimitMEMLOCK=infinity`).

  The classes install whichever matches `DISTRO_FEATURES`. In both cases `llcmux` and the
  agents set their own `SCHED_FIFO` priority and CPU affinity in code (§10.1), and `llcmux`
  waits for `/dev/arduino_mega_hw` itself (§6, rule 5), so nothing depends on init-specific
  features. This is additive to the image; `python3-rover-bridge` and `olympus_hlc` are
  not modified.
- **Development loop:** `SRC_URI` points at GitHub, so a normal `bitbake` only sees pushed
  commits. While iterating, use `devtool modify olympus-pose` (or `externalsrc`) against
  the local checkout, or the Yocto SDK (`bitbake olympus-image -c populate_sdk`) and `scp`,
  so each change does not need a push and a full image rebuild.

### 15.4 Review of `meta-olympus-pose` (2026-10-05)

The layer is well structured: separate collection name, priority 11 above `meta-olympus`,
pinned `SRCREV`, static library split into `-staticdev`/`-dev`, demos in their own package
with disabled units. Required changes before the application work starts:

| ID | Finding | Effect | Change |
|---|---|---|---|
| Y1 | The layer lived at `CMAES/CMAES Yocto layer/meta-olympus-pose`: **a path with spaces** | `BBLAYERS` is a whitespace-separated list, so the layer could not be added from where it was | **Done (2aff319):** renamed to `CMAES/CMAES_Yocto_layer/meta-olympus-pose`, kept inside `CMAES/` as requested; README install steps clone the repository and add the layer by path |
| Y2 | `SRCREV = ea4ce44…` built an old library. Worse, the current `main` (`a56a5e8`) **does not configure**: `install(TARGETS cmaes_pthreads)` ran before `add_library` (reproduced with an aarch64 cross build: "install TARGETS given target cmaes_pthreads which does not exist") | The layer could not build any recent library | **Done:** library fixed in `6358206` (install rules moved after the target); `SRCREV` pinned to it in `2aff319`. `6358206` is on `master`: merge the master→main pull request before running bitbake, because the recipe fetches `branch=main` |
| Y3 | Workarounds for problems already fixed upstream: the `sed` on `add_subdirectory`, the hand-written `do_install` | Dead code once Y2 is done | **Done (2aff319):** both removed; default `cmake` install plus `do_install:append` for the demo units. Verified that the upstream install rules produce exactly the four packaged files |
| Y4 | README caveat 1 and the recipe comments reference `CHANGES.md` (changes 1–5), which is not in the repository | Unverifiable references; changes 4–5 (GCC 14 errors) are not traceable | Add `CHANGES.md` or fold its content into `CMAES/docs/README.md`; mark 1–3 as done |
| Y5 | `ConstructorUSER_DEF_COND` was **not declared** in `CMAES.h` (thesis §3.1.5 says it was added) | Implicit function declaration in `Agent_Platform.c`: a warning on GCC 13, an error on GCC 14+ | **Done:** prototype added in `6358206`; `-Wno-error=implicit-function-declaration` and `=int-conversion` dropped from the recipe (neither is triggered any more). `=incompatible-pointer-types` stays for the 37 remaining `void`/`void*` constructor assignments (GCC 14 cleanup, not blocking on scarthgap) |
| Y6 | References to `docs/PORTING_NOTES.md` (now `docs/README.md`) | Broken references | **Done (671bf4a)** |
| Y7 | 84 build artefacts are committed under `CMAES/build/` (x86 binaries, `CMakeCache.txt` with an absolute path) | Clutter; a stale cache can confuse a local `cmake -B build` | Delete them and add `build/` to `.gitignore` |
| Y8 | README caveat 2 says the image is sysvinit; the HLC decision log says systemd | The units may never run | Check `ps -p 1 -o comm=` on the board; the `olympus-pose` recipe ships both (§15.3) |
| Y9 | Library built as C11; PE-RNF-007 asks for C99 | Inconsistent with the requirement | **Done (671bf4a)**: `CMAKE_C_STANDARD 99`, extensions off (`-std=c99`) |
| Y10 | The local working folder was not a git checkout and was older than GitHub | — | **Done:** working clone at `…/Olympus-Pose-Estimation-System-TFG-TEC` (branch `master`, PRs into `main`) |

Y1–Y3, Y5 and Y10 are done (2026-10-05). Y4, Y6–Y9 are hygiene and remain open.

---

## 16. Validation plan

### 16.1 Test infrastructure

| Level | Tool | Purpose |
|---|---|---|
| Unit (host) | `tests/` with test vectors, ASan/UBSan | Parsers, fusion math, EKF, buffer pool |
| MATLAB equivalence | `test_sep_frame` golden vector + scenario exports from model 2.0 (`run_sep_full_demo` MATLAB loop) | Generated C (`core/gen/`) vs the same `.m` functions in MATLAB, input by input. This catches code-generation, struct-filling and parameter-loading errors |
| SIL (host or RPi) | `llc_emulator.py` → pty → full app | Whole pipeline at exactly 50 Hz with scripted motion, frame loss, corruption, LLC reset; can stream the `llc_step` output exported from the Simulink model |
| Replay | `llcmux --record` file → `olympus-pose --replay` and → MATLAB | Bit-exact re-runs of rover tests (OE6 model comparison) |
| On rover | `llcmux` + `olympus_hlc` + `olympus-pose`, markers and tape measure, GPS outdoors | OE2, OE5, OE6 |

### 16.2 Tests

| ID | Objective / requirement | Method | Procedure | Pass criterion |
|---|---|---|---|---|
| V-LIB-1 | OE3 | T | After L1–L6: 50 Hz rock-paper-scissors, 45 000 messages, register/suspend/resume | No failures; RSS slope ≈ 0 after warm-up |
| V-LIB-2 | OE3 / L1 | T | Change system time during timed waits | Timeouts unaffected |
| V-LIB-3 | §10 / L3 | I | Check affinity and policy of every thread | Matches §10.1 |
| V-U-1 | PE-RF-008 | T | RAW parser: valid vectors, every malformed class, 10⁶ random mutations | No crash/UB; no syntactically invalid frame accepted |
| V-U-2 | PE-RF-013 | T | NMEA parser vectors (checksum, RMC/GGA, no fix) | All correct |
| V-U-3 | PE-RF-009 | A/T | Generated `sep_fusion_step` in C vs the same function in MATLAB, on the golden vector and model scenarios (parameters loaded from the exported `pose.conf`) | Relative difference ≤ 1e-9 (DRT: ≤ 1 % in v, ω) |
| V-U-4 | PE-RF-009 | T | Accumulators crossing ±2³¹; LLC reset; corrupted tick/count | S, Θ continuous; reset flagged; corrupted frame rejected and undone |
| V-U-5 | PE-RF-010 | A/T | Generated `sep_estimation_step` (with `sep_ekf_step`) in C vs MATLAB on model scenarios, including gaps and sub-stepping | State difference ≤ 1e-9; P symmetric and positive definite at every step |
| V-U-6 | PE-RF-010 | T | Bias changes only while `STILL` (model test replicated) | b̂ constant during motion |
| V-U-7 | PE-RF-011 | T | Model scenario with 30 % slip on one side (and on all six wheels) | `SLIP` and γ > 1 during one-side slip; six-wheel slip not detected (documented) |
| V-U-8 | PE-RF-007 | T | Buffer-pool stress: producer stamps a checksum, consumer adds random delays, 10⁶ messages | 0 corrupted payloads |
| V-I-1 | PE-RF-008, D2 | T | SIL with forced drop probability 1–10 % on each mailbox (test build) | Final S **bit-identical** to the no-drop run; final pose difference within time-resolution effects |
| V-I-2 | OE6 (simulation) | A | SIL with model-exported UMBmark (with 3 s pauses) | App pose vs model EKF within 1e-6 m; vs model ground truth within the model's own error |
| V-OE4-1 | OE4 / PE-RF-006 | D | Start the app; trace registrations and messages | 5 agents registered; every link active |
| V-OE4-2 | OE4 / PE-RNF-003 | T | 15 min at 50 Hz (emulator) = 45 000 frames, with vision running (or `stress-ng` on cores 0–2) | Each link ≥ 99 % delivery, ≤ 4 consecutive losses |
| V-OE2-1 | OE2 | T | 10 min on the rover, `llcmux` statistics/record | `RAW:` inter-arrival p99 ≤ 20 ms on the HLC clock; clock scale k reported (§10.4) |
| V-OE2-2 | OE2 / PE-RF-013 | T | ≥ 300 s outdoors | Valid fixes at ≥ 1 Hz |
| V-OE5-1 | OE5 / PE-RNF-002 | T | 15 min on the rover with `olympus_hlc` + vision running | p95 (and max reported) of the end-to-end latency < 100 ms; optional oscilloscope method |
| V-OE5-2 | PE-RNF-001 | T | Same run | Pose interval p95 ≤ 100 ms |
| V-OE5-3 | OE5 / PE-RNF-005 | T | Same run, extended to 60 min for margin | No agent restart or failure; RSS stable; no growing drop counters |
| V-OE5-4 | PE-RNF-006 | I/T | `olympus_hlc` manual and vision modes via `llcmux` vs direct (IMU disconnected for the direct baseline); compare ACK latency distributions, TLM reception rate and recorded byte streams; code diff of `olympus_hlc` | Streams identical except the removed `RAW:` lines; TLM received at ≈ 1 Hz with the IMU enabled; ACK latency unchanged within noise; no code change |
| V-OE5-5 | Safety | T | Kill `llcmux`; kill/restart `olympus-pose` | LLC FAULT ≤ 2 s; `olympus_hlc` unaffected; app recovers |
| V-OE5-6 | Degraded (DRT PE-RNF-009) | T | Unplug the IMU while running | App keeps running; `STALE` within 100 ms; resumes on reconnection |
| V-CAL-1 | OE6 prerequisite | T | Straight line, tape measure (scale first, thesis §3.3.8) | λ calibrated |
| V-CAL-2 | OE6 prerequisite | T | 360° spin | χ calibrated |
| V-CAL-3 | OE6 prerequisite | A | 1–2 h static recording; Allan variance of the gyro | σ_g, bias instability, initial b₀ |
| V-CAL-4 | PE-RF-011 | A | δ noise at real speed without slip | δ_th confirmed |
| V-OE6-1 | OE6 | T | 5 m straight line, markers | Final position error ≤ 3 % (15 cm) |
| V-OE6-2 | OE6 | T | 360° in place | Final heading error ≤ 3 % (10.8°) |
| V-OE6-3 | OE6, goal, general objective | T | 2 m UMBmark, clockwise and counter-clockwise, 3 s pauses | Final position error ≤ 24 cm; pose ≥ 10 Hz throughout |
| V-OE6-4 | OE6 (model) | A | Replay each recorded run in the app and in MATLAB; also the model's simulated scenario vs the rover estimate | Same bounds as V-OE6-1…3 |

### 16.3 Traceability

| Requirement | Design element | Tests |
|---|---|---|
| PE-RF-006 (≥ 4 agents) | §4, §7 | V-OE4-1 |
| PE-RF-007 (fixed-size messages, multi-buffer) | §8.1, D4 | V-U-8 |
| PE-RF-008 (accumulators unconverted to their user) | §7.1, §8.2, D2 | V-U-1, V-I-1 |
| PE-RF-009 (odometry, rest) | §7.2, §9.1 | V-U-3, V-U-4 |
| PE-RF-010 (EKF, persistent x̂, P) | §7.3, §9.2 | V-U-5, V-U-6 |
| PE-RF-011 (slip) | §7.3 step 3 | V-U-7, V-CAL-4 |
| PE-RF-012 (publish pose) | §11.1 | V-OE5-2 (rate), decoder check |
| PE-RF-013 (log pose + GPS, common clock) | §7.4, §7.5, §11.2 | V-OE2-2, log inspection |
| PE-RNF-001 (≥ 10 Hz) | D3 | V-OE5-2 |
| PE-RNF-002 (< 100 ms) | §10 | V-OE5-1 |
| PE-RNF-003 (≥ 99 %, ≤ 4 consecutive) | §8.1 link stats | V-OE4-2 |
| PE-RNF-005 (15 min, no leaks) | Static pools, §7.6 | V-OE5-3, V-LIB-1 |
| PE-RNF-006 (non-interference) | D1, §6 | V-OE5-4, V-OE5-5 |
| PE-RNF-007 (C99, no dynamic memory, portability) | D12, §15.2 | Inspection + `-std=c99 -Wall -Wextra` build |

---

## 17. Changes required in the thesis report (*Olympus_pose_TFG.pdf*)

The design keeps the estimator of §3.2 and the four-agent architecture of §3.4, but
several statements in the report no longer match it or contradict each other. Each item
gives the section, what must change and why. Suggested Spanish wording is included where
a paragraph must be rewritten.

### 17.1 §3.4.2 *Canal de sensores* — port sharing (major)

**Current text:** the `RAW:` frame goes over the same USART0 link and "su prefijo la
distingue de la telemetría, de modo que el software existente no se ve afectado".

**Problem:** the statement is not true, for two reasons:
1. The prefix does not let a second process read the port: `rover_bridge` opens it
   exclusively.
2. The prefix does not protect `olympus_hlc` either. Its loop reads **one line per cycle**
   with `recv_tlm()`, so at 33–50 `RAW:` lines per second the `TLM:` lines it depends on
   would be buried in a backlog (§3.2).

**Change:** add the mux, its `RAW:` filtering, the reset-on-open detail and the fact that
the frame is only sent when the IMU read succeeds. Suggested text:

> El puerto serie lo abre en exclusiva el software existente (`olympus_hlc`, mediante
> `rover_bridge`), así que el subsistema no puede leerlo directamente. Además, el prefijo
> por sí solo no basta para que el software existente no se vea afectado: su lazo lee una
> sola línea del puerto por ciclo, y a 50 tramas `RAW:` por segundo las tramas de
> telemetría quedarían rezagadas. Por eso, un proceso independiente, el multiplexor
> `llcmux`, toma el puerto y entrega a `olympus_hlc`, a través de una pseudoterminal con
> el mismo nombre de dispositivo, el flujo original sin las tramas `RAW:`, de modo que ve
> exactamente lo mismo que antes de activar la IMU. Al mismo tiempo, reenvía al agente de
> adquisición las tramas `RAW:` con la marca de tiempo de llegada en el reloj monotónico
> del HLC. El código de `olympus_hlc` no cambia; solo cambia el nombre que la regla de
> udev asigna al puerto real. Si el multiplexor falla, `olympus_hlc` deja de enviar `PING`
> y el watchdog del LLC detiene los motores en menos de 2 s, de modo que la falla es
> segura. El firmware solo emite la trama cuando la lectura de la IMU tiene éxito; por
> eso, una falla de la IMU interrumpe también los datos de los encoders.

Also in this paragraph: replace "el agente de estimación descarta las muestras con un Δt
fuera de (0, 0,5] s o con saltos de cuenta imposibles" with the split of §7.1–§7.2
(syntax in acquisition, plausibility in fusion) and the Δt rule of §17.8.

### 17.2 §3.4.3 *Descomposición en agentes* — cumulative fusion output (major)

**Current text (fusion):** "Convierte los acumuladores en el avance Δs, el giro Δθ_enc y
la velocidad angular ω_enc…".

**Problem:** it contradicts the rule of §3.3.6 ("convertir a incrementos en el último
paso, nunca antes de un buzón que pueda perder mensajes"), because the fusion→estimation
mailbox can drop messages too.

**Suggested text:**

> **Agente de fusión de datos.** Convierte los acumuladores en magnitudes físicas
> acumuladas: la distancia recorrida S y el giro acumulado de la odometría Θ_enc, que son
> combinaciones lineales de los acumuladores, y la suma de las mediciones del giroscopio Ω
> con su número de muestras. Además detecta el reposo (PE-RF-009) y descarta tramas con
> tiempos o saltos de cuenta imposibles. Corresponde al archivo `sep_fusion_step` del
> modelo de referencia, que usa `sep_odometry` sin cambios.
>
> **Agente de estimación.** Calcula Δs, Δθ_enc, Δt y la velocidad media del giroscopio
> como diferencias contra los últimos valores que él mismo procesó, ejecuta el EKF de la
> sección 3.2, conserva el estado y su covarianza entre ciclos y detecta el deslizamiento
> (PE-RF-010 y PE-RF-011). Así, un mensaje descartado en cualquiera de los dos buzones
> solo cuesta resolución en el tiempo, no distancia. Corresponde al archivo
> `sep_estimation_step`, que usa `sep_ekf_step` sin cambios.

**Also:**
- Add the **GPS agent** paragraph: "Lee las sentencias NMEA del receptor, las valida y
  les asigna la marca de tiempo del HLC; entrega cada posición al agente de comunicación
  para el registro (PE-RF-013). Nunca entra al filtro."
- Communication agent: name the outputs (UDP datagram to localhost, logs, control port).
- AMS paragraph: suspensions are requested by the communication agent on an operator
  command, because AMS replies arrive in the requester's mailbox (§3.6).
- **Figure 3.18:** add `llcmux` outside the CMAES process (between the UART and
  acquisition, with the branch to `olympus_hlc`), the GPS agent, and change the
  fusion→estimation label from "∆s, ∆θ, reposo" to "S, Θ_enc, Ω acumulados; reposo".
- "al menos cuatro agentes" stays true with five.

### 17.3 §3.4.4 *Mensajes entre agentes*

Add three points:

1. The accumulator rule applies **along the whole path**: every message carries cumulative
   quantities, and only the estimation agent takes differences.
2. **Activation is event-driven:** each agent waits on its mailbox. This removes the
   phase delay of a 20 ms timer and the "equal rates" case of §3.3.6.
3. **Buffer rotation detail:** the producer advances to the next buffer only after a
   successful send. Each message carries a per-link sequence number used to measure the
   delivery rate of PE-RNF-003.

### 17.4 §3.4.5 *Presupuesto de latencia*

- Replace Table 3.8 with the table of §10.2 (adds `llcmux`; total ≤ 77 ms, margin ≈ 23 ms).
- Define the measurement points: start = LLC sampling (bounded until the clock is fixed),
  HLC start = arrival stamped by `llcmux`, end = publication.
- Note that the real-time priorities are kept **below** the kernel IRQ threads (FIFO 50).
  The current text only says "prioridades escalonadas".
- State that OE5 ("hasta la estimación") and PE-RNF-002 ("hasta la publicación") use
  different end points, and that the stricter one is measured.

### 17.5 §3.3.2 and §3.3.7 — simulation model

- §3.3.2 says the estimation agent "se activa cada 20 ms". With D3 it is event-driven. The
  model gets an `ag.est_mode` option (§9.4) and the text should say both were compared.
- §3.3.7 lists "La fusión de datos y la estimación corren en el mismo paso" as a
  difference. With the second mailbox added to `hlc_step` (§9.4) this item is removed.
  V-I-1 also tests it in the implementation.
- §3.3.4 lists ten tests. Add tests 11–13 of §9.4.

### 17.6 §3.2.10, Table 3.5 — correspondence with the implementation

- Keep the four existing rows: the four core files are not modified.
- Add two rows:
  - `sep_fusion_step`: "Acumulación de S, Θ_enc y Ω a partir de `sep_odometry`,
    plausibilidad y detección de reposo" (data fusion agent).
  - `sep_estimation_step`: "Diferencias Δs, Δθ_enc, Δt y velocidad media del giroscopio;
    pasos intermedios en intervalos largos; llamada a `sep_ekf_step`" (estimation agent).
- §3.2.10 and §3.3.4 say "Estos cuatro archivos son los únicos que se convertirán en código
  C". This becomes six: the four core files plus the two wrappers, generated with MATLAB
  Coder.

### 17.7 §3.2.7 — measurement model with a gap

Add one sentence: when a step spans Δn_g gyro samples, z is their mean. Σ_v is kept at
σ_g², which is conservative by a factor Δn_g during the rare gaps and leaves eq. (3.7)
unchanged.

### 17.8 §3.2.4 vs §3.4.2 — inconsistent Δt rule

§3.2.4 **clamps** Δt to [1 ms, 500 ms]; §3.4.2 says samples outside (0, 0,5] s are
**discarded**. With cumulative inputs a long interval still has an exact Δs, so clamping
would distort ω_enc and discarding would lose distance. Proposed single rule (§7.2–§7.3):
- fusion rejects only impossible times (tick backwards, jumps > 2 s) and handles LLC
  resets;
- `sep_ekf_step` keeps its clamp to [1 ms, 500 ms] unchanged, as a last-line guard;
- `sep_estimation_step` splits intervals longer than 500 ms into sub-steps of ≤ 500 ms,
  so the clamp never triggers in normal operation, and flags `GAP`;
- beyond 10 s it only re-references.

Update both sections to the same rule.

### 17.9 §3.3.6 *Hallazgos*

- In "Los acumuladores deben llegar sin convertir…", add that the rule is applied to the
  fusion→estimation mailbox by sending cumulative quantities (decision D2), and that a
  corrupted but accepted frame is undone by the next one.
- In "La política del buzón", add that event-driven activation makes the full-mailbox case
  rare, and that `MAES_QueueOverwrite` is no longer required.

### 17.10 §3.1 — CMAES port

Add the library changes L1–L6 of §14 to Table 3.3, or to a new paragraph in §3.1.4. The
monotonic clock and the affinity call are design changes, not bug fixes.

### 17.11 Chapter 1 — figures and scope

- Figure 1.1: add `llcmux` between the UART and both consumers (optional but clearer).
- §1.6 is consistent (GPS outside the filter).
- The text says "MPU-9250" while the firmware uses an MPU-6050 driver. It accepts the
  MPU-9250, but the report should say so once (§3.5 or §3.4.2).

### 17.12 Chapter 4 — status and next steps

- Table 4.1, OE4 "Pendiente": "Implementación de cinco agentes y del multiplexor; prueba
  de entrega".
- §4.3 "El diseño de los agentes sigue abierto": now closed by this document; the open
  item becomes the evaluation of the second mailbox (V-I-1).
- §4.4 step 5: "Cerrar el diseño de los cuatro agentes…" → "Implementar los cinco agentes
  y el multiplexor `llcmux` según ARQ-PE-003, con mensajes acumulados, envío con tiempo
  límite cero y rotación de búferes".
- Table 4.2 cites the data contract "ICD-LLC-002 v1.1". The repository has
  `ICD-PE-002 v1.0` (and the system ICD is ICD-LLC-001 v1.3). Unify the name and version.

### 17.13 Companion documents (not the PDF, but referenced by it)

- **DRT-SEP-001** (repo copy v0.2) uses older requirement numbers (PE-RF-004 … PE-RF-010).
  Its PE-RF-008 says the estimator "adapta la matriz R", while thesis §3.2.9 correctly
  adapts **Q**. Its latency table (≈ 74 ms, 3-state EKF) predates Table 3.8. Update it to
  the numbering of §3.4.1.
- **ICD-PE-002** (repo copy v1.0) describes a dedicated 50-byte binary channel. The thesis
  says the current channel is the `RAW:` ASCII frame on USART0, with a **55-byte** binary
  frame as the target. Issue v1.1 documenting the `RAW:` frame as current, the binary frame
  as target (fix 50 vs 55 bytes), and the `llcmux` sharing.
- **HLC documentation** (`olympus-hlc-rpi5/docs`, not edited by this project): with the pose
  layer installed, `testing.md` ("`/dev/arduino_mega -> ttyACM0`") and `architecture.md`
  ("udev symlink → ttyACM0") no longer describe the device. `/dev/arduino_mega` then points
  to a pty and the real port is `/dev/arduino_mega_hw`. Document this in the integration
  manual of this project, and tell the owner of the HLC repository.

### 17.14 Simulation model repository (`Simulation Model v2.0`)

These are changes to the model code and its README, which the thesis cites in §3.3:

- Add `sep_fusion_step.m`, `sep_estimation_step.m`, the second mailbox and
  `ag.est_mode` in `hlc_step.m`, and tests 11–13 (§9.4).
- Fix the reference-on-reject behaviour of `hlc_step.m` (keep the last accepted frame as
  the reference). It costs about two frame intervals of distance per rejected frame today.
- Reconcile `slip_thresh` (0.10 rad/s in `sep_ekf_params`, 0.006 rad/s in thesis §3.2.9).
- The README says the `hlc_step` emulates "four agents"; it models acquisition, estimation
  (which also does the fusion) and communication. After the change it models acquisition,
  fusion, estimation and communication as separate stages; the GPS agent is not modelled.
- The README codegen boundary says "only the four core files" become C. Update it to six.

### 17.15 New decision table for §3.4

Section 3.2 closes with an architecture-decision table (Table 3.6), but §3.4 has none. Add
one with the same format (decision / options evaluated / justification), so the
alternatives raised during the design, including the adviser's, are on record:

| Decisión | Opciones evaluadas | Justificación |
|---|---|---|
| Compartir el enlace con un multiplexor independiente (`llcmux`) | Canal 2 dedicado (USART libre + adaptador); modificar `rover_bridge`; agente pasarela dentro de la aplicación CMAES | No requiere cambios de firmware ni de código del software existente. Una falla de la aplicación de estimación no corta el enlace de `olympus_hlc`. Con el agente pasarela, cualquier caída, reinicio o suspensión de la aplicación detendría el róver |
| Retirar las tramas `RAW:` del flujo que recibe `olympus_hlc` | Copia idéntica byte a byte | `olympus_hlc` lee una línea por ciclo; a 50 tramas por segundo la telemetría quedaría rezagada |
| Mensajes acumulados entre fusión y estimación | Incrementos; incrementos con arrastre | Un mensaje perdido cuesta resolución temporal y no distancia; ω_enc y δ siguen siendo coherentes en los huecos |
| Activación por eventos | Agente de estimación periódico (20 ms) | Elimina hasta 20 ms de retardo de fase y el caso de tasas iguales de la sección 3.3.6 |
| GPS como quinto agente | Proceso aparte | Pose y referencia comparten reloj y registro (PE-RF-013) |
| Núcleo en C generado con MATLAB Coder | Código escrito a mano | Una sola implementación del algoritmo, la del modelo de referencia |

---

## 18. Open items and risks

| ID | Item | Needed for | Proposed action |
|---|---|---|---|
| S-1 | **Spike:** confirm `rover_bridge` works through a pty. Run `tools/spike_pty_check.py` on the rover as root; it uses a fake LLC and never touches the real port | D1 | First task; if it fails, fall back to a tee hook in `rover_bridge` (needs owner approval) |
| Q-1 | ~~pty naming~~ — **closed:** udev rename (from the pose layer, via `.bbappend`) + `llcmux` publishes `/dev/arduino_mega` (§3.2, §15.3, D13) | — | Inform the owner of `olympus-hlc-rpi5`; that repository is not edited |
| Q-2 | ~~Model files missing~~ — **closed:** `Simulation Model v2.0` reviewed; changes listed in §9.4 and §17.14 | — | — |
| Q-3 | ~~Hand-written vs generated~~ — **closed:** MATLAB Coder from the unchanged core files + two wrappers (D14); MATLAB and MATLAB Coder licenses confirmed (2026-10-05) | — | — |
| Q-4 | ~~GPS connection~~ — **closed:** GPIO UART `/dev/ttyAMA0` (§7.5) | — | Before connecting: TX level, `/dev/ttyAMA0` present, no console on it (§7.5 checklist) |
| Q-5 | ~~How CMAES is built~~ — **closed:** own Yocto layer, built at image build time; `olympus-pose` recipe goes in the same layer (§15.3, L9) | — | — |
| Q-6 | ~~Gateway agent owning the LLC port~~ — **closed (2026-10-05): not adopted.** Keep the separate `llcmux` process (D1). Reason: with the port inside the pose application, any crash, restart or AMS suspension of the experimental application would cut `olympus_hlc`'s link and stop the rover, which is hard to reconcile with PE-RNF-006; the separate process keeps the design simpler | — | Record the alternative and the reason in the thesis decision table (§17.15) |
| R-1 | LLC clock (65 % hypothesis) and ≈ 33 Hz frame rate with v2.20 | OE2, OE6 | §10.4 measures it; OE4 tested with the 50 Hz emulator meanwhile |
| R-2 | Counts-per-revolution anomaly (Hall bounce) | λ, `rate_max_counts_s`, rest detection (bounce would prevent `STILL`) | V-CAL-1; inspect counts while stationary |
| R-3 | Tail latency on a non-RT kernel with vision load and the powersave governor | OE5 | V-OE5-1 first; then optional `isolcpus`/governor (§10.1) |
| R-4 | IMU failure stops all frames (fw v2.20) | DRT PE-RNF-009 | Report as a limitation, or propose sending the frame with an `imu_ok` field in a future firmware |

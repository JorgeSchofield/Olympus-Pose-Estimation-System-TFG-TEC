# LLC firmware patch: timer-driven main loop (`[PATCH timer-loop]`)

Patch for the Olympus rover's low-level controller (ATmega2560, Rust), firmware v3.0
(MPU-9250 driver, `RAW:` frame with `temp` and `imu_ok`). It makes the main loop run at
a real 50 Hz and makes the frame time stamp count real milliseconds.

**Status: built, flashed and used in all the rover tests of the thesis.** Measured with it:
49.88 Hz `RAW:` frame rate (interval p50 20.0 ms, p99 22.0 ms) and an LLC clock scale of
1.0024 against the HLC clock. The remaining 0.24 % is the board's ceramic resonator, not the
loop. USART0 transmission is still blocking (see "What is not included" below).

---

## Contents of this folder

| File | What |
|---|---|
| `src/main.rs` | Patched file (replace `src/main.rs` of the firmware) |
| `src/config.rs` | Patched file (replace `src/config.rs`) |
| `llc_timer_loop.patch` | The same changes as a unified diff (`git apply` / `patch -p1`) |
| `original/main.rs`, `original/config.rs` | The base the patch was written against (from `rover-low-level-controller-patched.rar`, firmware v3.0, 2026-10-07) |
| `README.md` | This file |

Every changed place in the code is marked with a `[PATCH timer-loop]` comment (in Spanish,
like the rest of the firmware). No other file of the firmware changes.

---

## Why: what the rover measurements showed

From the rover (build `all-bts7960,ramp-converge,idle-disable,no-stall,no-oc`; `llcmux`
statistics over 11 870 frames and the pose application's `stats.csv`):

| | Measured | Requirement |
|---|---|---|
| `RAW:` frame rate | **34.0 Hz** (interval p50 28.1 ms, p99 45.0 ms, max 49.0 ms) | ≥ 50 Hz (objective OE2) |
| LLC clock (`llc_clock_scale`) | **1.465**: the tick runs at 68 % of real time | 1.00 |

The cause is the end of the main loop:

```rust
elapsed_ms = elapsed_ms.wrapping_add(LOOP_MS);   // +20 "ms" per cycle
arduino_hal::delay_ms(LOOP_MS);                   // then wait another 20 ms
```

A cycle therefore lasts **20 ms + the work done in the cycle**, and the time stamp adds 20
per cycle whatever the cycle really took. The measured intervals match the firmware's
schedule exactly:

| Cycle | Work | Cycle length |
|---|---|---|
| every cycle | IMU read + blocking send of the ≈ 90-byte `RAW:` line (≈ 7.5 ms at 115 200 baud) ≈ 8 ms | 28 ms (the median) |
| every 25th (`SEN_READ_PERIOD`) | + analog read, 13 channels × 8 samples ≈ 10.8 ms | ≈ 39 ms |
| every 50th (`TLM_PERIOD`) | + analog read + blocking send of the TLM line (≈ 100 bytes ≈ 8.7 ms) | ≈ 48 ms (the max) |

Mean ≈ 29.0 ms predicted against 29.3 ms measured. Two consequences:

1. **OE2 fails**: 34 Hz instead of ≥ 50 Hz. A build with the stall and over-current checks
   enabled was even slower (26.7 Hz, clock scale 1.87, captures of 2026-10-06).
2. **The tick is not a clock.** The HLC estimator computes Δt from the tick, so a wrong
   tick gives wrong turn rates. The full simulated UMBmark ends 45 % off with the tick as
   time base, against ≈ 1 % with the HLC arrival time. Until now the HLC had to use
   `est.dt_source = 1` (arrival time) to work around it.

The amount of work per cycle is not the problem: 8 ms typical, 19 ms worst once the slow
tasks are spread out. It fits in 20 ms. The problem is that the loop adds its 20 ms on top
of the work instead of measuring 20 ms from the start of the cycle.

---

## The changes

### 1. A 1 ms time base from the Timer0 overflow interrupt (`main.rs`)

**What.** New `TIMER0_OVF` interrupt handler, a `millis()` function and `timebase_init()`,
placed after the encoder interrupt handlers. `timebase_init()` is called once in `main()`,
right before `avr_device::interrupt::enable()`.

**How it works.** Timer0 in 8-bit Fast PWM mode with prescaler 64 overflows every
64 × 256 / 16 MHz = **1024 µs**. Each overflow adds 1 ms to a 32-bit counter and 24 µs to a
fraction, kept in units of 8 µs (+3 per overflow, 125 = 1 ms). When the fraction reaches
1 ms, one extra ms is added, so the counter does not drift: it is as exact as the
crystal. This is the same method as Arduino's `millis()`. `millis()` reads the counter
with interrupts disabled for a few cycles, because a 32-bit read is not atomic on an
8-bit AVR.

**Why Timer0, and why the PWM is not affected.** In the `mixed-drivers` and `all-bts7960`
builds Timer0 already runs exactly in this mode (`Timer0Pwm::new(dp.TC0, Prescale64)`) to
drive D4 (OC0B, RL LPWM) and D13 (OC0A, RR LPWM). Enabling its overflow interrupt
(`TOIE0`) adds an interrupt; it does not change the counter, the period or the duty
cycles. `timebase_init()` uses read-modify-write on `TCCR0A`/`TCCR0B`, so the PWM output
bits (`COM0A`/`COM0B`) are never touched. In the builds that do not use Timer0 for PWM, the
same call puts it in that mode without outputs, so the time base works in **every feature
combination**. Nothing else in the firmware uses Timer0 (checked).

**Cost.** One short interrupt (a few µs) every 1.024 ms: about 0.3 % of the CPU. The
encoder interrupts can be delayed by those few µs, which is far below the time between
encoder edges.

### 2. Absolute-deadline loop instead of `delay_ms(LOOP_MS)` (`main.rs`)

**What.** A variable `next_deadline` is set before the main loop. The `delay_ms(LOOP_MS)` at
the end of the loop is replaced by a wait until `next_deadline`, which then advances by
`LOOP_MS`.

**Why it gives exactly 50 Hz.** The deadline advances by 20 ms per cycle *regardless of
how long the cycle took*, so the mean period is exactly `LOOP_MS`:

- **normal cycle** (work < 20 ms): waits for the rest of the 20 ms, so the period is exact;
- **long cycle** (finished late, by less than one period): does not wait, and the next
  cycle starts at once. Because that cycle is short, the loop is back on its 20 ms grid
  after one or two cycles. No frame is skipped and the mean rate stays at 50 Hz;
- **late by a whole period or more** (an exceptional block, e.g. a sensor timeout): the
  deadline is re-synchronized to the current time. Otherwise the loop would run several
  cycles back-to-back to "catch up" on time that is already lost.

The comparisons use a signed difference of `u32` values, so they stay correct when
`millis()` wraps after 49.7 days.

**Interrupts during the wait.** It is a busy wait, as `delay_ms` was. The encoder,
`USART0_RX`, `USART2_RX` and Timer0 interrupts keep running during it exactly as before,
so the USART receive ring buffers work the same way.

### 3. The `RAW:` tick is the real sampling time (`main.rs`)

**What.** In step 3 of the loop, `let sample_ms = millis();` is taken right after the IMU
read and immediately before the six encoder counters are read. That value is written as
the `tick` field of the `RAW:` frame (it was `elapsed_ms`). The frame format is
unchanged: `RAW:tick:ax:ay:az:gx:gy:gz:encL:encR:temp:imu_ok`.

**Why.** The HLC computes the time step as the difference between consecutive ticks. With
the real instant at which the counters were read, Δt is the true time between samples,
including the cycle-to-cycle jitter. Before, Δt was always a multiple of 20 ms, whatever
the real interval was. The IMU is read about 1 ms before the counters; that offset is
constant, so it cancels in the differences.

**`elapsed_ms` removed.** It was only used for the `RAW:` tick and for
`sensor_frame.tick_ms`, the `t=…ms` field of the TLM line. The TLM field is now set from
`millis()` at the end of each cycle, so it is real time as well.

### 4. Staggered slow tasks (`config.rs` + `main.rs`)

**What.** Two new constants in `config.rs`, `SEN_PHASE = 2` and `TLM_PHASE = 37`, used as
the initial values of `sen_counter` and `tlm_counter` in `main.rs` (both were 0). The TLM
period stays 50 cycles, the analog read 25 and the HC-SR04 5.

**Why.** With every counter starting at 0, the HC-SR04 scan, the full analog read and the
TLM line all ran in the same cycle every 50 cycles: the 48–49 ms cycle. A counter that
starts at F with period P fires at cycle P − F and then every P cycles:

| Task | Period | Phase | Fires at cycles | Shares a cycle with |
|---|---|---|---|---|
| HC-SR04 | 5 | 0 | 0 (mod 5) | — |
| Analog read | 25 | 2 | 23 (mod 25), i.e. 3 (mod 5) | never the HC-SR04 |
| TLM | 50 | 37 | 13 (mod 50), i.e. 3 (mod 5) and 13 (mod 25) | never the HC-SR04, never the analog read |

Worst cycle ≈ 8 ms + 10.8 ms (analog) ≈ 18.8 ms, inside the 20 ms period. The fast
over-current check (`SEN_FAST_PERIOD = 3`, ≈ 1.25 ms) can still land on an analog cycle,
giving ≈ 20 ms. That small, occasional overrun is absorbed by change 2.

**Compile-time check.** A `const _: () = { assert!(…) }` block in `config.rs` checks
that the phases are below their periods, that the periods divide each other, and that
none of the three tasks can share a cycle. If someone changes a period or a phase so that
two slow tasks coincide again, the firmware no longer compiles.

### 5. Documentation comments (`config.rs`)

`LOOP_MS` is now documented as the loop **period** (it was "duration of each cycle", which
was not true). `TLM_PERIOD`'s comment notes that "~1 s" is now 1.00 s real (it was
≈ 1.47 s).

---

## Effects to be aware of

Everything the firmware counts in cycles now has the real-time duration its constants
were written for. Today it runs ≈ 47 % slower than that:

| Constant | Cycles | Before (≈ 29.3 ms/cycle) | After (20 ms/cycle) |
|---|---|---|---|
| PING watchdog `WATCHDOG_MAX` | 100 | ≈ 2.9 s | **2.0 s** |
| Stall detection `STALL_THRESHOLD` | 50 | ≈ 1.5 s | 1.0 s |
| Periodic TLM `TLM_PERIOD` | 50 | ≈ 1.5 s | 1.0 s |
| HC-SR04 scan `HC_READ_PERIOD` | 5 | ≈ 150 ms | 100 ms |
| Ramp steps (`RAMP_STEP_*` per cycle) | — | slower | as designed (≈ 47 % faster than today) |

**Check the PING watchdog with `olympus_hlc` in vision mode.** `olympus_hlc` sends a PING
once per control cycle, and in vision mode a cycle can last close to 2 s (inference time).
With the real 2.0 s watchdog the margin is smaller than before (it was really ≈ 2.9 s). If
the LLC goes to FAULT while vision is running, raise `WATCHDOG_MAX` (in
`state_machine/mod.rs`) or make `olympus_hlc` send PING more often.

**Ramps.** Acceleration ramps now reach their target in the real time the constants say.
Drive the rover once in manual mode to check the motors' feel.

---

## What is not included (optional next step)

**Interrupt-driven USART0 transmission.** `CommandInterface::send_response` waits for every
byte (`nb::block!`). The `RAW:` line costs ≈ 7.5 ms of blocking per cycle and the TLM line
≈ 8.7 ms. A transmit ring buffer emptied by the `USART0_UDRE` interrupt would remove that
wait: work per cycle would drop to ≈ 1–3 ms, worst case ≈ 13 ms, with a wide margin. It
is a larger change: a new TX buffer, an ISR, and care with buffer overflow when a burst of
log lines is sent at start-up. It is not needed to reach 50 Hz once changes 2 and 4 are
in. The simulation model can quantify it (`llc_params`: `tx_mode = 'interrupt'`).

---

## How to apply

The patch was written against the v3.0 sources in `rover-low-level-controller-patched.rar`.

- **Firmware unchanged since that version:** copy `src/main.rs` and `src/config.rs` over the
  firmware's files.
- **Firmware changed since then:** apply the diff, which only touches the places marked
  `[PATCH timer-loop]`:
  ```bash
  cd rover-low-level-controller-patched
  git apply --check /path/to/llc_timer_loop.patch   # dry run
  git apply /path/to/llc_timer_loop.patch
  ```
  If a hunk does not apply, make the change by hand: each one is small and listed above.

Build and flash with the usual target, e.g. the build used in the tests:

```bash
make flash-imu
```

or `cargo +nightly run --release ... --features all-bts7960,ramp-converge,idle-disable,...`
as in the Makefile. The patch adds no feature flag: it applies to every build.

**Possible compile issues** (the code was checked by hand only):

- the interrupt name `TIMER0_OVF` comes from `avr-device`'s ATmega2560 vector table. If the
  compiler does not find it, list the names with `cargo doc` for `avr-device`
  (`atmega2560::Interrupt`);
- if an unused-variable warning appears in a `no-mpu` build, it is harmless: `sample_ms`
  only exists when the IMU is compiled in.

**Rollback:** copy back `original/main.rs` and `original/config.rs`.

---

## How to verify on the rover

1. **Rate and clock, with `llcmux` and `olympus-pose` running** (5 min at rest is enough):
   - the `llcmux` statistics line (`tail -f /var/log/llcmux.log`) should show
     `RAW interval p50 ≈ 20.0 … (≈ 50.0 Hz)`. Before: p50 28.1 ms, 34.0 Hz;
   - `stats.csv` → `llc_clock_scale` ≈ **1.00** (before 1.465);
   - the TLM `t=…ms` field should advance ≈ 1000 ms per real second.
2. **Tail of the interval distribution:** `RAW interval p99` should drop from 45 ms to
   ≈ 20–21 ms (the max may show an occasional ≈ 20–21 ms cycle and its short follower).
3. **Motors and watchdog:** manual drive (ramps); vision mode for a few minutes (no
   spurious FAULT from the PING watchdog, see above).
4. **Then the HLC time base:** with a real-time tick, `est.dt_source = 0` (LLC tick) and
   `1` (HLC arrival time) give the same result. 0 avoids the USB transfer jitter, but
   either works; set it in `sim_params.m` and re-export `pose.conf`. Confirm with
   one recording replayed both ways (`hlc_replay` in MATLAB or `olympus-pose -r`).

With these, objective OE2 (≥ 50 Hz on the HLC clock) can be measured on the rover, and the
OE4 15-minute test can run on real frames at 50 Hz instead of the emulator.

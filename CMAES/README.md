# CMAES for Linux — pthreads port

A native POSIX/pthreads port of **CMAES**, a C implementation of the FreeMAES
multi-agent framework, built for the Pose Estimation system on the **Olympus**
rover (Raspberry Pi 5, Linux). Originally, CMAES only ran on FreeRTOS; this
repo adds a Linux backend with the same public API, so agent code looks the
same regardless of which backend it runs on.

## Background

[MAES](https://github.com/AndresConejoBoza/CMAES) is a framework for
multi-agent embedded systems designed by C. Chan-Zheng at TU Delft
(**FreeMAES**). Andrés Conejo Boza reimplemented it in plain C on top of
FreeRTOS (**CMAES**), using a hand-rolled "vtable" pattern to fake OOP:
structs hold both data and function pointers, wired up by `Constructor*`
functions. On top of that sit FIPA-ACL-inspired agents, mailboxes, an Agent
Management System (AMS) that handles register/suspend/resume/kill/restart,
organizations (groups with roles/affiliations), and Cyclic/OneShot
behaviours (setup → loop action → done).

This project exists because a thesis Pose Estimation system for Olympus
needs that same agent/behaviour structure — sensor-input agents, a
fusion/estimation agent, message-passing between them — but running on
**Linux on a Raspberry Pi 5**, not FreeRTOS. See
[`docs/README.md`](docs/README.md) for the full analysis of
the original code and the design decisions behind the port.

## Repository layout

```
libCMAES_pthreads/           native POSIX/pthreads backend (this project)
  include/CMAES.h            public API, unchanged from the original
  src/                       backend implementation
linux_demo/
  sender_receiver/           two-agent mailbox smoke test
  rock_paper_scissors/       three-agent test: broadcast, suspend/resume via
                             the AMS, get_state, OneShotBehaviour
  rps_stress/                the OE3 indicator test (50 Hz, 45 000 messages)
docs/README.md               findings from reading the original code + every
                             design decision made while porting it
CMAES_Yocto_layer/meta-olympus-pose/
                             Yocto layer: this library, its demos, and the pose
                             application (../agents) with llcmux
CMakeLists.txt               top-level build (libCMAES_pthreads + linux_demo)
```

The pose estimation application built on this library is in
[`../agents`](../agents/README.md); it links the library from the same commit.

The original FreeRTOS and libcsp backends are not vendored here; see the
[upstream repository](https://github.com/AndresConejoBoza/CMAES) for them.

## Building and running on a development host

Requires a C toolchain, CMake and pthreads — standard on Ubuntu or Raspberry
Pi OS (`sudo apt install build-essential cmake` if not already present).
**An Ubuntu VM works fine**; nothing here depends on Raspberry Pi hardware.

```bash
cmake -S . -B build
cmake --build build

./build/linux_demo/sender_receiver/cmaes_sender_receiver_demo
./build/linux_demo/rock_paper_scissors/cmaes_rock_paper_scissors_demo
./build/linux_demo/rps_stress/cmaes_rps_stress_demo 45000 20 3
```

- **`sender_receiver`**: two agents exchange a message once a second through
  the platform's Agent Management System — the minimal test that agent
  creation, registration and mailbox send/receive work end to end.
- **`rock_paper_scissors`**: two players broadcast a random throw to a
  referee agent every round; the referee suspends both via the AMS once
  it's seen both throws, announces a winner, then resumes them for another
  round. Exercises more of the API: multi-receiver broadcast,
  `OneShotBehaviour`, suspend/resume, and `get_state` — the same primitives
  the Pose Estimation agents will use to pause/resume subsystems. Runs
  indefinitely; `Ctrl+C` to stop.
- **`rps_stress`**: the OE3 indicator test. Same game, but each player
  throws every `period_ms` (default 20 ms, 50 Hz) with `MAES_DelayUntil`,
  every round suspends and resumes both players through the AMS, and the
  run stops after a target number of messages (default 45 000). Each
  payload carries a per-player sequence number and a checksum, so lost,
  duplicated or overwritten messages are detected; RSS is sampled after
  warm-up and at the end. Prints a summary and exits with 0 (`PASS`) or 1.
  Arguments: `[messages] [period_ms] [cpu]`; run as root for `SCHED_FIFO`.

Both are direct ports of the demos the original FreeRTOS repo ships;
porting the second one surfaced and fixed two real bugs in the original
library/demo — see `docs/README.md` §8.

The one behaviour that differs between a VM and the rover is real-time
scheduling privilege (`docs/README.md` §3). On an unprivileged VM the
demos still run correctly, but agent priorities are advisory rather than
enforced, and a one-time warning is printed to stderr.

## Building with Yocto for the Olympus HLC

The rover's HLC does not run Raspberry Pi OS. It runs a Yocto-built image
(`olympus-image`, from the
[olympus-hlc-rpi5](https://github.com/Alonso11/olympus-hlc-rpi5) build tree),
so the library and demos are cross-compiled as part of that image rather
than built natively on the board.

**Target environment**

| | |
| --- | --- |
| Yocto release | scarthgap (5.0 LTS), `DISTRO = "poky"` |
| Machine | `raspberrypi5`, aarch64 Cortex-A76 |
| Kernel | linux-raspberrypi 6.12.x (pinned by the rover layer) |
| Toolchain | GCC 13.3, glibc |
| Image | `olympus-image` |

### The layer

Integration is done with the Yocto layer `meta-olympus-pose`, which lives in
this repository at `CMAES/CMAES_Yocto_layer/meta-olympus-pose`. It adds **no
source code**: it tells BitBake where this repository lives and how to build
it, and it also builds the pose application and `llcmux`:

```
meta-olympus-pose/
  conf/layer.conf                                  collection olympus-pose, scarthgap
  recipes-cmaes/cmaes/cmaes_0.1.0.bb               this library and its demos, pinned SRCREV
  recipes-cmaes/cmaes/files/*.service              systemd units for the demos (inert on sysvinit)
  recipes-pose/olympus-pose/olympus-pose_0.1.0.bb  the pose application and llcmux
  recipes-core/custom-udev-rules/                  real Arduino port -> /dev/arduino_mega_hw
  recipes-core/images/olympus-image.bbappend       adds the packages to the rover image
```

The layer directory is named `meta-olympus-pose`, not `meta-olympus`: the
rover's own layer already occupies that directory name and registers the
BitBake collection `meta-olympus`.

Both recipes pin `SRCREV` to a commit of this repository on branch `main`.
Bump it whenever the code changes, or BitBake keeps building the old
revision from its cache.

### Build

```bash
cd olympus-hlc-rpi5
git clone https://github.com/JorgeSchofield/Olympus-Pose-Estimation-System-TFG-TEC.git layers/olympus-pose-repo
source layers/poky/oe-init-build-env build
bitbake-layers add-layer ../layers/olympus-pose-repo/CMAES/CMAES_Yocto_layer/meta-olympus-pose

bitbake cmaes          # the recipe alone: minutes, catches most problems
bitbake olympus-image  # incremental, on the existing sstate cache
```

Nothing in `local.conf` needs to change. Details and caveats are in the
layer's [README](CMAES_Yocto_layer/meta-olympus-pose/README.md).

### Packages produced

Because the library is **static**, there is nothing to install on the board
at runtime — the archive is linked into whatever binary uses it.

| Package | Contents | Reaches the image |
| --- | --- | --- |
| `cmaes` | empty | — |
| `cmaes-staticdev` | `libcmaes_pthreads.a` | no (build sysroot only) |
| `cmaes-dev` | `CMAES.h` | no (build sysroot only) |
| `cmaes-demos` | the three demo binaries (`sender_receiver`, `rock_paper_scissors`, `rps_stress`) | yes, via the bbappend |

Verify after a build with:

```bash
oe-pkgdata-util list-pkgs -p cmaes
oe-pkgdata-util list-pkg-files -p cmaes
```

### Deploying without reflashing

Once the board runs an image that contains the recipe, pushing a changed
build over SSH takes seconds instead of a full reflash:

```bash
devtool modify cmaes
devtool build cmaes
devtool deploy-target cmaes root@<board>
```

Anything deployed this way lives only in the running rootfs and is lost on
the next flash.

## Verification on the Raspberry Pi 5

Both demos were built by the recipe above and run on a Raspberry Pi 5
booted from the rover image. The binaries land in `/usr/bin`:

```
/usr/bin/cmaes_sender_receiver_demo
/usr/bin/cmaes_rock_paper_scissors_demo
```

**Functional behaviour is identical to the development-host build.** Both
demos boot the platform, register their agents and run indefinitely with the
same output and the same timing as on an x86-64 VM — agent creation,
registration, mailbox send/receive, multi-receiver broadcast and AMS
suspend/resume all behave the same on aarch64.

**Real-time scheduling is active on the target.** Unlike an unprivileged VM,
the rover image runs the demos as root, so `pthread_setschedparam` succeeds
and agent priorities are enforced rather than advisory. The image ships
BusyBox `ps`, which has no `-eLo`, so the scheduling class is read from
`/proc` instead:

```bash
cmaes_rock_paper_scissors_demo > /tmp/rps.log 2>&1 &
PID=$!
sleep 2
awk '{ sub(/.*\) /, ""); print FILENAME, "policy="$39, "rtprio="$38 }' \
    /proc/$PID/task/*/stat
kill $PID
```

Field 41 of `/proc/<tid>/stat` is the scheduling policy and field 40 the RT
priority; the `sub` strips the `pid (comm)` prefix first, which shifts the
field numbers by two. Observed: `policy=0` on the main thread, which only
calls `boot` and waits, and `policy=1` (`SCHED_FIFO`) with a non-zero
`rtprio` on every agent and AMS thread. The fallback warning
(`docs/README.md` §3) was not emitted.

### OE3 indicator: passed (2026-10-07)

`cmaes_rps_stress_demo 45000 20 3`, as root on a Raspberry Pi 5 with the
Olympus image, pinned to CPU 3: 45 000 messages in 450.0 s (100 msg/s),
22 499 rounds with 44 998 AMS suspends and resumes each; 0 sequence gaps,
0 duplicates, 0 bad checksums, 0 send failures, 0 AMS errors, 0 suspend or
receive timeouts; RSS 1712 kB at baseline and at the end (growth 0 kB).
`RESULT : PASS`, milestone H3 closed (`agents/DESIGN.md` §14).

Delivery and latency of the pose application's own agent chain over 15
minutes are the OE4 and OE5 indicators, measured with the application's
`stats.csv` (see `agents/README.md`).

Two defects found while planning these measurements are fixed (see
`docs/README.md` §3, "Real-time helpers"):

- `agent_waitFunction` uses a **relative** `nanosleep`, so a periodic
  behaviour's period drifts. Periodic work now uses `MAES_DelayUntil`, an
  absolute `clock_nanosleep(TIMER_ABSTIME)` deadline on `CLOCK_MONOTONIC`.
- The mailbox condition variables used **`CLOCK_REALTIME`**, which steps
  when time sync lands after boot. They are now bound to `CLOCK_MONOTONIC`.

### Portability notes

- **GCC 14 / Clang 20 ready.** The `void`/`void*` vtable mismatches, the
  `NULL`-to-`pthread_t` assignment and the implicit function declaration
  inherited from the original library are fixed; the library builds as
  strict C99 with no warnings under Clang 20 (`docs/README.md` §4); the
  recipe builds it without any `-Wno-error` flag.
- **Init system: sysvinit** (confirmed on the board, 2026-10-07). The
  `.service` files the layer ships for the demos are installed but inert;
  run the demos from the shell. The pose application and `llcmux` ship
  sysvinit scripts.
- **The Raspberry Pi kernel is not `PREEMPT_RT`.** `SCHED_FIFO` gives
  priority ordering, not a bounded worst case.

## Status

- [x] Read and understood the original FreeRTOS implementation
- [x] Native pthreads backend (`libCMAES_pthreads`) implementing the same
      `CMAES.h` API
- [x] Builds and runs on a development host (Ubuntu VM, x86-64)
- [x] Cross-compiled with Yocto (scarthgap) for `raspberrypi5`
- [x] Verified running on a Raspberry Pi 5 with the Olympus rover image,
      with `SCHED_FIFO` active
- [x] OE3 indicator: 45 000 messages at 50 Hz with AMS suspend/resume, no
      failures, no memory growth (2026-10-07)
- [x] Pose Estimation agent application built on top (`../agents`, five
      agents, in the rover image through `meta-olympus-pose`)

## Credits

- MAES framework design: C. Chan-Zheng, TU Delft (FreeMAES)
- Original FreeRTOS C implementation (CMAES): Andrés Conejo Boza
  ([AndresConejoBoza/CMAES](https://github.com/AndresConejoBoza/CMAES))
- Olympus HLC Yocto image and `meta-olympus` layer: F. Gómez
  ([olympus-hlc-rpi5](https://github.com/Alonso11/olympus-hlc-rpi5))
- pthreads/Linux backend, Yocto integration and Pose Estimation work:
  this thesis project

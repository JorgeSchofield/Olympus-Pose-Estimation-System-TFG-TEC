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
[`docs/PORTING_NOTES.md`](docs/PORTING_NOTES.md) for the full analysis of
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
docs/PORTING_NOTES.md        findings from reading the original code + every
                             design decision made while porting it
CMakeLists.txt               top-level build (libCMAES_pthreads + linux_demo)
```

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

Both are direct ports of the demos the original FreeRTOS repo ships;
porting the second one surfaced and fixed two real bugs in the original
library/demo — see `docs/PORTING_NOTES.md` §8.

The one behaviour that differs between a VM and the rover is real-time
scheduling privilege (`docs/PORTING_NOTES.md` §3). On an unprivileged VM the
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

Integration is done with a separate Yocto layer, `meta-olympus-pose`, which
adds **no source code** — it tells BitBake where this repository lives and
how to build it:

```
meta-olympus-pose/
  conf/layer.conf                           collection olympus-pose, scarthgap
  recipes-cmaes/cmaes/cmaes_0.1.0.bb        fetches this repo at a pinned SRCREV
  recipes-cmaes/cmaes/files/*.service       systemd units for the demos
  recipes-core/images/olympus-image.bbappend adds cmaes-demos to the rover image
```

The layer directory is named `meta-olympus-pose`, not `meta-olympus`: the
rover's own layer already occupies that directory name and registers the
BitBake collection `meta-olympus`.

The recipe pins `SRCREV` to a specific commit of this repository. Bump it
whenever the port changes, or BitBake will keep building the old revision
from its cache.

### Build

```bash
cd olympus-hlc-rpi5
git clone <meta-olympus-pose remote> layers/meta-olympus-pose
source layers/poky/oe-init-build-env build
bitbake-layers add-layer ../layers/meta-olympus-pose

bitbake cmaes          # the recipe alone — minutes, catches most problems
bitbake olympus-image  # incremental, on the existing sstate cache
```

Nothing in `local.conf` needs to change.

### Packages produced

Because the library is **static**, there is nothing to install on the board
at runtime — the archive is linked into whatever binary uses it.

| Package | Contents | Reaches the image |
| --- | --- | --- |
| `cmaes` | empty | — |
| `cmaes-staticdev` | `libcmaes_pthreads.a` | no (build sysroot only) |
| `cmaes-dev` | `CMAES.h` | no (build sysroot only) |
| `cmaes-demos` | both demo binaries | yes, via the bbappend |

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
(`docs/PORTING_NOTES.md` §3) was not emitted.

### Not yet characterised

Functional correctness is established; timing is not. The demos run at 1 Hz
with no load and no instrumentation, so they say nothing about the
quantitative targets of the Pose Estimation subsystem. Still to measure:
period jitter of a cyclic behaviour at 50 Hz, message delivery rate with
depth-1 mailboxes under load, end-to-end latency across the agent chain, and
memory/thread stability over a long run.

Two defects found while planning those measurements, both relevant to the
rover and neither triggered by the demos:

- `agent_waitFunction` uses a **relative** `nanosleep`, so a periodic
  behaviour's period drifts by its own execution time plus wakeup latency
  on every iteration. Periodic work needs an absolute deadline
  (`clock_nanosleep` with `TIMER_ABSTIME` on `CLOCK_MONOTONIC`).
- The mailbox condition variables use **`CLOCK_REALTIME`**. The Pi 5 has no
  battery-backed RTC, so the wall clock steps when time sync lands after
  boot, and a `pthread_cond_timedwait` in flight across that step either
  returns immediately or blocks for the size of the jump.
  `pthread_condattr_setclock(..., CLOCK_MONOTONIC)` is the fix.

### Portability notes

- **GCC 14 and newer reject this code as written.** Since GCC 14,
  `-Wincompatible-pointer-types` and `-Wint-conversion` are errors by
  default, and the port inherits ~38 `void`/`void*` vtable mismatches from
  the original library, one `NULL`-to-`pthread_t` assignment, and one
  implicit function declaration. scarthgap's GCC 13 only warns. The recipe
  carries `-Wno-error=` flags as insurance; correcting the signatures is the
  real fix. See `docs/PORTING_NOTES.md` §5.
- **The rover image uses sysvinit**, not systemd, so the `.service` files
  the layer ships are installed but inert. Run the demos from the shell.
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
- [ ] Timing characterised against the subsystem's quantitative targets
- [ ] Pose Estimation agent system built on top

## Credits

- MAES framework design: C. Chan-Zheng, TU Delft (FreeMAES)
- Original FreeRTOS C implementation (CMAES): Andrés Conejo Boza
  ([AndresConejoBoza/CMAES](https://github.com/AndresConejoBoza/CMAES))
- Olympus HLC Yocto image and `meta-olympus` layer: F. Gómez
  ([olympus-hlc-rpi5](https://github.com/Alonso11/olympus-hlc-rpi5))
- pthreads/Linux backend, Yocto integration and Pose Estimation work:
  this thesis project

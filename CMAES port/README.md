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
CMAES/
  libCMAES_FreeRTOS/          original FreeRTOS backend (upstream, untouched)
  libCMAES_CSP/                original, partial libcsp backend (upstream, untouched)
  libCMAES_pthreads/           new: native POSIX/pthreads backend (this project)
linux_demo/
  sender_receiver/             two-agent mailbox smoke test
  rock_paper_scissors/         three-agent test: broadcast, suspend/resume via
                                the AMS, get_state, OneShotBehaviour
Visual Studio/                 original Windows/FreeRTOS-simulator project (upstream)
docs/PORTING_NOTES.md          findings from reading the original code + every
                                design decision made while porting it
CMakeLists.txt                 top-level build (libCMAES_pthreads + linux_demo)
```

## Building and running (Linux — a Raspberry Pi, or an Ubuntu VM)

Requires a C toolchain, CMake, and pthreads. All standard on Raspberry Pi OS
or Ubuntu (`sudo apt install build-essential cmake` if not already present)
— **an Ubuntu VM works fine for this**; nothing here depends on Raspberry
Pi-specific hardware. The one thing that differs on real Pi hardware is the
real-time scheduling privilege check (see `docs/PORTING_NOTES.md` §3) — on
an unprivileged VM the demos still run correctly, just with advisory rather
than enforced agent priorities, and a one-time warning printed to stderr.

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
  round. Exercises more of the API: multi-receiver broadcast, `OneShotBehaviour`,
  suspend/resume, and `get_state` — the same primitives the Pose Estimation
  agents will use to pause/resume subsystems. Runs indefinitely; `Ctrl+C` to
  stop.

Both are direct ports of the demos the original FreeRTOS repo ships
(`sender_receiver`, `rock_paper_scissors`); porting the second one surfaced
and fixed two real bugs in the original library/demo — see
`docs/PORTING_NOTES.md` §8.

## Status

- [x] Read and understood the original FreeRTOS implementation
- [x] Native pthreads backend (`libCMAES_pthreads`) implementing the same
      `CMAES.h` API
- [ ] Verified building and running on actual target hardware (Raspberry Pi
      5 / Raspberry Pi OS)
- [ ] Pose Estimation agent system built on top

## Credits

- MAES framework design: C. Chan-Zheng, TU Delft (FreeMAES)
- Original FreeRTOS C implementation (CMAES): Andrés Conejo Boza
  ([AndresConejoBoza/CMAES](https://github.com/AndresConejoBoza/CMAES))
- pthreads/Linux backend and Pose Estimation work: this thesis project

# Porting notes: CMAES from FreeRTOS to native POSIX/pthreads

This document records what the original CMAES codebase turned out to
contain, and every design decision made while porting it to run natively on
Linux (target: Raspberry Pi 5 / Raspberry Pi OS, for the Olympus rover's
Pose Estimation system).

## 1. What the original code actually is

CMAES ([AndresConejoBoza/CMAES](https://github.com/AndresConejoBoza/CMAES))
is a C port of **FreeMAES**, a multi-agent framework for embedded real-time
systems from TU Delft, implemented directly on top of FreeRTOS primitives.

### OOP-in-C via manual vtables

Every "class" in `CMAES.h` is a struct holding both data and function
pointers:

```c
struct Agent_Msg {
    MsgObj msg;
    ...
    ERROR_CODE (*send)(Agent_Msg*, Agent_AID aid_receiver, TickType_t timeout);
    MSG_TYPE   (*receive)(Agent_Msg*, MAESTickType_t timeout);
    ...
};
```

A `Constructor*` function (e.g. `ConstructorAgent_Msg`) wires each pointer to
its real implementation. Callers do manual `this`-passing:
`msg.send(&msg, target, timeout)`. Every core type follows this pattern:
`MAESAgent`, `Agent_Msg`, `Agent_Platform`, `Agent_Organization`,
`CyclicBehaviour`, `OneShotBehaviour`, `sysVars`, `USER_DEF_COND`.

### Core pieces

- **`MAESAgent`** (`Agent.c`) — an agent's identity: `Agent_AID` (a FreeRTOS
  `xTaskHandle`), mailbox handle, name, priority, org/role state.
- **`sysVars env`** (`SysVars.c`) — a global 64-slot array mapping
  `Agent_AID → MAESAgent*`. Since a bare task handle carries no application
  data, this table stands in for task-local storage; nearly every method
  looks the target agent up here first.
- **`Agent_Msg`** (`Agent_Msg.c`) — messaging. Each agent's mailbox is a
  1-slot `xQueueCreate(1, sizeof(MsgObj))`; `send`/`receive` wrap
  `xQueueSend`/`xQueueReceive`. Messages carry a FIPA-ACL-style performative
  (`MSG_TYPE`: `REQUEST`, `INFORM`, `PROPOSE`, `CFP`, `ACCEPT_PROPOSAL`, …)
  plus a `char*` payload. `send` also enforces organization/role permissions
  before delivering.
- **`Agent_Platform`** (the AMS, `Agent_Platform.c`) — a special agent
  running at the highest FreeRTOS priority. Its task loop processes admin
  `REQUEST`s: `REGISTER`, `DEREGISTER`, `KILL`, `SUSPEND`, `RESUME`,
  `RESTART`, each gated by a **`USER_DEF_COND`** policy callback (defaults
  to always-allow). `agent_init` creates a task suspended; `register_agent`
  un-suspends it and sets its real priority.
- **`CyclicBehaviour` / `OneShotBehaviour`** — template-method wrappers:
  `execute()` runs `setup()` once, then loops `action()` →
  `failure_detection()` → `failure_identification()`/`failure_recovery()`
  until `done()` (`Cyclic` never is; `OneShot` always is, after one pass).
  You override `action`/`setup`/`done` per agent; that becomes the task body
  handed to `agent_init`.
- **`Agent_Organization`** — FIPA-style groups (`HIERARCHY`/`TEAM`), roles
  (`MODERATOR`/`PARTICIPANT`/`VISITOR`), affiliations
  (`OWNER`/`ADMIN`/`MEMBER`), with invite/add/kick/ban — mainly used to
  scope which agents may message which.

### The key discovery: a second backend already existed

`CMAES/libCMAES_CSP` is a parallel, partially-finished implementation of the
same `CMAES.h` API built on **libcsp** (Cubesat Space Protocol) instead of
raw FreeRTOS:

```c
#define MAESTaskHandle_t  csp_thread_handle_t
#define MAESQueueHandle_t csp_queue_handle_t
```

Diffing it against `libCMAES_FreeRTOS/src` (only `Agent_Msg.c`,
`Agent_Organization.c`, `Agent_Platform.c`, `sender_reciever.c` were ported;
no `rock_paper_scissors`/`telemetry`, so it's unfinished) shows that **only
three files actually call RTOS primitives**. A grep across the whole
FreeRTOS backend for `TickType_t|UBaseType_t|xTaskGetCurrentTaskHandle|
portMAX_DELAY|pdPASS` confirmed the same three files, plus two raw FreeRTOS
type leaks directly in `CMAES.h` itself. Everything else — `Agent.c`,
`SysVars.c`, `CyclicBehaviour.c`, `OneShotBehaviour.c`, `USER_DEF_COND.c` —
is pure portable C with zero RTOS dependency.

## 2. Why a native pthreads backend, not FreeRTOS-on-Linux or libcsp

Three options existed once the small RTOS-touching surface was clear:

1. **FreeRTOS's official POSIX/Linux simulator port** — swap the Windows
   MSVC-MinGW port for FreeRTOS-Kernel's `GCC/Posix` port, rebuild
   `libCMAES_FreeRTOS` almost unchanged. Least work, but the Pose
   Estimation system would still be FreeRTOS tasks simulated inside one
   Linux process, not real Linux threads/scheduling.
2. **Finish the existing libcsp backend** — libcsp already has a POSIX
   driver. Reuses the most existing code, but pulls in a satellite
   networking-protocol stack not needed for single-process inter-thread
   messaging.
3. **Native pthreads backend** (chosen) — implement the same `CMAES.h`
   surface directly on POSIX threads/mutexes/condvars. More upfront work,
   but idiomatic Linux, real POSIX scheduling, no RTOS emulation layer, and
   a clean, describable thesis contribution.

## 3. Design decisions in `libCMAES_pthreads`

### Type mapping (`CMAES.h`)

| FreeRTOS | pthreads backend |
|---|---|
| `xTaskHandle` (`MAESTaskHandle_t`) | `pthread_t` |
| `xQueueHandle` (`MAESQueueHandle_t`) | `MAES_Queue*` (new type, see below) |
| RTOS ticks (`MAESTickType_t`) | `uint32_t` **milliseconds** |
| `portMAX_DELAY` | `MAES_MAX_DELAY` (`UINT32_MAX` sentinel, "block forever") |
| `xTaskGetCurrentTaskHandle()` | `MAES_GetCurrentTaskHandle()` → `pthread_self()` |

**`Agent_AID` identity caveat**: like the original (which compares raw
`xTaskHandle` values with `==`), this backend compares raw `pthread_t`
values with `==`. That's well-defined on glibc/Linux, where `pthread_t` is
an integral type — the target platform. It is **not** portable to platforms
where `pthread_t` is an opaque struct; a future improvement would route
every comparison through `pthread_equal()`.

### `MAES_Queue` — the mailbox primitive

Every mailbox in the original is `xQueueCreate(1, sizeof(MsgObj))`:
single-slot, blocking-with-timeout on both send and receive. Implemented in
`MAES_Queue.c` as a mutex + two condvars (`not_empty`/`not_full`) + one
slot. A finite timeout becomes an absolute `struct timespec` deadline for
`pthread_cond_timedwait`; `0` is a non-blocking try, matching
`xQueueSend`/`xQueueReceive` semantics exactly.

**Cancellation safety**: the wait is wrapped in
`pthread_cleanup_push/pop(unlock_mutex_cleanup, &q->lock)` so that if the
waiting thread is cancelled (see `kill_agent` below), the mutex is
guaranteed to be released rather than left locked and wedging every other
agent that touches that mailbox.

> **Gotcha caught during implementation**: on glibc, `pthread_cleanup_push`
> /`pthread_cleanup_pop` expand to an opening/closing **brace pair**. A
> local variable (`ok`) was originally declared *between* push and pop and
> read *after* pop — out of scope, would not compile. Fixed by declaring it
> before the push. Worth remembering for any future code using these
> macros.

### Agent lifecycle on pthreads (`Agent_Platform.c`)

FreeRTOS gives free preemptive suspend/resume/delete of another task from
outside; POSIX doesn't. This required the most re-design:

- **Creation / "start gate"**: `agent_init` creates the mailbox and a
  control block (`MAES_AgentControl`: mutex + condvar + `suspended` flag +
  `AGENT_MODE`) *before* calling `pthread_create`, then `pthread_create`s
  the trampoline immediately. The trampoline's first action is to block on
  that control block while `suspended == true`. This reproduces "created
  suspended, only runs once registered" using the same primitive that later
  implements suspend/resume — and avoids any race, since the control block
  is fully initialized before the thread that reads it is created.
- **`register_agent` / `resume_agent`**: open the gate (`suspended = false`,
  broadcast the condvar).
- **`suspend_agent`**: sets `suspended = true`. This is **cooperative, not
  preemptive** — the agent only actually blocks the next time it hits a
  checkpoint. `MAES_CheckSuspend()` was added as that checkpoint and wired
  into `CyclicBehaviour`/`OneShotBehaviour`'s `execute()` loop (top of every
  iteration) and into `agent_wait()`. **This is a real behavioural
  difference from FreeRTOS**, worth remembering when writing Pose Estimation
  agents: a suspend takes effect at the next loop/wait boundary, not
  instantly. It fits the sense→estimate→publish cycle shape naturally, but
  a very long single `action()` call would delay it.
- **`kill_agent`**: `pthread_cancel()` + `pthread_join()` with deferred
  cancellation (fires only at cancellation points, e.g. the mailbox's
  `cond_wait`).
- **`get_state`**: FreeRTOS's `eTaskGetState()` has no POSIX equivalent.
  Since suspend/resume/kill are now implemented by this code rather than the
  OS scheduler, `AGENT_MODE` is simply tracked as a field on the control
  block and read back directly — coarser than FreeRTOS (no distinct
  "blocked in a syscall" state) but always available, unlike the original's
  `#ifdef tskKERNEL_VERSION_MAJOR`-guarded version.

### Admin-command permission checks — a deliberate deviation

FreeRTOS identifies "is the caller allowed to do this" by checking
`uxTaskPriorityGet(caller) == configMAX_PRIORITIES - 1` (i.e. "is the caller
running at the AMS's priority"), plus a `caller == NULL` branch for
pre-scheduler setup calls.

This backend instead compares the caller's thread id directly against
`platform->agentAMS.agent.aid` (is it the AMS?) or a new
`platform->setup_thread` field recorded when the platform is constructed
(is it the thread that called `Agent_Platform()`, e.g. `main()`?).

**Why**: priority is made best-effort on this backend (see below) — an
unprivileged process may silently fail to set real-time priorities at all,
in which case every thread would report the same priority and a
priority-equality check would authorize *any* caller, not just the AMS.
Thread-id comparison has no such failure mode.

### Priorities — best-effort `SCHED_FIFO`

`vTaskPrioritySet`/`uxTaskPriorityGet` map to
`pthread_setschedparam`/`getschedparam` with `SCHED_FIFO`. Unprivileged
processes on Raspberry Pi OS typically lack `CAP_SYS_NICE`, so a failed
`pthread_setschedparam` call is **not fatal**: it prints a one-time warning
and the agent proceeds on the default `SCHED_OTHER` policy (priorities
become advisory only). `MAES_RTPrioFor()` maps an agent's small ordinal
priority into the RT range (`min + priority`), always strictly below the
AMS; `MAES_AMSRTPrio()` puts the AMS there explicitly after the normal
registration path runs (which would otherwise map it through the same
ordinal formula as everyone else).

The AMS priority is `MAES_DEFAULT_AMS_PRIORITY` (46) unless
`MAES_SetAMSPriority()` is called before `boot()`. It used to be the
maximum (99). 46 keeps the whole platform below the kernel's threaded IRQ
handlers (`SCHED_FIFO` 50), so the USB/serial interrupt threads that deliver
sensor data are never starved by agents.

### Real-time helpers (no FreeRTOS equivalent in the original API)

- **Monotonic mailbox timeouts.** The `MAES_Queue` condition variables are
  bound to `CLOCK_MONOTONIC` and deadlines are computed on that clock. With
  `CLOCK_REALTIME`, setting the wall clock (at boot, the RPi 5 has no RTC
  battery by default, or by NTP) stretched or cut short every pending
  timeout.
- **`MAES_DelayUntil(&last_wake_ms, period_ms)`** and
  **`MAES_GetTickCount()`**: drift-free periodic loops on an absolute
  `clock_nanosleep(TIMER_ABSTIME)` deadline, the equivalent of
  `vTaskDelayUntil`. One deliberate difference: after an overrun (or a
  suspension) it re-anchors to now instead of catching up, so a late agent
  never fires a burst of back-to-back iterations. It is also a suspend
  checkpoint, like `agent_wait`.
- **`MAES_SetAffinity(aid, cpu)`**: pins an agent's thread to one CPU
  (`pthread_setaffinity_np`); `cpu < 0` leaves it alone.

### `stackSize` is not applied

`Agent_resources.stackSize` is kept in the struct for API compatibility
(existing demo code still passes it to `Iniciador`) but is not passed to
`pthread_attr_setstacksize` — the original FreeRTOS values are far too
small for a POSIX stack and could crash. Every agent gets pthreads' default
stack (commonly 8 MiB on Linux).

## 4. Bugs found in the original API (fixed here, not present as new bugs)

Two struct method signatures in the original `CMAES.h` declared a pointer
return type, but every implementation and call site actually used a bare
value:

- `Agent_AID* (*AID)(MAESAgent*)` — implementation always `return`ed a bare
  `Agent_AID`, and every call site (e.g. `receiver.AID(&receiver)` passed
  directly as an `Agent_AID` argument) treated it as one. Harmless on
  FreeRTOS only because `xTaskHandle` is itself a pointer typedef, so
  "pointer to it" and "it" are both single-word pointers. On pthreads,
  `Agent_AID` is `pthread_t` (not a pointer), so this would have been a
  genuine int-from-pointer-function bug. **Fixed**: declared and
  implemented as returning `Agent_AID`.
- `Mailbox_Handle* (* get_mailbox)(Agent_Msg*, Agent_AID)` — same pattern,
  same fix, returning `Mailbox_Handle`.

Further defects fixed in the second pass (October 2026, while preparing the
OE3 indicator test). Clang 20 and GCC 14 reject several of them outright:

- **27 method declarations returned `void*` while every implementation
  returned `void`** (constructors, `setup`/`action`/`execute`, `agent_wait`,
  `set_msg_type`, ...). Assigning a `void` function to a `void*` function
  pointer is undefined behaviour and was the source of the ~37
  `incompatible-pointer-types` warnings. The declarations now say `void`.
  User code, which was always written with `void` functions, is unaffected.
- **`get_AP_description`** was declared to return `AP_Description*` but the
  implementation returned the struct by value: undefined behaviour at every
  call. It now returns `&platform->description`.
- **`kill_agent` decremented the subscriber count twice** (once in
  `deregister_agent`, once itself), so after each kill `agent_search()` no
  longer saw the last registered agent.
- **`failure_identification` was wired to the detection function** in both
  behaviour constructors; it now points to `failure_identificationFunction`.
- **`ConstructorUSER_DEF_COND` had no prototype** (implicit declaration).
- **`agent_init`/`agent_initConParam` took the behaviour as `void*`**;
  ISO C does not allow converting a function pointer to `void*`. They now
  take `void (*)(void*)`, which is what every caller passes.
- `MAX_RECEIVERS` gained parentheses; the `USER_DEF_COND` callbacks are
  declared `(void)` instead of `()`.
- `install()` rules were placed before `add_library()`, so CMake refused to
  configure the project at all.

With these fixes the library builds as **strict C99** (`-std=c99`, required
by PE-RNF-007 for the HLC code) with no warnings, also under Clang 20.

## 5. Known limitations

- **Self-kill deadlock**: an agent calling `msg.kill()` on *itself* would
  have the AMS thread call `pthread_cancel`+`pthread_join` on... the AMS
  thread only ever kills *other* agents in the current demo flow, so this
  isn't hit in practice, but if a future agent design routes a
  self-termination request through the platform, `pthread_join` on your own
  thread is undefined behaviour (POSIX: `EDEADLK`). FreeRTOS supports
  self-delete natively (`vTaskDelete(NULL)` is idiomatic there); this
  backend does not yet have an equivalent. Would need to special-case
  "target == calling AMS-external agent that requested its own kill" and use
  `pthread_detach` + return-to-exit instead of cancel+join.
- **Benign compiler warnings in copied files**: `SysVars.c` (copied
  verbatim, untouched per design) assigns/compares `Agent_AID` fields
  against the `NULL` macro in a few places. On glibc, `NULL` is
  `((void*)0)`; assigning/comparing that against a `pthread_t` (an integer)
  produces a "makes integer from pointer" / "comparison between pointer and
  integer" warning, not an error — it evaluates correctly since `NULL`'s
  bit pattern is zero either way. Left as-is to keep that file an exact copy
  of the portable original; flagged here so the warnings aren't a surprise.
- **Pre-existing `void`/`void*` return mismatch**: every `Constructor*`
  assigns a `void`-returning implementation function to a `void*`-returning
  struct field (e.g. `agente->Iniciador = &MAESAgent1;` where `MAESAgent1`
  returns `void`). This is systemic throughout the *original* library, not
  introduced by the port — every caller discards the return value, and it
  is harmless on every real ABI, but it is technically undefined behaviour
  per the C standard. Not fixed, to avoid touching the entire codebase for
  something with no functional effect.

## 6. Build system

The original repo has no build system beyond the Visual Studio project.
Added: `CMAES/libCMAES_pthreads/CMakeLists.txt` (static library, links
`Threads::Threads`, defines `_GNU_SOURCE` for `pthread_cancel`/
`pthread_setschedparam`/`clock_gettime`), `linux_demo/CMakeLists.txt` (smoke
test executable), and a top-level `CMakeLists.txt` tying both together.

## 7. Verification status

No `gcc`/`cmake`/Linux environment was available in the sandbox this port
was written in, so every file was reviewed by hand rather than compiled —
this is how the `pthread_cleanup_push`/`pop` scoping bug above was caught,
but it means a real build on target hardware (Raspberry Pi 5 / Raspberry Pi
OS, or an Ubuntu VM for earlier iteration) is still the next step, via:

```bash
cmake -S . -B build
cmake --build build
./build/linux_demo/sender_receiver/cmaes_sender_receiver_demo
./build/linux_demo/rock_paper_scissors/cmaes_rock_paper_scissors_demo
```

`sender_receiver` expected output: interleaved `***Sending Message***` /
`***Message Received: Hello world***`, roughly once a second, indefinitely.
`rock_paper_scissors` expected output: see §8.

## 8. Findings from porting the second demo (rock_paper_scissors)

Porting `rock_paper_scissors.c` exercised parts of the API the
`sender_receiver` smoke test never touches — multi-receiver broadcast
(`send0` to more than one agent), `OneShotBehaviour`, `suspend`/`resume`
routed through the AMS, and `get_state` — and surfaced two more genuine
bugs, one in the library and one in the demo itself.

### Library bug: `clear_all_receiver` never reset the subscriber count

`clear_all_receiverFunction` (`Agent_Msg.c`) cleared every slot in the
`receivers[]` array back to `NULL`, but never reset `Message->subscribers`
back to `0`. Every `Cyclic`/`OneShot` behaviour calls `Agent_Msg()` (which
calls `clear_all_receiver`) once per `setup()`, i.e. once per round for this
game's Referee. Without the counter reset, `subscribers` only ever grows:
`add_receiver` refuses once it hits `MAX_RECEIVERS` (63), which for a
2-receivers-per-round broadcast is **~31 rounds** before the game silently
stops adding the Referee's receivers and breaks. This bug is present in the
original FreeRTOS and libcsp backends too (upstream, not touched here) —
it's a general library correctness issue, not specific to this demo, and
would have hit any long-running agent that repeatedly rebuilds a receiver
list (which is exactly the shape a Pose Estimation sensor-fusion agent
would take). **Fixed** in `libCMAES_pthreads/src/Agent_Msg.c`: `subscribers`
is now reset to `0` alongside the array.

### Demo bugs fixed while porting (not library issues)

- **`getRandom()` returned `rand() % 2`**, i.e. only 0 or 1 — `SCISSORS`
  (case 2) was unreachable in the original demo. Fixed to `rand() % 3`.
- **`choices()` compared strings with `==`** (`msg == "ROCK"`) instead of
  `strcmp`. This "worked" on the original build only if the compiler happened
  to pool/intern identical string literals into the same address across
  `playAction`'s and `choices`'s translation unit — fragile, not guaranteed
  by the C standard. Fixed to use `strcmp`.
- **`printf(some_variable)`** was used directly as a format-string argument
  in several places (agent names, message content) instead of
  `printf("%s", some_variable)`. Not attacker-reachable in this demo (the
  strings are all fixed literals from the game logic), but it's the general
  shape of a format-string vulnerability (CWE-134) and trivial to avoid, so
  fixed everywhere it appeared while porting.

### A design point worth knowing: the suspend-then-poll pattern relies on message wakeups, not on scheduling order

The Referee's `watchoverAction` calls `msg.suspend()` (fire-and-forget: it
queues a request to the AMS and does not wait for the reply) and then
immediately checks `get_state()` on both players. If the AMS hasn't
processed the request yet, the check fails and the Referee loops back to a
blocking `receive()` — this is not a busy-wait; it happens to self-resolve
because the AMS's own `CONFIRM` reply lands in the Referee's mailbox and
wakes that same `receive()` call, giving it another chance to re-check
state. This works regardless of scheduling because it's driven by mailbox
delivery (condition-variable wakeups), not by timing or thread priority —
confirmed by re-tracing the message sequence by hand, since no build
environment was available to actually observe it. It's a fragile-looking
but sound pattern already present in the original FreeRTOS demo; ported
as-is rather than redesigned, since changing it would reduce comparability
with the original. Worth knowing if you extend this demo or write a similar
suspend-then-verify sequence for a Pose Estimation agent: the retry loop
needs *some* message to eventually arrive to re-trigger the check, not a
fixed delay.

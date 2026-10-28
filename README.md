# Pose Estimation Subsystem — Olympus Rover

[![License: MIT](https://img.shields.io/badge/License-MIT-lightgrey.svg)](./LICENSE)
[![Focus: State Estimation](https://img.shields.io/badge/Focus-State%20Estimation-blue.svg)](#)
[![Status: Rover testing](https://img.shields.io/badge/Status-Rover%20testing-orange.svg)](#objectives-and-status)
[![Platform: Raspberry Pi 5 + ATmega2560](https://img.shields.io/badge/Platform-RPi5%20%2B%20ATmega2560-teal.svg)](#system-architecture)
[![Framework: CMAES](https://img.shields.io/badge/Framework-CMAES-purple.svg)](#related-work)

> **TL;DR** — Pose estimation subsystem for a 6-wheel skid-steer rover. An Extended
> Kalman Filter fuses wheel odometry with an MPU-9250 gyroscope. It runs as a five-agent
> application on the **CMAES** framework, ported to Linux (pthreads) on the Raspberry Pi 5
> high-level controller (HLC) and built into the rover's Yocto image. The low-level
> controller (LLC) frames reach it over the rover's existing serial link through a small
> multiplexer, so the deployed rover software is not modified. A GPS receiver is logged as
> a reference only: it never enters the filter, as on a planetary surface without
> satellite navigation.

Undergraduate thesis, Electronic Engineering, Instituto Tecnológico de Costa Rica.
Space Systems Laboratory (SETEC Lab) — ELANaV project.

---

## Table of Contents

- [Overview](#overview)
- [Why This Subsystem](#why-this-subsystem)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Objectives and Status](#objectives-and-status)
- [Known Limitations](#known-limitations)
- [Documentation](#documentation)
- [Related Work](#related-work)

---

## Overview

The Olympus rover had no pose estimate of its own: its telemetry carried the pose fields
as zeros. This subsystem produces a fused estimate of the planar pose (position and
heading in 2D) on the HLC, with its covariance, and publishes it to any consumer on the
rover or the ground station.

The design follows from a single constraint: **no sensor on this platform observes
absolute position.** Localization is relative to the pose at start-up (x points where
the rover's front pointed, y to its left), and the error grows with the distance
travelled. The accuracy requirement is therefore a fraction of distance (≤ 3 %), not an
absolute figure.

---

## Why This Subsystem

Three deficiencies of odometry-only localization motivate the work.

**Heading bias.** Skid steering produces lateral tyre slip by design during every
turn. Heading derived from the difference between the two sides is therefore
systematically biased, and that bias propagates into position in proportion to the
distance driven afterwards. The gyroscope provides the heading instead.

**Slip blindness.** Wheel encoders cannot distinguish a wheel that advances from one
that spins in place. The filter compares the encoder and gyroscope turn rates and
inflates its uncertainty when they disagree.

**Uncalibrated kinematics.** Encoder resolution, wheel radii and track width were not
characterized; the front right wheel even counts 2.47 times the pulses of the front left
one. The constants come from a measured model, and a straight-run calibration
(`agents/tools/odom_calib.py`) sets them on the real rover.

A further constraint shapes the integration: the subsystem must be added to a rover
already in operation **without altering the existing communication protocol or the
behaviour of the deployed software**.

---

## System Architecture

```
  ATmega2560 (LLC)                         Raspberry Pi 5 (HLC, Yocto image)
  firmware v3.0
 ┌──────────────────┐  USB serial     ┌───────────────────────────────────────────────┐
 │ 6 quadrature     │  115 200 baud   │ llcmux ──► /dev/arduino_mega ──► olympus_hlc  │
 │ encoders         │ ──────────────► │   │   (stream without RAW:)       (unchanged) │
 │ MPU-9250 (I²C)   │  RAW: + TLM     │   │                                           │
 └──────────────────┘  + MSM commands │   └──► /run/olympus/llc_raw (RAW:/TLM: + time)│
                                      │              │                                │
  GPS (GPIO UART) ──────────────────► │   olympus-pose (CMAES, 5 agents)              │
                                      │   Acquisition → Fusion → Estimation           │
                                      │   GPS ─────────────┐        │                 │
                                      │                    ▼        ▼                 │
                                      │                 Communication ──► UDP pose,   │
                                      │                                 logs, web GUI │
                                      └───────────────────────────────────────────────┘
```

- **`llcmux`** owns the Arduino's serial port, gives `olympus_hlc` the same stream as
  before (minus the 50 Hz `RAW:` lines) and copies the sensor frames, stamped with their
  arrival time, to the pose application. A udev rule renames the real port, so no file of
  the rover's software changes.
- **`olympus-pose`** has five CMAES agents: acquisition (frame parsing), data fusion
  (encoder counts to cumulative distance and heading), estimation (5-state EKF with gyro
  bias and slip detection), communication (UDP pose datagram, logs, control port) and GPS
  (NMEA, reference only). The estimator's C code is generated with MATLAB Coder from the
  reference simulation model, which runs the same chain.
- **Outputs:** a binary pose datagram on UDP, full-rate CSV logs per run, and a web GUI
  served from the rover (pose, GPS, test markers, controls, logs).

The full design, decisions and validation plan are in [`agents/DESIGN.md`](agents/DESIGN.md).

---

## Repository Structure

```
agents/                      the pose application (C99) and its tools
  src/                       agents, portable core (incl. generated EKF), Linux port layer
  tools/llcmux/              the serial-link multiplexer
  tools/                     web GUI, calibration, LLC/GPS emulators, replay converters
  config/pose.conf           configuration (model parameters exported from MATLAB)
  tests/                     unit tests, MATLAB↔C equivalence and model/app parity tests
  DESIGN.md                  design document ARQ-PE-003
CMAES/                       CMAES multi-agent library, native pthreads port
  libCMAES_pthreads/         the library
  linux_demo/                demos, including the OE3 stress test (rps_stress)
  docs/                      porting notes
  CMAES_Yocto_layer/meta-olympus-pose/   Yocto layer: library, app, llcmux, udev rule
requirements/                technical requirements (DRT-SEP-001), Channel 2 ICD (ICD-PE-002)
document/                    thesis report (PDF)
```

The MATLAB/Simulink reference model and the LLC firmware live in separate repositories
(see [Related Work](#related-work)).

---

## Getting Started

| To… | Read |
|---|---|
| build and run the unit tests on any PC, run the app, use the GUI, test on a spare Pi | [`agents/README.md`](agents/README.md) |
| add the subsystem to the rover's Yocto image | [`CMAES/CMAES_Yocto_layer/meta-olympus-pose/README.md`](CMAES/CMAES_Yocto_layer/meta-olympus-pose/README.md) |
| use the CMAES library on its own | [`CMAES/README.md`](CMAES/README.md) |
| change the estimator or its parameters | the simulation repository, then `export_pose_conf` (see `agents/README.md`) |

---

## Objectives and Status

Indicators from the thesis (§1.3). Status as of 2026-10-25; evidence in
[`agents/DESIGN.md`](agents/DESIGN.md) §16 and §19.

| Objective | Indicator | Status |
|---|---|---|
| **OE1** Electrical interface of the new sensors | I²C levels and rise time within spec (oscilloscope); 45 000 IMU reads at 50 Hz without errors | Not recorded in this repository |
| **OE2** Acquisition and telemetry | LLC sends 6 encoders + IMU at ≥ 50 Hz on the HLC clock; GPS valid fixes at ≥ 1 Hz | **LLC rate met** with the timer-loop firmware (49.9 Hz, clock scale 1.002, 2026-10-23). GPS: valid fixes at ≥ 1 Hz still to be recorded outdoors |
| **OE3** CMAES on Linux | Builds in the Yocto image; 45 000 messages at 50 Hz with register/suspend/resume, no failures, no memory growth | **Met** (2026-10-07, Raspberry Pi 5) |
| **OE4** Agent application | ≥ 4 agents; 15 min at 50 Hz with ≥ 99 % delivery and ≤ 4 consecutive losses | **Met**: five agents, 45 min at 50 Hz on the rover, 100.000 % delivery, max gap 0 (2026-10-23) |
| **OE5** Integration on Olympus | End-to-end latency < 100 ms for ≥ 15 min without failures | **Met** if `olympus_hlc` and vision were running: 45 min, HLC latency p95 0.1 ms, end-to-end bound ≈ 30 ms, no agent failure, RSS flat (2026-10-23) |
| **OE6** Accuracy vs model and physical tests | 5 m straight, 360° spin, 2 m UMBmark: ≤ 3 % position (≤ 10.8° on the spin) against markers, and the same bounds against the simulation model | **Pending: final runs.** Encoder scale calibrated 2026-10-27 (±3 %, FR = 2.47 × FL refuted); the 5 m runs verify it. Model/app equivalence verified |
| **General** | Pose at ≥ 10 Hz; UMBmark final error ≤ 3 % (24 cm) | Pending (final runs) |

---

## Known Limitations

- **Unbounded position drift.** No absolute reference is available, by design of the
  application domain.
- **LLC clock.** Firmware before the timer-loop patch counts its time stamp in software
  (+20 ms per loop while the loop takes 28–49 ms). With such firmware the estimator must
  use the HLC arrival time (`est.dt_source = 1`); with the LLC tick the simulated UMBmark
  is 45 % off instead of ≈ 1 %. The timer-loop firmware fixes it (clock scale 1.002).
- **Gyroscope bias is estimated only at standstill.** During a sustained turn the yaw
  rate and the bias are not separable, so the bias is held while moving. Thermal drift
  between standstill intervals is not modelled (the v3.0 frame now carries the IMU
  temperature, so it can be studied).
- **Whole-vehicle bogging is undetectable.** If all six wheels spin without advancing,
  encoders and gyroscope agree and the filter integrates distance that did not occur.
  The accelerometer is a possible solution (future work).
- **No slope compensation.** The gyroscope measures in the body frame; at 15° of pitch
  the missing term introduces roughly 3.5 % of systematic heading error.

---

## Documentation

| Document | Contents |
|---|---|
| [`agents/DESIGN.md`](agents/DESIGN.md) | Design document ARQ-PE-003: architecture, agents, messages, interfaces, faults, validation plan, implementation status, and the changes it implies for the thesis report |
| `requirements/DRT-SEP-001` | Functional and non-functional requirements, design constraints, latency and error budgets, traceability matrix |
| `requirements/ICD-PE-002` | Channel 2 binary link (dedicated UART). Evaluated but **not used**: the existing serial link is shared through `llcmux` instead (DESIGN.md D1) |
| `document/` | Thesis report: theoretical framework, design, implementation, verification and results |

---

## Related Work

- **[Reference simulation model](https://github.com/JorgeSchofield/olympus-pose-estimation-simulation)**
  — MATLAB/Simulink model of the plant, the LLC, the serial link and the HLC agents. The
  application's estimator is generated from it, and both run the same estimation chain.
- **[Olympus rover](https://github.com/Alonso11/Olympus-Project-TFG-TEC)** — the platform,
  developed by Fabricio Gómez Quesada and taken to TRL-4 in April 2026. Its documentation
  identifies odometry–IMU fusion as future work and transmits the pose fields as zero;
  this project closes that gap. HLC image:
  [olympus-hlc-rpi5](https://github.com/Alonso11/olympus-hlc-rpi5); LLC firmware:
  [rover-low-level-controller](https://github.com/Alonso11/rover-low-level-controller).
- **[CMAES](https://github.com/AndresConejoBoza/CMAES)** — the multi-agent framework,
  from the laboratory's research line on embedded multi-agent systems: MAES (Chan-Zheng)
  → FreeMAES (Rojas Marín) → CMAES (Conejo Boza).

## Author & Institution

**Jorge Schofield Carvajal**
Instituto Tecnológico de Costa Rica (TEC)

School of Electronics Engineering

SETEC Lab — Space Systems Laboratory

Advisor: Prof. Johan Carvajal Godínez.

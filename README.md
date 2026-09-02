# Pose Estimation Subsystem — Olympus Rover

[![License: MIT](https://img.shields.io/badge/License-MIT-lightgrey.svg)](./LICENSE)
[![Focus: State Estimation](https://img.shields.io/badge/Focus-State%20Estimation-blue.svg)](#)
[![Status: In Development](https://img.shields.io/badge/Status-In%20Development-orange.svg)](#status)
[![Platform: Raspberry Pi 5 + ATmega2560](https://img.shields.io/badge/Platform-RPi5%20%2B%20ATmega2560-teal.svg)](#system-architecture)
[![Framework: CMAES](https://img.shields.io/badge/Framework-CMAES-purple.svg)](#related-work)

> **TL;DR** — Pose estimation subsystem for a 6-wheel skid-steer rover. An Extended
> Kalman Filter fuses wheel odometry with an MPU-9250 gyroscope, running as a
> four-agent application on the **CMAES** framework ported to Linux on the
> Raspberry Pi 5 (HLC). A GPS receiver works as a filter measurement — it is
> ground truth for offline validation only, mirroring the absence of satellite
> navigation on a planetary surface.

Undergraduate thesis, Electronic Engineering, Instituto Tecnológico de Costa Rica.
Space Systems Laboratory (SETEC Lab) — ELANaV project.

---

## Table of Contents

- [Overview](#overview)
- [Why This Subsystem](#why-this-subsystem)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Objectives and Indicators](#objectives-and-indicators)
- [Status](#status)
- [Known Limitations](#known-limitations)
- [Documentation](#documentation)
- [Related Work](#related-work)

---

## Overview

This repository contains the documentation for the Olympus rover's own pose estimation subsystem. 
This subsystem comes to fill this missing capability with a fused estimate of the planar pose (position and
heading in 2D) produced at the high-level controller and published to the navigation stack.

The design follows from a single constraint: **no sensor on this platform observes
absolute position.** Localization is therefore relative to the pose at power-up, and
the error grows monotonically with distance travelled. The accuracy requirement is
consequently expressed as a fraction of distance rather than as an absolute figure,
and the estimate is reported together with its covariance.

---

## Why This Subsystem

Three deficiencies of odometry-only localization motivate the work.

**Heading bias.** Skid steering produces lateral tyre slip by design during every
turn. Heading derived from the difference between the two sides is therefore
systematically biased, and that bias propagates into position in proportion to the
distance driven afterwards.

**Slip blindness.** Wheel encoders cannot distinguish a wheel that advances from one
that spins in place. On unstructured terrain the platform reports displacement that
never happened.

**Uncalibrated kinematics.** Effective encoder resolution, wheel radii and equivalent
track width are not characterized, so the current estimate rests on provisional
values that have not been verified experimentally.

A further constraint shapes the integration: the subsystem must be added to a rover
already in operation **without altering the existing communication protocol or the
behaviour of the deployed software**.

---

## System Architecture

```
  ATmega2560 (LLC)                          Raspberry Pi 5 (HLC)
  20 ms control loop                        CMAES agents on POSIX threads
 ┌──────────────────┐                      ┌──────────────────────────────┐
 │ 6 quadrature     │   Channel 2          │  Acquisition → Fusion        │
 │ encoders         │   50 Hz binary       │       ↓           ↓          │
 │ MPU-9250 (I²C)   │ ───────────────────▶ │  Communication ← Estimation  │
 └──────────────────┘   dedicated UART     └──────────────────────────────┘
          │                                              │
          │  Channel 1 — existing MSM protocol           ▼
          └────────────────────────────────────▶  pose @ ≥ 10 Hz
                                                          + covariance

  GPS ──▶ HLC, logged only. Never enters the filter.
```

Channel 2 is a new unidirectional binary link on the free USART of the ATmega2560.
It carries a fixed-size 50-byte frame at 50 Hz and leaves the existing MSM protocol
untouched, which is how the subsystem satisfies the non-interference requirement by
construction.

---

## Repository Structure

```
document/       Thesis manuscript (LaTeX)
requirements/   Technical requirements (DRT-SEP-001) and Channel 2 ICD
firmware/       Low-level controller extension (Rust, no_std)
agents/         Multi-agent application and CMAES portability layer
```

The MATLAB/Simulink reference model lives in a separate repository — see
[Related Work](#related-work).

---

## Objectives and Indicators

| Objective | Indicator | Status |
|---|---|---|
| Integrate the IMU and GPS into the platform | Encoders and IMU at ≥ 50 Hz; GPS at ≥ 1 Hz | Pending |
| Design the multi-agent application | ≥ 4 agents; ≥ 99 % message delivery | In design |
| Integrate the subsystem on the rover | ≥ 15 min continuous; < 100 ms end-to-end latency | Pending |
| Evaluate performance | Position error ≤ 3 % of distance travelled | Pending |

---

## Status

Week 4 of 16. The requirements document, the simulation reference model and the first
two chapters of the manuscript are complete. Design of the multi-agent application is
under way.

Two open items should be read before any result in this repository:

- **Kinematic parameters are not characterized.** Ticks per revolution, wheel radii and
  effective track width are provisional, and the figures available from the previous
  test campaign are mutually inconsistent by two orders of magnitude. Simulation results
  demonstrate that the algorithm works; they do **not** demonstrate that the platform
  meets the accuracy target.
- **Measurements are blocked** until the section of the rover damaged in an accident is
  repaired.

---

## Known Limitations

- **Unbounded position drift.** No absolute reference is available, by design of the
  application domain.
- **Gyroscope bias is estimated only at standstill.** During a sustained turn the yaw
  rate and the bias are not separable, so the bias is held while moving.
 **NOTE: Thermal drift between standstill intervals is not tracked. This needs to be studied further.**
- **Whole-vehicle bogging is undetectable.** If all six wheels spin without advancing,
  encoders and gyroscope agree and the filter integrates distance that did not occur.
  **NOTE: Using the IMU's accelerometer is a viable solution, which is work in progress.**
- **No slope compensation.** The gyroscope measures in the body frame; at 15° of pitch
  the missing term introduces roughly 3.5% of systematic heading error.

---

## Documentation

| Document | Contents |
|---|---|
| `requirements/DRT-SEP-001` | Functional and non-functional requirements, design constraints, latency and error budgets, traceability matrix |
| `requirements/ICD-PE-002` | Channel 2 frame format, link budget and semantics |
| `document/` | Thesis manuscript: theoretical framework, design, implementation, verification and results |

---

## Related Work

- **[Reference simulation model](https://github.com/JorgeSchofield/olympus-pose-estimation-simulation)** — MATLAB/Simulink implementation of the filter, the
  plant and the sensor error models, used to size the error budget and to validate the
  embedded implementation.
- **[Olympus rover](https://github.com/Alonso11/Olympus-Project-TFG-TEC)** — the platform, developed by Fabricio Gómez Quesada and taken to
  TRL-4 in April 2026. Its documentation identifies odometry–IMU fusion as future work
  and transmits the pose fields as zero; this project closes that gap.
- **[CMAES](https://github.com/AndresConejoBoza/CMAES)** — the multi-agent framework, from the laboratory's research line on embedded
  multi-agent systems: MAES (Chan-Zheng) → FreeMAES (Rojas Marín) → CMAES (Conejo Boza).


## Author & Institution

**Jorge Schofield Carvajal**
Instituto Tecnológico de Costa Rica (TEC)

School of Electronics Engineering

SETEC Lab — Space Systems Laboratory

Advisor: Prof. Johan Carvajal Godínez.

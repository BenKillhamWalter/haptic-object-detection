# MARS Hand — Magnetic AI-enabled Recognition System

5-class tactile object recogniser on an underactuated prosthetic hand,
extended with a 4-mode EMG-driven multi-grasp controller, force fail-safe,
and browser UI.

> **The README content that used to live here is obsolete.** It described
> the pre-split single-board architecture (6 sensors, TCA9548A mux, on-chip
> `micromlgen` inference). The system has been completely re-architected.
>
> **Start here instead:**
>
> 1. **`CLAUDE.md`** — current architecture, folder layout, conventions,
>    Stage 2 status, recent landmarks. **Read this first.**
> 2. **`STAGE2_BRINGUP.md`** — long-form reference for stage 2: per-mode
>    behaviour, every serial command, every event, tuning procedures (esp.
>    force fail-safe finger-pushback test), calibration recommendations.
> 3. **`CONTEXT.md`** — pre-split single-board history. Mostly stale but
>    has useful hardware and feature-extraction notes if you need them.

## Quick start (the demo-day order)

```
# Bench setup (one-time per session)
python python/run.py                ← calibrate + a few warm-up grasps
python python/run.py train          ← only if you've collected new data

# Live UI (demo)
python python/server.py             ← opens http://localhost:8080
```

Or headless:
```
python python/daemon.py             ← stdout prints RESULT: <class>
```

See `CLAUDE.md` for everything else.

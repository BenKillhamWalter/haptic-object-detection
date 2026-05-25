# CLAUDE.md — Project Conventions and Current State

**Read this first.** This file is the canonical current-state reference.
- For deep stage-2 details (tuning procedures, every command, event,
  per-mode behaviour, calibration recommendation) see
  **`STAGE2_BRINGUP.md`** — it's the long-form companion.
- `CONTEXT.md` is the pre-split (single-board) history, mostly stale.
- `README.md` is stale (predates the split); ignore it.

Last updated: **2026-05-20** — Stage 2 is fully implemented and tuned.
Recent landmarks (most recent first):
- Drive-mode enforcement at boot + `I<id>` toggle command for M2/M3
  direction (was: M2 randomly flipped direction after power cycle).
- `R<id>` motor-reboot command for recovering an overload-shutdown motor
  without a power cycle.
- M2 (soft-finger) safety switched from `PRESENT_LOAD` (always 0 on XC330
  X-series) to `PRESENT_CURRENT`. Soft fingers now wrap and stop.
- Screwing mode no longer closes M2 — tripod-style grip with wrist
  rotation. Wrist range tightened to [30°, 270°]; home = 150°.
- M1 (grip) force fail-safe switched from `PRESENT_LOAD` to
  `PRESENT_CURRENT` (same XM430 X-series gotcha). Default 200
  (~540 mA) prevents the gear-stripping that was happening on stiff
  PLA cubes.
- EMG semantics changed from level-based (flex closes / release opens)
  to **push-button toggle** (1st flex closes, 2nd flex opens). Hysteresis
  raised from 8 to 15 for noise robustness.
- UI server (`python/server.py`) + browser UI (`python/ui/index.html`)
  added — aiohttp + WebSocket, single port localhost:8080.
- Daemon (`python/daemon.py`) added for headless OBJ_RECOG operation.

---

## Project

A 5-class tactile object recogniser on an underactuated prosthetic hand:
`cube_soft`, `cube_stiff`, `cylinder_soft`, `cylinder_stiff`, `no_object`.
A 6th label `unknown_object` is emitted at inference time when a grasp
fails (no t1 or no t2 detected) — it is not a trainable class.

---

## Current architecture: split, two-board

```
  Seeed XIAO ESP32S3                       OpenRB-150
  ├── 4× MLX90393 (one I2C bus,            ├── XM430-W350-R via RS485->TTL bridge
  │   no multiplexer)                      │   (Protocol 2.0, ID=1, 57600 baud)
  ├── pure sensor pump                     ├── XC330 is on the bus but NOT
  │   (raw x/y/z, 50 Hz)                   │   used for object recognition
  └── I2C recovery sequence at boot        ├── grasp protocol on-chip
                                           ├── encoder-stall t1 detection
                                           └── event marker emitter
              │                                       │
              └──────  USB  ──  Laptop  ──  USB  ─────┘
                                  │
                          Python orchestrator
                            (see python/run.py)
```

Both boards are services. The laptop is the conductor — it auto-detects
COM ports, sends commands, merges streams, subtracts motor-field, runs
magnetic-t1 detection, extracts the 57-feature vector, trains sklearn,
and predicts. **No on-chip inference.** Model lives in Python (`model.pkl`).

The two boards are connected only via the laptop USB chassis ground.
**No direct signal wire between them.**

---

## Folder layout (complete)

```
prosthetic_hand_fixed/
├── CLAUDE.md                          ← this file (current state)
├── STAGE2_BRINGUP.md                  ← stage-2 long-form (modes, EMG, safety,
│                                        commands, events, tuning procedures)
├── CONTEXT.md                         ← pre-split history (mostly stale)
├── README.md                          ← stale; ignore
│
├── xiao_esp32s3/
│   ├── sensor_test/sensor_test.ino    ← bench debug
│   └── sensor_stream/sensor_stream.ino← production sensor firmware (UNCHANGED)
│
├── openrb/
│   └── motor_control/motor_control.ino← stage-2 motor + EMG + button + LED +
│                                        4-mode state machine + force safety
│
└── python/
    ├── hand/                          ← importable library (UNCHANGED in stage 2)
    │   ├── constants.py protocol.py controller.py
    │   ├── motor_field.py grasp.py features.py
    │
    ├── run.py setup_check.py calibrate.py collect.py train.py inference.py
    │                                  ← stage-1 data/training/inference scripts
    │
    ├── daemon.py                      ← stage-2 headless OBJ_RECOG driver
    ├── server.py                      ← stage-2 aiohttp UI server (localhost:8080)
    ├── ui/index.html                  ← stage-2 browser UI (single file)
    │
    ├── requirements.txt               ← + aiohttp for server.py
    └── data/                          ← session CSVs, calibration.npz,
                                        model.pkl, confusion matrix
```

---

## Hardware quick reference

| Item | Value | Notes |
|---|---|---|
| Sensor MCU | Seeed XIAO ESP32S3 | 3.3V logic, native USB, I2C on D4/D5 |
| Motor MCU | OpenRB-150 | drives XM430-W350-R via external RS485↔TTL bridge |
| Sensors | 4× MLX90393 | addresses 0x14, 0x15, 0x16, 0x17 — direct I2C, no mux |
| Pull-ups | 4.9 kΩ external | SDA→3V3 and SCL→3V3 (already installed) |
| Motor | XM430-W350-R | ID=1, Protocol 2.0, 57600 baud |
| Motor range | 66° (open) → 110° (close) | 45 encoder-degree bins for motor-field profile |
| Sample rate | 50 Hz on both boards | 20 ms sample interval |

### Sensor addressing

| Sensor | Addr | A1, A0 | Role |
|---|---|---|---|
| s0 | 0x14 | 0, 0 | Index ID1 (base) |
| s1 | 0x15 | 0, 1 | Index ID2 (tip) |
| s2 | 0x16 | 1, 0 | Thumb ID3 (base) |
| s3 | 0x17 | 1, 1 | Thumb ID4 (tip, most sensitive) |

### Amp settings — must stay byte-identical across all sketches

```cpp
mlx.setGainSel(0);            // highest gain (5x XY / 8x Z)
mlx.setResolution(2, 2, 2);   // coarse LSB, no saturation at high field
mlx.setOverSampling(2);       // 4x averaging
mlx.setDigitalFiltering(5);   // heavier on-chip IIR
```

---

## How the pipeline works end-to-end

### 1. Serial protocol

**XIAO ESP32S3** — `sensor_stream.ino`
```
commands:  p / e / s / x / r
samples:   S,<millis>,<s0x>,<s0y>,<s0z>,<s1x>,<s1y>,<s1z>,<s2x>,<s2y>,<s2z>,<s3x>,<s3y>,<s3z>
events:    # ENUM_READY | # ENUM_FAIL ... | # ENUM <n> 0x14 0x15 ... | # STREAM_ON | # STREAM_OFF | # pong | # ERROR ...
```

**OpenRB-150** — `motor_control.ino`
```
commands:  p / e / s / x / o / c / h / g / r / a
samples:   M,<millis>,<enc_deg>,<load>,<current>
events:    # GRASP_START / # T1_STALL / # STABLE_DETECTED / # T2_CAPTURE
           # GRASP_COMPLETE / # GRASP_TIMEOUT
           # CALIB_START / # CALIB_COMPLETE / # ABORTED / # MOTOR ... / # pong
```

### 2. Two t1 detection paths (kept from original design)

- **Encoder-stall t1**: detected on OpenRB (it has native low-latency encoder
  access). Emits `# T1_STALL <ms> <enc>`.
- **Magnetic t1**: detected in Python from motor-field-subtracted sensor
  samples, against per-channel auto-derived thresholds.
- Whichever fires first wins; `GraspRecord.t1_source` records which.

### 3. t2 capture

OpenRB runs the stable-detection logic (encoder still for
`ENCODER_STILL_COUNT` samples), waits `HOLD_DURATION_MS`, then emits
`# T2_CAPTURE <ms> <enc> <load> <current>`. Python uses these values
directly for features 52..54.

The canonical `time_to_stall_ms` is set **at stable-detection time**, not
at t1 firing. This was the critical bug in the old single-board firmware
(CONTEXT.md section 13) — both `grasp.py` and `motor_control.ino` enforce
the correct definition.

### 4. Motor-field profile + auto thresholds

`calibrate.py` (or the calibration step inside `run.py`) commands a slow
sweep, captures both streams, and builds a `(45, 4, 3)` profile array
indexed by encoder-degree bin. Right after building the profile, it
derives per-channel contact thresholds from residual noise:

```
threshold[s][a] = max(THRESHOLD_FLOOR, p99(|residual|) * THRESHOLD_SAFETY_FACTOR)
```

Both profile and thresholds are saved together in
`data/calibration.npz`.

### 5. Subtraction at runtime

For each new sensor sample, Python:
1. Finds the nearest motor sample by laptop wall-clock receive time
2. Looks up `profile.profile[bin_for(enc_deg)]` → (4,3) baseline
3. Subtracts: `delta = raw - profile_baseline`

The motor-field profile *is* the baseline. There is no separate static
baseline subtraction in the production path — the profile already
captures the empty-hand field at every encoder position, including the
open position. The bench-debug sketch (`sensor_test.ino`) does use a
separate static baseline because it doesn't have motor data.

### 6. 57-feature extraction

`hand/features.py:extract_features(GraspRecord)` is the **single source
of truth** used by both `collect.py` and `inference.py`. No feature drift
possible between train and infer. Layout matches CONTEXT.md section 5.

### 7. Training

`train.py` reads every `data/grasps_session*.csv`, drops failed grasps,
runs `GroupKFold` CV with `session_id` as the group (avoids session
leakage), prints classification report, saves a row-normalised confusion
matrix PNG and `data/model.pkl`.

### 8. Inference

`inference.py` (or `run.py infer`) loads `model.pkl` + `calibration.npz`,
runs grasps on user command, extracts features, predicts. If the grasp
fails (no t1 / no t2 / extract returns None), prints `unknown_object`
with the failure reason instead of forcing a low-quality prediction.

---

## User workflows

### One-time setup
```
1. Flash sensor_stream.ino to the XIAO ESP32S3
2. Flash motor_control.ino to the OpenRB-150
3. pip install -r python/requirements.txt
```

### Standard data collection day
```
python python/run.py
```
This does setup_check → calibrate → collect in one go. Boards are
auto-detected by USB VID/PID + handshake.

### Multi-session protocol
After ~25 grasps, type `q`, power-cycle the boards, then run
`python run.py` again. The collector auto-increments `session_id` so
the GroupKFold in `train.py` can keep sessions separate.

### Training
```
python python/run.py train
```
(or `python python/train.py` for the same thing without the orchestrator.)

### Live inference
```
python python/run.py infer
```

---

## Recovery sequence for XIAO sensor enumeration

`sensor_stream.ino` automatically retries I2C init up to 3 times at boot.
Between attempts it manually toggles SCL 16 times to free any stuck slave
holding SDA low. If all 3 attempts fail it emits `# ENUM_FAIL` listing the
missing addresses and refuses to enter the streaming state. The XIAO
remains responsive to `r` for a manual recovery retry.

Python's `setup_check.py` and `run.py` both refuse to proceed to
calibration / collection if fewer than 4 sensors are present after one
recovery attempt — they print the missing addresses and exit with a
non-zero code. The user must replace / repair the strip before continuing.

---

## Conventions going forward

1. **No on-chip inference.** Model in Python only.
2. **No TCA9548A on the sensor side.** 4 sensors, unique addresses, one bus.
3. **No direct wire between XIAO and OpenRB.** USB-only via the laptop.
4. **Old CSVs are not compatible.** The new CSV schema has different columns.
5. **Amp settings frozen** (see above). Any change requires retraining.
6. **GND-first when mating connectors.** Power off the board before
   connecting or disconnecting the strip.
7. **Two t1 paths**: encoder-stall (OpenRB, on-chip) and magnetic
   (Python, post-subtraction). First-wins. Both report which path fired.
   **Neither path stops the motor** — they only label `t1_source` for
   the CSV. The only mechanism that stops the motor on contact is the
   force fail-safe (point 9).
8. **time_to_stall_ms is set at stable-detection** in both `grasp.py` and
   `motor_control.ino`. Don't reintroduce the old bug.
9. **Grip motor (XM430, ID 1) has a hardware-current fail-safe**
   (added 2026-05-20 in `motor_control.ino`). Every grasp /
   calibration loop iteration reads `PRESENT_CURRENT`; when
   `|current| > CURRENT_LIMIT_GRASP` (default 200, units of ~2.69 mA
   ≈ 540 mA) for `CURRENT_CONFIRM` consecutive samples, the firmware
   freezes the motor at its current encoder position via
   `gotoDeg(readEncoderDeg())`. This was added because stiff PLA
   cubes were stripping the gear train — the encoder-stall and
   magnetic-t1 detectors fire but do not change the motor goal, so
   the position PID kept pushing at full current until the t2 capture
   completed ~1 s later. **Use `PRESENT_CURRENT`, not `PRESENT_LOAD`**
   — the latter is not implemented on XM430 X-series and always reads
   0. Encoder-stall is intentionally left observation-only so
   soft/medium objects can still compress during the 800 ms HOLD
   (preserves training-data distribution); only objects approaching
   gear-damage current levels trigger the freeze. See
   `STAGE2_BRINGUP.md` section 8 for the tuning procedure — **always
   run the 6-step tuning, especially the finger-pushback test in
   step 3, before grasping stiff objects**.

---

## Open items (as of 2026-05-20)

- [ ] **Stage-2-aware calibration in `server.py`.** Currently typing
      `r` in the UI's serial input runs the firmware sweep but does
      not rebuild the Python-side motor-field profile (only
      `python python/run.py` does that). Adding the calibration-build
      path to `server.py` would let users refresh calibration without
      dropping out to the CLI.
- [ ] **More training data.** ~30 grasps was enough to get good
      accuracy but soft-class confidence sits around 40 %. Collect
      2–3 more sessions of 25 grasps each (`python python/run.py`,
      power-cycle between sessions for proper GroupKFold) and
      retrain. Expected to push soft confidence to 65–80 %.
- [ ] **Phase A of Q3 features** (deferred from previous session):
      add `peak_current`, `enc_displacement = enc_t2 - enc_t1`, and
      inter-finger asymmetry features to `hand/features.py`. All
      computable from existing CSVs — retraining on current data
      should jump stiff/soft accuracy to ~98–100 % and cube/cylinder-
      soft to 80–90 %.
- [ ] If timestamp alignment proves too coarse in practice, add a
      single sync-pulse wire (OpenRB GPIO → XIAO GPIO) with 100Ω
      series resistor. Not needed at 50 Hz so far.

---

## Stage 2 — multi-mode hand with EMG (FULLY IMPLEMENTED)

> Stage 1 = the object-recognition pipeline described above.
> Stage 2 = adds two more motors, four grasp modes, EMG actuation,
> button mode-cycle, LED indicator, force fail-safe, and a browser
> UI. **All implemented in 2026-05.** Deep detail is in
> `STAGE2_BRINGUP.md`; this section is the summary.

### Hardware (wired and working)

| Item | Detail |
|---|---|
| Motor ID 1 | XM430-W350-R — grip (66°–110° full, 66°–97° in screwing) |
| Motor ID 2 | XC330-T288 — soft fingers, cable-actuated (30°–260°, default rotation polarity enforced at boot) |
| Motor ID 3 | XC330-T288 — wrist; range **[30°, 270°]**, home **150°**, screwing-cycle 270°→30° via 180° |
| EMG / pot | A1 / A0 (analog) |
| Button | D4 (`INPUT_PULLUP`, short=cycle, long=home) |
| LED | D5 (active-high, 680 Ω inline) |
| Power | 12 V wall adapter |

### Four grasp modes (cycled by short button-press)

EMG is **push-button toggle** — rising edge only actuates motors;
release is observed but does not actuate. See `STAGE2_BRINGUP.md`
section 3 for the full per-mode table.

| Mode | LED | Behaviour summary |
|---|---|---|
| POWER | solid | 1st flex closes M1+M2; 2nd flex opens both. |
| TRIPOD | 1 Hz blink | 1st flex closes M1 only (M2 stays open); 2nd flex opens. |
| OBJ_RECOG | 4 Hz blink | Firmware emits `# EMG_FLEX`; daemon/server sends `g`, runs classifier, hand auto-opens. |
| SCREWING | double-blink | Tripod-style + wrist rotation. 3 user flexes + 1 automatic: close M1 to 97° + wrist→270° → wrist 270°→30° → open M1 → (3 s) → wrist 30°→270°. M2 stays open throughout. |

### Force fail-safe (the only thing that actually stops a grasp)

Both `cmdGrasp`/`cmdCalib` (M1) and `m2Tick` (M2) monitor
`PRESENT_CURRENT` every loop iteration. When `|current|` exceeds the
mode's limit for N consecutive samples, the motor is frozen at its
present position via `gotoDeg(readEncoderDeg())`. Encoder-stall t1
and magnetic-t1 detection still run but are **observation-only** —
they only label `t1_source` in the CSV.

| Motor | Limit constant | Default | Unit |
|---|---|---|---|
| M1 (XM430) | `CURRENT_LIMIT_GRASP` | 200 | ~2.69 mA/LSB → ≈ 540 mA |
| M2 (XC330) | `M2_CURRENT_LIMIT` | 300 | 1 mA/LSB → 300 mA |

> **Gotcha:** `PRESENT_LOAD` is not implemented on X-series motors
> and always returns 0. Always use `PRESENT_CURRENT`. Both M1 and M2
> safety paths previously read load and silently never tripped; both
> are fixed.

### EMG signal-processing path

Matches `latestWorkingCodeEMG.ino` verbatim (read raw → diff → abs →
80-sample moving average → compare to `analogRead(potPin) / 4`).
Only deviation: hysteresis = 15 counts on the falling-edge check,
needed for binary edge detection (the reference does level-based
proportional servo control, so it doesn't need hysteresis). See
`STAGE2_BRINGUP.md` section 3 for tuning and the reference-code path.

### Python entry points (stage 2)

| Script | Purpose |
|---|---|
| `python python/server.py` | Browser UI on localhost:8080. Mirrors firmware mode, drives classifier on EMG_FLEX in OBJ_RECOG, broadcasts EMG waveform / events / results to the UI. |
| `python python/daemon.py` | Headless equivalent of server.py. No UI; prints `RESULT: <class>` to stdout. |
| `python python/run.py` | Stage-1 orchestrator (calibrate + collect). **Run this before each demo session to refresh `data/calibration.npz`** — stale calibration is the most likely cause of low OBJ_RECOG confidence in the UI. |
| `python python/run.py train` | Retrain on all `data/grasps_session*.csv`, writes `data/model.pkl`. |

### Recovery commands (no power cycle needed)

| Command | Effect |
|---|---|
| `R<id>` | Reboot a stuck motor (HARDWARE_ERROR_STATUS recovery). E.g. `R3` after wrist gear catches. ~2.5 s. |
| `I<id>` | Toggle `DRIVE_MODE` bit 0 for M2/M3 — fixes "motor rotates opposite direction" without recompiling. To make permanent, also update `M<id>_DRIVE_MODE_DEFAULT` constant. |
| `?` | Print `# DIAG current_limit=... peak_load_last=... peak_current_last=...` for force-safety tuning. |
| `h` | Home all motors (long-press button = same). |
| `a` | Torque off all motors. |

Full command + event list in `STAGE2_BRINGUP.md` sections 4 + 5.

---

## What a new session needs to know

If you're picking this up fresh:

1. **Stage 2 firmware is in `openrb/motor_control/motor_control.ino`** —
   single file, fully self-documenting header comment. The current
   semantics (toggle, screwing-without-M2, current-based safety,
   `R<id>`/`I<id>` commands) are the latest after 2026-05-20.
2. **Don't touch `python/hand/`** — the classifier pipeline is stable.
   `protocol.py` is generic enough that new firmware events parse as
   `Event(name, args)` without modification.
3. **`server.py` is the demo-day entry point.** Reflash, calibrate
   via `run.py`, then `server.py`, then open localhost:8080.
4. **Before grasping stiff objects, run the finger-pushback test**
   (`STAGE2_BRINGUP.md` section 8 step 3). The force fail-safe must
   trip on finger pressure or the gears go.
5. **If a motor stops responding**, try `R<id>` before unplugging
   12 V. If M2 rotates the wrong way, try `I2` and see if that fixes
   it; persist by updating `M2_DRIVE_MODE_DEFAULT`.
6. **If OBJ_RECOG confidence is lower than training-time**, run
   `python python/run.py` to refresh calibration before blaming
   anything else.

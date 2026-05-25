# Prosthetic Hand Object Recognition — Project Context

> ⚠️ **PARTIALLY SUPERSEDED — read `CLAUDE.md` first.**
>
> As of 2026-05-14 the system was split across two microcontrollers
> (Arduino Nano 33 BLE for sensors, OpenRB-150 for motors) with the model
> moved to Python (`sklearn` `.pkl`). Anything in this document that
> describes single-board firmware, on-chip inference (`micromlgen` /
> `model.h`), TCA9548A multiplexing, or the old CSV format is **stale**.
>
> Still valid in this document: hardware (sensors, motor, silicone),
> sensor addressing, motor geometry, the 57-feature definitions, RF
> hyperparameters, multi-session protocol, diagnostic history, do-not-touch
> reasoning. The amp settings (section 3) have been **changed** for the
> new system — see `CLAUDE.md`.

**Purpose of this file**: lossless handoff for a fresh AI session (or new collaborator).
Reading this in your first turn should be enough to continue the project without
asking what we already decided. Last updated: 2026-05-10.

If you're a new AI session, do this:

1. Read this file fully **before** modifying anything.
2. Treat the **"Do-not-touch" list** at the bottom as load-bearing.
3. When asked to change a constant, check if the rationale below explains the
   current value before overriding it.

---

## 1. The 30-second summary

A 5-class tactile object recogniser on an underactuated prosthetic hand.

- **Hardware**: OpenRB-150 (SAMD21) + Dynamixel XM430 motor + 4 MLX90393
  magnetic tactile sensors (Index ID1, Index ID2, Thumb ID3, Thumb ID4),
  covered by a stiff silicone glove.
- **Workflow**: Hand closes once (open=60° → close=110°), captures sensor
  time series + motor state, extracts a 57-feature vector, classifies with a
  Random Forest exported to C++ via `micromlgen`.
- **Classes (alphabetical, required by sklearn LabelEncoder)**:
  `cube_soft, cube_stiff, cylinder_soft, cylinder_stiff, no_object`.
  Plus runtime fallback `unknown_object` for hard failures (no t1 or no t2).
- **Key idea**: the rotor magnet of the Dynamixel motor swamps weak tactile
  signal. To recover the tactile signal we subtract a per-encoder-bin baseline
  captured by sweeping the motor open→close with no object (called the
  "motor-field profile", calibrated by serial command `r`).

---

## 2. Hardware

| Item | Detail |
|---|---|
| MCU | OpenRB-150 (SAMD21, 32 KB SRAM, 256 KB Flash) |
| Motor | Dynamixel XM430-W350-R, ID=1, baud=57600, Protocol 2.0 |
| Multiplexer | TCA9548A at I2C 0x70 |
| Sensors | 4× MLX90393, see table below |
| I2C library | arduino-MLX90393 by Theodore Yapo |
| Motor library | Dynamixel2Arduino by ROBOTIS |

### Sensor wiring (do not change without re-checking the firmware)

| Sensor index | Physical role | TCA channel | begin(A1, A0) | I2C addr |
|---|---|---|---|---|
| 0 | Index ID1 (base) | TCA_CH_INDEX (0) | `begin(0, 0)` | 0x14 |
| 1 | Index ID2 (tip)  | TCA_CH_INDEX (0) | `begin(0, 1)` | 0x15 |
| 2 | Thumb ID3 (base) | TCA_CH_THUMB (1) | `begin(1, 0)` | 0x16 |
| 3 | **Thumb ID4 (TIP)** ← most important first-contact sensor | TCA_CH_THUMB (1) | `begin(1, 1)` | 0x17 |

**Middle finger has been removed.** TCA channel 2 had irrecoverable hardware
failure (sensors at that strip stopped responding to I2C). Firmware was
restructured to 4 sensors; do not try to re-enable middle without first
fixing channel 2 hardware. See section 13 for the symptom diagnostics.

### Motor geometry

| Parameter | Value |
|---|---|
| `OPEN_POS_DEG` | 60.0 |
| `CLOSE_POS_DEG` | **110.0** — intentionally limited so fingertips don't collide at end-stop |
| `GRASP_SPEED_RPM` | 10.0 |
| `CALIB_SPEED_RPM` | 3.0 — slow sweep for motor-field profile calibration |

The 110° close was reduced from 115° then 120° because:
- 120°: fingertips collided at end-stop, polluted no_object baseline.
- 115°: still some collision under load.
- 110°: clean end-stop in mid-air. No mechanical contact between fingers
  during empty grasps. Cleaner residual signal for the motor-field profile.

### Silicone

The silicone covering is **stiff** (decision made to not remold). Soft
objects deflect the sensor magnets very little through this silicone — that
is the central physical constraint of the whole system, and the reason
encoder-stall fallback exists.

---

## 3. Firmware structure

Two `.ino` files that MUST stay byte-identical on their shared constants:

- `data_collection/data_collection.ino` — records labelled CSV grasps.
- `inference/inference.ino` — runs the trained RF and prints a class label.

The same `applyAmpSettings()`, `CONTACT_THRESHOLDS`, `CONTACT_CONFIRM_SAMPLES`,
`PROFILE_BIN_*`, `motor_field_profile` logic, dual-t1 detection, and stable-
detection code lives in both files. If you change one, mirror to the other.

### Sensor amplification (matches across both files)

```cpp
inline void applyAmpSettings(MLX90393& s) {
  s.setGainSel(2);                  // 3x XY / 4.8x Z gain
  s.setResolution(0, 0, 0);          // finest LSB on all axes
  s.setOverSampling(2);              // 4x ADC averaging
  s.setDigitalFiltering(2);          // on-chip IIR
  s.setTemperatureCompensation(1);   // reduce thermal drift
}
```

These were arrived at after testing. Lower OSR/DF makes the noise floor too
high; higher OSR/DF makes the read budget at 50 Hz too tight (6 sensors ×
~3 ms per read + TCA switching).

### I2C reliability (every boot)

```cpp
Wire.begin();
delay(150);          // MLX90393 chips need ~5-10 ms after VDD; 150 ms is safe
tcaDisableAll();     // resets TCA9548A to known state (all channels off)
```

`initOne()` retries the address probe up to 4 times with 50 ms gaps before
giving up on a sensor. Missing sensors get `sensor_present[i] = false` and
their CSV columns become exact zeros (instead of hanging the firmware).

### Motor-field profile — the key invention

The MLX90393 sensors pick up the rotor magnet of the Dynamixel. As the
motor rotates 60→110°, the field at each sensor changes by tens of µT
**even with no object in the hand**. That motor-induced field was dominating
the magnetic signal we tried to use for contact detection.

Fix: an "empty-hand" baseline captured per encoder degree, subtracted at
runtime so the residual is mostly tactile contact + noise (no rotor).

**Constants**:
```cpp
const int PROFILE_BIN_DEG_MIN = (int)OPEN_POS_DEG;   // 60
const int PROFILE_BIN_DEG_MAX = (int)CLOSE_POS_DEG;  // 110
const int PROFILE_NUM_BINS    = 51;                  // one bin per degree
float    motor_field_profile[51][4][3];              // ~2.4 KB
uint16_t profile_count[51];
bool     profile_calibrated = false;
```

**Calibration** (triggered by `r` command, 2-step protocol):
1. User sends `r`. Firmware prints "REMOVE ALL OBJECTS, send 'y' to begin".
2. User confirms with `y`. Motor sweeps slow (3 RPM) 60→110° while sampling.
3. Online running mean per encoder-degree bin (no big sum buffers).
4. Empty bins filled by nearest-neighbour after the sweep.
5. `profile_calibrated = true` if ≥ half the bins were populated.

**Run-time subtraction**: `readAllSensors()` calls `getBaseline()`, which
returns `motor_field_profile[bin]` if `profile_calibrated` is true, else
falls back to the static open-position baseline captured at boot.

**Profile is RAM only.** Power-cycle wipes it. The firmware prints a loud
warning at boot until `r` is run. logger.py prompts for it before grasps.

### Contact thresholds (post-subtraction)

```cpp
const float CONTACT_THRESHOLDS[4][3] = {
  {40.0f, 40.0f, 65.0f},  // s0 Index ID1
  {40.0f, 40.0f, 60.0f},  // s1 Index ID2
  {25.0f, 25.0f, 35.0f},  // s2 Thumb ID3 (base)
  {25.0f, 25.0f, 30.0f}   // s3 Thumb ID4 (TIP) — most sensitive overall
};
const uint32_t CONTACT_IGNORE_MS       = 150;
const int      CONTACT_CONFIRM_SAMPLES = 8;     // 160 ms sustained at 50 Hz
```

**Rationale**:
- Z axes on s0/s1 had empty-grasp residuals reaching 60+ µT (the index Z
  axis has the largest residual after motor-field subtraction, probably
  because the rotor's motion projects most strongly onto Z at those
  sensors' physical position). Thresholds set above observed empty peaks.
- s2/s3 (thumb) Z residuals were much smaller (~30–40 µT max). Thresholds
  just above that floor.
- **s3 (thumb tip) is the most sensitive sensor** because it is the
  dominant first-contact sensor — 35.5% of training-time t1 events fired
  on s3 vs 11.6% on s2.
- `CONTACT_CONFIRM_SAMPLES = 8` requires 160 ms of sustained over-threshold
  signal. Single noise spikes don't qualify. Real grasp loading lasts
  200–500 ms so contact still triggers easily.

If empty-grasp false-magnetic rate creeps above ~1/10, the typical fix is
either bump CONFIRM to 10, or raise the dz threshold of whichever sensor is
firing (run `motor_field_check.py` to see).

### Dual t1 detection

```cpp
// Path 1: magnetic — any enabled axis sustains over-threshold for CONFIRM samples
// Path 2: encoder stall — encoder hasn't moved by ENCODER_STALL_DEG for
//                        ENCODER_STALL_CONFIRM samples, gated by:
//                          - ts >= CONTACT_IGNORE_MS (no early-grasp stalls)
//                          - hand has actually moved by ENCODER_MOVING_MIN_DEG
```

`t1_source` is logged 0=none, 1=magnetic, 2=encoder_stall, 3=both.

This dual-path was added because **soft objects through stiff silicone do
not produce strong enough magnetic signal**. Encoder stall is the safety
net: when the motor physically stops moving against the object, we declare
t1 fired even though no magnetic axis crossed threshold.

Encoder-stall constants:
```cpp
const float   ENCODER_MOVING_MIN_DEG = 1.0f;
const float   ENCODER_STALL_DEG      = 0.3f;
const uint8_t ENCODER_STALL_CONFIRM  = 4;
```

### Stable detection + t2 capture

After t1 fires, count samples where `delta_enc < ENCODER_STILL_DEG (0.5f)`.
After `ENCODER_STILL_COUNT (5)` consecutive still samples, mark stable.
Set `time_to_stall_ms = ts` **here, NOT at t1**. (Critical bug fix —
inference.ino used to set it at t1, which broke train/inference parity.)

After `HOLD_DURATION_MS (800)` past stable detection, capture t2: sensor
deltas, encoder, load, current. Break loop.

---

## 4. CSV format (27 columns)

```
grasp_id, label, timestamp_ms, enc_deg,
s0_dx, s0_dy, s0_dz, s1_dx, s1_dy, s1_dz,
s2_dx, s2_dy, s2_dz, s3_dx, s3_dy, s3_dz,
t1_flag, t2_flag, t1_ms, t2_ms,
load_t1, load_t2, current_t1, current_t2, enc_t1, time_to_stall_ms,
t1_source
```

- `s*_d*` are **post-subtraction deltas** (raw − motor_field_profile[bin]).
- `t1_flag` and `t2_flag` are 0/1 markers: at most one row per grasp has
  t1_flag=1, at most one has t2_flag=1.
- `t1_ms`, `t2_ms`, `enc_t1`, `time_to_stall_ms`, `t1_source`, `load_t*`,
  `current_t*` are set once when their event fires and then repeated on
  every subsequent row (logger-side convenience).

**Old CSVs** (from before the motor-field profile, before the time_to_stall_ms
fix, or from 6-sensor middle-finger era) are **incompatible**. Delete them
or move to an archive folder before retraining.

---

## 5. Feature vector (57 features)

Exactly mirrored between `train_and_export.py` and `inference.ino`. A
compile-time `#error` in inference.ino guards against `MODEL_NUM_FEATURES`
mismatch with `NUM_FEATURES`.

| Index range | Feature group |
|---|---|
| 0–11 | `peak_abs` of 12 magnetic channels (max |delta| across whole grasp) |
| 12–23 | `mean_abs` of 12 channels (sum of |delta| / sample_count) |
| 24–35 | `RMS` of 12 channels (sqrt(sum of delta² / sample_count)) |
| 36–47 | raw deltas at t2 (s0_dx, s0_dy, s0_dz, …, s3_dz) |
| 48–51 | 4 sensor magnitudes at t2 (sqrt(dx²+dy²+dz²) per sensor) |
| 52 | `enc_t2` (encoder angle at t2) |
| 53 | `load_t2` (Dynamixel PRESENT_LOAD at t2) |
| 54 | `current_t2` (PRESENT_CURRENT at t2) |
| 55 | `time_to_stall_ms` (time of stable detection — NOT t1 firing) |
| 56 | `enc_t1` (encoder angle when t1 fired) |

Channel order within each block follows `SENSOR_COLS`:
`s0_dx, s0_dy, s0_dz, s1_dx, s1_dy, s1_dz, s2_dx, s2_dy, s2_dz, s3_dx, s3_dy, s3_dz`.

`t1_source` is logged in the CSV for diagnostics but is **not** a feature.

---

## 6. Random Forest training

```python
RandomForestClassifier(
    n_estimators=trees,        # default 25
    max_depth=depth,           # default 8
    min_samples_leaf=2,        # prevents 1-sample leaves
    class_weight="balanced",   # auto-weight by inverse class frequency
    random_state=42,
    n_jobs=-1,
)
```

- 5-fold StratifiedKFold cross-validation.
- Confusion matrix is **row-normalised percentages**, saved with `.2f`
  formatting (e.g. `92.31`) and `Blues` colormap. No colorbar.
- Exports: `model.h` (micromlgen RF), `model_config.h` (NUM_FEATURES +
  class accessor macro), `class_names.h`, `feature_names.txt`,
  `model_info.txt`, plus `confusion_matrix_cv.png` in `grasp_data/`.

**Avoid `--depth 10` with < 200 grasps/class** — overfits. Stick to depth 8.

---

## 7. Data collection workflow

```
# Once at session start:
1. Power on OpenRB-150 (motor power + USB).
2. python training\logger.py
3. Send 'r' → logger prompts "remove all objects, type y to begin sweep".
4. Type 'y'. Wait ~6 s for "# Profile sweep done … CALIBRATED".
5. Send '5' (no_object) once and confirm magnetic does NOT fire (must show
   `# t1 detected (encoder_stall)`). If it fires magnetic on empty,
   thresholds need raising or CONFIRM needs bumping (see section 13).

# For each grasp:
6. Place object. Type label number (1–5). Wait for "# Grasp complete and valid".
7. Vary placement / rotation slightly between grasps.

# Multi-session protocol (critical for generalisation):
8. After ~25 grasps, power-cycle the OpenRB. Wait 5 minutes.
9. Send 'r' → 'y' again (each session needs its own calibration).
10. Another ~25 grasps. Repeat for a 3rd session if possible.

# Result: ~75 grasps split across 3 sessions, alternating-class within each.
```

**Why alternating, not blocked**: if you do 25 cube_soft in a row, then 25
cube_stiff, baseline drift over those 25 grasps becomes a per-class feature
the model can memorise — leaks session structure into "class structure".

**Why multi-session**: cross-validation within one session is hot-potato
optimistic. The model finds within-session quirks (temperature curve,
operator habits) that vanish on the next power-cycle. Multi-session forces
generalisation. Expected effect: CV accuracy drops ~10 percentage points
versus single-session CV, but **real deployment accuracy rises to match CV**.

---

## 8. Training + deployment workflow

```
# Train
cd <repo>\LatestCode_03052026\prosthetic_hand_fixed
python training\train_and_export.py --trees 25 --depth 8

# Outputs land in:
#   inference\model.h           (model body)
#   inference\model_config.h    (MODEL_NUM_FEATURES, MODEL_CLASS)
#   inference\class_names.h     (alphabetical class array)
#   inference\feature_names.txt
#   inference\model_info.txt
#   training\grasp_data\confusion_matrix_cv.png

# Flash inference
1. Open inference\inference.ino in Arduino IDE.
2. Verify it compiles — the #error guard will fail if NUM_FEATURES drift.
3. Flash to OpenRB-150.

# Run inference
4. Send 'r' → 'y' (no object) to calibrate motor-field profile (fresh per power cycle).
5. Place object, send 'g'. Firmware prints "RESULT: <class>".
   - "RESULT: No object detected" = predicted no_object.
   - "RESULT: <class>" = predicted soft/stiff cube/cylinder.
   - "RESULT: unknown_object (...)" = hard failure (no t1 or no t2).
```

`DUMP_FEATURES_FOR_DEBUG 1` in inference.ino prints the 57-feature vector
before each `predict()` call — use this when accuracy drops in deployment
to verify feature parity with training.

---

## 9. File layout

```
prosthetic_hand_fixed/
├── CONTEXT.md                       <-- this file
├── README.md
├── data_collection/
│   └── data_collection.ino          <-- main firmware (label-driven grasps -> CSV)
├── inference/
│   ├── inference.ino                <-- inference firmware (RF prediction)
│   ├── class_names.h                <-- auto-generated by train_and_export
│   ├── model_config.h               <-- auto-generated
│   ├── model.h                      <-- auto-generated, ~50 KB on disk
│   ├── feature_names.txt            <-- auto-generated
│   └── model_info.txt               <-- auto-generated
├── sensor_test/
│   └── sensor_test.ino              <-- bench-test single strip (no mux)
└── training/
    ├── logger.py                    <-- serial logger + label prompt
    ├── train_and_export.py          <-- RF train + micromlgen export
    ├── threshold_check.py           <-- diagnostic: noise floor vs thresholds
    ├── motor_field_check.py         <-- diagnostic: rotor-vs-sensor correlation
    └── grasp_data/                  <-- CSV output (logger writes here)
```

Logger writes CSVs to its own folder via `os.path.dirname(os.path.abspath(__file__))`
anchoring — runs from any cwd land in the right place.

---

## 10. Diagnostic scripts (run these before guessing)

| Script | When to use | What it tells you |
|---|---|---|
| `motor_field_check.py` | First time you suspect motor-field is dominating | Correlation between enc_deg and each channel; if |r| > 0.5 on Z axes, subtraction will help |
| `threshold_check.py` | After collecting no_object grasps; tuning thresholds | Noise p99 + signal p95 per axis, dead-channel detection, t1_source distribution, first-trigger sensor |
| `sensor_test.ino` | Before mounting a new strip to the hand | Live x/y/z deltas per sensor on Serial Plotter; verifies a strip's chips respond and pads deflect |

---

## 11. t1_source distribution we expect

Per training-time class:

| Class | %magnetic | %stall | Notes |
|---|---|---|---|
| no_object | 0% | 100% | Magnetic should never fire on empty grasps — if it does, raise thresholds or bump CONFIRM |
| cube_stiff | 60–97% | 3–40% | Strongest signals, fires magnetic easily |
| cylinder_stiff | 30–70% | 30–70% | Curvature spreads contact force, sometimes stalls before crossing threshold |
| cube_soft | 0–30% | 70–100% | Silicone damps soft contact; mostly stall |
| cylinder_soft | 0–30% | 70–100% | Same |

Real deployment numbers depend on each session's calibration. The exact
percentages don't matter for accuracy — what matters is **consistency
within a class**. If half of cube_stiff fires magnetic and half stalls,
`enc_t1` distribution becomes bimodal and the model has to handle that.

---

## 12. Sensor-by-sensor first-contact dominance (from 100-grasp training)

| Sensor | % of grasps where this was the first to trigger t1 |
|---|---|
| s0 Index ID1 | 26.5% |
| s1 Index ID2 | 26.5% |
| s2 Thumb ID3 (base) | 11.6% |
| s3 Thumb ID4 (TIP) | **35.5%** ← dominant |

This is why s3 has the lowest thresholds and is called out in code comments
as "the most sensitive sensor in the system."

---

## 13. Known issues / how we ruled things out

Things we hit and fixed; keep this catalogue to skip the same diagnostic
journey twice.

**Symptom: every grasp prediction was wrong (5% real-world accuracy).**
Root cause: `time_to_stall_ms` was set at t1 firing in inference.ino but
at stable-detection time in data_collection.ino. Train and inference
disagreed on one feature. Fix: set it at stable-detection in both.

**Symptom: random sensors fail to respond at boot.**
Root cause: MLX90393 chips need ~5–10 ms after VDD rises before they ACK
I2C, and a previous run could leave the TCA9548A on a random channel.
Fix: `delay(150); tcaDisableAll();` after `Wire.begin()`, plus a 4-retry
loop in `initOne()` with 50 ms gaps.

**Symptom: middle finger (TCA ch 2) returns constant 0 or pegged values.**
Root cause: hardware failure on TCA ch 2 specifically. Tried I2C scan,
chip reset, threshold tweaks, lowering I2C clock — all confirmed the
sensors don't respond. Fix: removed middle finger from firmware. Do not
re-enable until hardware is repaired.

**Symptom: empty no_object grasps fire magnetic ~30–40% of the time.**
Root cause: residual after motor-field subtraction can still hit 50–60 µT
on index Z axes; old thresholds (40 µT) + old CONFIRM (5 samples) allowed
bursty noise to satisfy the trigger. Fix: raise dz thresholds, bump
CONFIRM to 8. Section 3 has current values.

**Symptom: 98.2% CV accuracy, 5% deployment accuracy.**
Compound root cause: (a) time_to_stall_ms bug (above), (b) session
leakage in CV — 100 grasps in one session share within-session noise,
CV partitioned within that session reports inflated numbers. Fixes:
fixed the bug AND switched to multi-session collection protocol.

**Symptom: motor doesn't reach CLOSE_POS_DEG (110°), stalls early.**
Root cause: usually mechanical drag (silicone sticking, finger linkage,
or torque limit). Diagnostic: in inference, watch the
`# Motor travel: enc_start=… enc_t2=… reached XX% of expected` line.
If empty-hand grasps "reach < 95%", investigate hand mechanics, not firmware.

**Symptom: stiff cube classified as cylinder_soft.**
Root cause: stiff cube wasn't pressing thumb tip firmly → s3 signal was
weak (~75 µT post-amp) → encoder_stall fired instead of magnetic, giving
soft-object-like enc_t1 distribution. Fix: position the object so the
thumb tip is engaged. Long-term fix: retrain on multi-session data so the
model sees varied positions.

---

## 14. Do-not-touch list

These are deliberate and load-bearing. Don't change without reading why.

1. **`class_names.h` is auto-generated.** Edit triggers automatic
   regeneration on the next `train_and_export.py` run, so manual edits
   are lost. Class order must be alphabetical (sklearn LabelEncoder
   sorts internally and uses that order in `model.h`).

2. **`MODEL_NUM_FEATURES` (model_config.h) must equal `NUM_FEATURES`
   (inference.ino).** Both currently 57. There's a compile-time `#error`
   in inference.ino that catches drift.

3. **CONTACT_THRESHOLDS and CONTACT_ENABLED must be byte-identical
   between data_collection.ino and inference.ino.** They drive t1
   detection in both, and any mismatch breaks training-inference parity.

4. **applyAmpSettings() must be byte-identical between the two .ino
   files.** Different amp settings = different sensor scale = features
   shift = the trained model is wrong about input distributions.

5. **Profile bins are derived from OPEN/CLOSE_POS_DEG.** If you change
   the motor range, `PROFILE_NUM_BINS` recomputes automatically. Don't
   hard-code a different bin count.

6. **`time_to_stall_ms` is set at stable-detection, not at t1 firing.**
   This was a critical bug fix. Both .ino files now set it in the
   `if (still_count >= ENCODER_STILL_COUNT)` block.

7. **`r` command does motor-field profile sweep, not static baseline.**
   The static baseline still happens at boot as a fallback. The `r`
   command requires user confirmation with `y` to avoid accidental
   profile-with-object capture.

8. **Sensor `begin(A1, A0)` calls must match the physical strip wiring**
   (which A0/A1 pin is high). See section 2 table.

9. **Middle finger (sensors 4, 5, TCA ch 2) is removed.** Don't add it
   back without first confirming the hardware works (use sensor_test).

10. **Old CSVs are not compatible with the current motor-field profile
    baseline scheme.** Delete or archive them before retraining.

---

## 15. Open items / what's next

Active when this CONTEXT.md was written:

- [ ] Multi-session data collection: 3+ sessions of ~25 grasps each,
      alternating-class, with `r` calibration per session.
- [ ] Retrain on the multi-session dataset.
- [ ] Verify CV vs real-world gap is < 10 percentage points.
- [ ] Document the achieved per-class precision/recall.

Probably-needed-later improvements (don't pursue unless deployment
accuracy is < 75%):

- [ ] Add **paired-sensor difference features**: |s0_mag − s1_mag|,
      |s2_mag − s3_mag| at t2 (per-finger asymmetry), and Z/XY ratios
      per sensor (compression vs shear). Would add ~10 features →
      NUM_FEATURES = 67. Most likely to help with cube vs cylinder
      discrimination in the soft classes.
- [ ] Persist motor-field profile to EEPROM so power-cycle doesn't
      require re-`r`.
- [ ] Build a true confidence-based `unknown_object` path (requires
      modifying micromlgen RF export to expose vote counts; currently
      `unknown_object` only fires on hard failures).

Not pursuing unless asked:

- Re-introducing the middle finger (hardware first).
- Adding extra time-series features (per-axis slope, peak time, etc.) —
  the current 57 are working; more features on 100 grasps probably
  overfits.

---

## 16. Glossary of constants you'll touch

| Constant | Where | Value | Effect |
|---|---|---|---|
| `OPEN_POS_DEG` | both .ino | 60 | Hand-open encoder position |
| `CLOSE_POS_DEG` | both .ino | 110 | Hand-close target |
| `GRASP_SPEED_RPM` | both .ino | 10 | Motor speed during grasps |
| `CALIB_SPEED_RPM` | both .ino | 3 | Motor speed during `r` sweep |
| `SAMPLE_INTERVAL_MS` | both .ino | 20 | 50 Hz sensor sampling |
| `BASELINE_SAMPLES` | both .ino | 80 | Static-baseline samples at boot |
| `CONTACT_IGNORE_MS` | both .ino | 150 | Ignore early-grasp transients |
| `CONTACT_CONFIRM_SAMPLES` | both .ino | 8 | 160 ms sustained over-threshold needed |
| `ENCODER_STILL_DEG` | both .ino | 0.5 | Encoder delta to count as "still" |
| `ENCODER_STILL_COUNT` | both .ino | 5 | Still samples to mark stable |
| `ENCODER_STALL_DEG` | both .ino | 0.3 | Encoder delta to count as "stall" |
| `ENCODER_STALL_CONFIRM` | both .ino | 4 | Stall samples before t1 (encoder path) |
| `ENCODER_MOVING_MIN_DEG` | both .ino | 1.0 | Hand must have moved this much before stall counts |
| `HOLD_DURATION_MS` | both .ino | 800 | Hold before t2 after stable detection |
| `GRASP_TIMEOUT_MS` | both .ino | 5000 | Abort grasp if stable not reached |
| `NUM_SENSORS` | both .ino | 4 | Index ID1/ID2 + Thumb ID3/ID4 |
| `NUM_CHANNELS` | inference.ino | 12 | 4 sensors × 3 axes |
| `NUM_FEATURES` | inference.ino | 57 | Must equal MODEL_NUM_FEATURES |
| `PROFILE_NUM_BINS` | both .ino | 51 | Derived: CLOSE − OPEN + 1 |

---

End of CONTEXT.md. If you've reached this line you have everything you need
to continue the project. Ask only what isn't covered above.

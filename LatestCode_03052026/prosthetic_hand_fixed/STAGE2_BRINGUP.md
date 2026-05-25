# Stage 2 bring-up notes

Companion to `CLAUDE.md`. Covers the second-phase additions:
motors 2+3, EMG, mode-cycle button, LED indicator. Read this once
before you wire the second-phase hardware to the OpenRB-150.

Stage-1 (object-recognition pipeline) is unchanged — same sensor
firmware, same Python pipeline, same model. The OpenRB sketch was
extended; existing serial commands (`p e s x o c h g r a` and the
`M,<ms>,<enc>,<load>,<current>` stream) still behave exactly as
before.

---

## 1. Pin assignments (OpenRB-150)

| Pin | Role | Wiring |
|---|---|---|
| `A0` | Potentiometer wiper (EMG threshold) | 10 kΩ pot. Outer pins → 3V3 and GND, wiper → A0. |
| `A1` | EMG analog input | Output of EMG front-end module. Must share GND with OpenRB. |
| `D4` | Mode-cycle push button | `INPUT_PULLUP` in firmware. Wire the button between **D4** and **GND** (active-low). No external pull-up needed. |
| `D5` | Mode-indicator LED | Active-high. Drive through the **680 Ω** series resistor → LED anode → LED cathode → GND. |

ADC resolution is explicitly set to 10-bit in `setup()` so the
potentiometer's `analogRead() / 4` gives 0–255 — matching the
original `latestWorkingCodeEMG.ino` reference behaviour.

If you change any pin assignment, update the matching `PIN_*`
`const uint8_t` lines at the top of `openrb/motor_control/motor_control.ino`.

---

## 2. Dynamixel motor IDs and ranges

| ID | Model | Role | Range | Home | Speed |
|---|---|---|---|---|---|
| 1 | XM430-W350-R | Grip (thumb + index) | 66° (open) → 110° (close, full); screwing mode caps at **97°** | 66° | 10 RPM grasp, 1 RPM calibration |
| 2 | XC330-T288 | Ring + little (cable-actuated soft fingers) | 30° (open) → 260° (close) | 30° | 10 RPM |
| 3 | XC330-T288 | Wrist (full 0°–360° envelope) | 0° → 360° (operational: 30°, 150°, 270°) | **150°** | 20 RPM |

All three motors share the same RS485 bus at 57600 baud, Protocol 2.0.
The firmware probes each ID independently at boot; missing motors
emit `# WARN motor N ...` and their handlers no-op (so the rig stays
usable for partial bring-up).

**Travel-limit enforcement.** All goal-position commands for IDs 2
and 3 are clamped to the ranges above in firmware (`clampf`). Manual
`w<deg>` (wrist) and `f<deg>` (soft fingers) commands are clamped too.
Don't bypass the clamps without confirming the mechanical envelope
first — XC330 cables can bind and the wrist linkage may collide.

### Wrist (ID 3) — home + screwing convention (updated 2026-05-20)

- **Home position: 150°.** The wrist parks here at boot, after any
  mode change, after a `h` (home) command, and after a long-press
  abort. Applies to all four grasp modes.
- **Screwing-cycle start: 270°.** When the user's first flex in
  screwing mode fires, the wrist moves from 150° to 270° (the
  screwing operational position) as part of the grip step. After a
  screwing cycle completes and the 3-sec auto-return fires, the
  wrist sits at 270° ready for the next cycle. Mode-exit returns
  it to 150°.
- **Screwing-cycle end: 30°.** A screwing flex rotates the wrist
  from 270° down to 30° — a **240° rotation** via 180°. The motor
  is in X-series Position Control mode (mode 3), which takes the
  shortest path between two angles by default; that would be the
  60° path via 0°/360°, the *wrong* direction. The firmware
  therefore commands an intermediate goal of **180°** before the
  final 30° goal, forcing the long 240° path through the lower
  half of the rotation envelope. `SCREW_INTERMEDIATE_DEG = 180.0f`
  in the firmware.
- **Travel envelope clamped to [0°, 360°].** Single-turn. Manual
  `w<deg>` commands are clamped to this range.

If the physical screwing direction is reversed (wrist hardware
assembled mirrored), swap `SCREW_START_DEG` and `SCREW_END_DEG` —
that is the single point of change. The `SCREW_INTERMEDIATE_DEG`
stays at 180° regardless of direction (it's the midpoint of the
long path either way).

### Motor 2 (ID 2) — current safety (updated 2026-05-20)

The XC330 driving the soft-finger cable cannot keep increasing torque
forever: the cable will bind or the linkage will overload. The
firmware monitors `PRESENT_CURRENT` while closing and freezes the
goal at the present position once `|current| > M2_CURRENT_LIMIT`
for `M2_CURRENT_CONFIRM` consecutive samples.

> **Note (2026-05-20):** the safety previously read `PRESENT_LOAD`,
> which always returns 0 on the XC330 X-series (same gotcha that
> bit the M1 grip safety). The check now reads `PRESENT_CURRENT`,
> which is the working torque proxy. If you observed soft fingers
> "keep pulling and never stop" before this fix, that bug is why.

| Parameter | Default | Tune by... |
|---|---|---|
| `M2_CURRENT_LIMIT` | 300 (mA, XC330 unit is 1 mA per LSB) | Soft fingers don't wrap firmly → raise (e.g. 400). Cable binds, motor warms up → lower (e.g. 200). XC330 stall is ~910 mA, so 300 leaves comfortable margin. |
| `M2_CURRENT_CONFIRM` | 3 samples | Increase to 5–8 if a momentary spike causes the soft fingers to freeze prematurely. |
| `M2_MOTION_TIMEOUT_MS` | 4000 ms | Hard timeout regardless of current — covers the case where the cable goes slack and current never builds. |

When the safety triggers, the firmware emits
`# M2_STALL <ms> <enc> <current>`. The position is held by setting
the goal to the present position, so the motor's internal PID keeps
a firm but non-escalating grip. **The soft fingers wrap and stop.**

---

## 3. Modes and gestures

| Mode | LED pattern | Cycle index |
|---|---|---|
| `POWER` | solid | 0 |
| `TRIPOD` | 1 Hz blink (500 / 500 ms) | 1 |
| `OBJ_RECOG` | 4 Hz blink (125 / 125 ms) | 2 |
| `SCREWING` | double-blink (100 / 100 / 100 / 700 ms) | 3 |

### Button
- **Short press (< 400 ms)** → cycle to next mode (`POWER → TRIPOD → OBJ_RECOG → SCREWING → POWER`).
- **Long press (≥ 1500 ms)** → abort + home all (open M1, open M2, wrist to **150°**). Fires immediately at the threshold, even mid-grasp.
- 400–1500 ms is the deliberate ambiguous band; nothing happens. Press deliberately short or hold deliberately long.

### EMG (in-mode trigger) — updated 2026-05-20

EMG behaves as a **push button**: only the rising edge actuates
motors. The falling edge is observed (emits `# EMG_RELEASE`) but
does not change any motor state. This replaces the earlier
"flex closes, release opens" semantics.

- 125 Hz loop, 80-sample moving average over rectified differential (≈ 640 ms window). **Identical to the reference `latestWorkingCodeEMG.ino` signal-processing path** — read raw, compute signed diff against `last_raw`, rectify, push into ring buffer, average.
- Threshold = `analogRead(A0) / 4`, floored at 10. Pot fully CCW disables EMG (threshold higher than any plausible signal).
- Hysteresis = **15** counts (raised from 8 on 2026-05-20 for binary edge-detection robustness). After a rising edge, the smoothed value must drop below `threshold - 15` before the next rising edge can fire.
- Rising edge above threshold → `# EMG_FLEX <ms> <smoothed> <threshold>`.
- Falling edge below `threshold - HYST` → `# EMG_RELEASE <ms> <smoothed> <threshold>`.

### Per-mode EMG behaviour

| Mode | 1st EMG flex | 2nd EMG flex | 3rd EMG flex | EMG release |
|---|---|---|---|---|
| POWER | close M1 + M2 | open M1 + M2 | (toggle continues) | ignored |
| TRIPOD | close M1 | open M1 | (toggle continues) | ignored |
| OBJ_RECOG | emit event → daemon runs classifier → hand auto-opens at `# GRASP_COMPLETE` | (system already idle — next flex starts a new classification) | — | ignored |
| SCREWING | close M1 to **97°**, move wrist to **270°** (M2 stays open) | rotate wrist **270° → 30°** via 180° | open M1 (M2 was never closed), start 3-sec timer | ignored |

### Screwing sequence (updated 2026-05-20)

The screwing cycle is now **3 EMG-driven steps + 1 automatic step**:

| Step | Trigger | Action |
|---|---|---|
| 0 → 1 (GRIP) | flex 1 | Close M1 to 97° (hard limit, **not** 110°). **M2 stays open** — screwing is a tripod-style grasp with the soft fingers out of the way. Move wrist to 270° (screwing-operational position). On the very first cycle after entering screwing mode, wrist moves from 150° home to 270°; on subsequent cycles wrist is already at 270° from the auto-return and this `wristMoveTo` is a no-op. |
| 1 → 2 (CW) | flex 2 | Rotate wrist **270° → 30°** via 180°. The intermediate goal of 180° forces the 240° long path; X-series mode-3 position control would otherwise take the 60° short path via 0°/360°. Blocking; takes ~2 s at 20 RPM. |
| 2 → 3 (RELEASE) | flex 3 | Open M1 (back to 66°). M2 was never closed, so no action needed for it. Start a 3-sec timer. Wrist stays at 30°. |
| 3 → 0 (AUTO_RETURN) | **3-sec timer**, no flex | Rotate wrist **30° → 270°** via 180° (long path again). Emits `# SCREW_AUTO_RETURN <ms>`. Returns to step 0, ready for the next cycle's flex 1. |

Each EMG-driven step requires a separate flex (release first, then
flex again). The system **ignores flex events during step 3** so the
user cannot skip the auto-return. Mode-exit (button short-press)
cancels any pending auto-return and returns the wrist to 150°.

---

## 4. Serial protocol — additions

The stage-1 commands (`p e s x o c h g r a`) and the `M,...` stream
line are unchanged in format. Added in stage 2:

### New commands
| Command | Effect |
|---|---|
| `m0` `m1` `m2` `m3` | Force mode (bypasses button). Useful for laptop-driven tests. |
| `w<deg>` | Manual wrist target. Clamped to **[30°, 270°]** (tightened 2026-05-20 after the gears jammed beyond 270°). e.g. `w90` |
| `f<deg>` | Manual soft-finger (M2) target. Clamped to [30°, 260°]. e.g. `f200` |
| `?` | Print force fail-safe diagnostic line (current_limit + peak_load + peak_current). |
| `R<id>` | **Reboot a stuck motor**. Use when a motor stops responding to position commands but still pings (HARDWARE_ERROR_STATUS shutdown — typically overload, overheat, or encoder fault). E.g. `R3` reboots the wrist motor after it jammed against a mechanical stop. Takes ~2.5 s. Emits `# REBOOTING` then `# REBOOTED` (or `# REBOOT_FAIL` if the motor doesn't come back). After reboot the motor is re-initialised to its mode-appropriate safe position. **Avoids the need for a full 12 V power cycle.** |
| `I<id>` | **Toggle motor direction** for M2 (`I2`) or M3 (`I3`). Use when a motor unexpectedly rotates the opposite direction (DRIVE_MODE register flipped). The new value persists in EEPROM, but `setup()` also forces `M<id>_DRIVE_MODE_DEFAULT` on every boot — so to make a direction change permanent across re-flashes you must also update that constant in the firmware. |

### New events
| Event | Emitted when |
|---|---|
| `# MODE <n> <name>` | Boot, and after any mode change (button or `m<n>`). |
| `# EMG_FLEX <ms> <smoothed> <threshold>` | EMG smoothed value crosses threshold upward. |
| `# EMG_RELEASE <ms> <smoothed> <threshold>` | EMG drops below `threshold - 8`. |
| `# WRIST_AT <ms> <enc>` | Wrist reached its goal (or timed out — same emission). |
| `# M2_AT <ms> <enc>` | M2 reached its goal. |
| `# M2_STALL <ms> <enc> <load>` | M2 load-safety triggered; goal frozen at `<enc>`. |
| `# SCREW_STEP <n> <name> <ms>` | Step `n` of the screwing sequence started (n=0 GRIP, n=1 CW, n=2 RELEASE). |
| `# SCREW_AUTO_RETURN <ms>` | The automatic 3-sec post-open wrist-return step fired (30° → 270° via 180°). |
| `# REBOOTING ID=<id> at <ms>` | `R<id>` command sent a `dxl.reboot()` instruction to the motor. |
| `# REBOOTED ID=<id> at <ms>` | Motor came back online after reboot and was re-initialised (operating mode, drive mode, torque on, parked at safe position). |
| `# REBOOT_FAIL ID=<id> at <ms>` | Motor did not respond to ping after reboot. May need a power cycle. |
| `# DRIVE_MODE ID=<id> was=<old> now=<new> ...` | Response to runtime `I<id>` toggle. Old and new register values shown. |
| `# DRIVE_MODE_RESET ID=<id> was=<old> now=<new>` | `setup()` detected the DRIVE_MODE register didn't match the firmware default and rewrote it to enforce consistent rotation direction across power cycles. |
| `# HOMED <ms>` | Long-press abort / home completed. |
| `# BUTTON_SHORT <ms>` | Diagnostic: short-press detected. |
| `# BUTTON_LONG <ms>` | Diagnostic: long-press detected. |
| `# MOTOR <id> <model> ok\|none` | One line per motor on `e` command, plus `# MOTOR found ID=...` at boot. (Existing format; now emitted for all 3 IDs.) |

`hand/protocol.py` parses any `# <name> <args...>` line into an
`Event(name, args)` without changes; the daemon and any future UI
can dispatch on `evt.name`.

---

## 5. Python daemon (`python/daemon.py`)

Long-running laptop process. Run after `run.py` has produced
`data/calibration.npz` and `data/model.pkl`:

```
python python/daemon.py            # verbose logging to stderr
python python/daemon.py --quiet    # only RESULT: lines on stdout
```

Behaviour:
- Holds the dual USB connection open and keeps streams on.
- Mirrors `# MODE` events into an in-memory state.
- In `OBJ_RECOG` mode, on `# EMG_FLEX` calls `record_grasp()` (which
  sends `g` to the OpenRB and follows the existing grasp pipeline),
  extracts the 57-feature vector, predicts, and prints
  `RESULT: <class>   confidence: NN.N%   t1_source=...`.
- In other modes the daemon is a passive logger; the firmware drives
  the hardware directly.

If model or calibration is missing at startup, the daemon still runs
but OBJ_RECOG flexes return `RESULT: unknown_object (...)`. The other
three modes work either way — they are firmware-only.

### Calibration freshness (matters for OBJ_RECOG confidence)

The motor-field profile (`data/calibration.npz`) is what makes the
sensor subtraction work. It's captured during the `r` calibration
sweep and is **session-specific** — it depends on:

- The exact resting position of the soft fingers (M2) and wrist (M3)
- Ambient temperature (Dynamixel rotor field drifts slightly with temp)
- Mechanical settling of the silicone glove
- USB grounding state at calibration time

If the profile is stale (collected days/weeks ago, or after a
mechanical adjustment), the residual after subtraction will be
noisier than what the model was trained on, and **confidence on the
soft classes will drop noticeably** (`cube_soft` and `cylinder_soft`
are the most sensitive because their magnetic signal is small to
begin with).

**Recommendation for demo / inference sessions:**

1. Before opening the UI for a demo, run `python python/run.py` once
   to do calibration plus a few warm-up grasps. This refreshes
   `calibration.npz`.
2. Power-cycle the OpenRB between calibration and demo only if
   needed (e.g. to test the boot sequence). Otherwise leave it on —
   the Python-side profile persists regardless of OpenRB state.
3. If you have to skip step 1, you can also trigger a fresh
   calibration mid-UI-session by typing `r` in the serial input
   box on the UI. The firmware will run the slow sweep; **but**
   `server.py` does not currently rebuild the Python-side profile
   from the live sweep — that's a `run.py`-only path right now.
   So the `r` command alone improves nothing for confidence; you
   need the full `python python/run.py` workflow.

In short: **stale calibration is the most likely culprit when you
see lower confidence in the UI than during recent training.** Run
`python python/run.py` to refresh and you should see confidence
return to training levels.

---

## 6. Bring-up checklist

Do these in order the first time the stage-2 hardware is wired up.
Each step assumes the previous one passed.

1. **Power off.** Disconnect 12 V before changing any wiring.
2. **Set Dynamixel IDs.** Use ROBOTIS Dynamixel Wizard to set the
   XC330 motors to IDs **2** and **3** (factory default is ID 1 →
   they will conflict with the XM430 if left unchanged). Baud 57600,
   Protocol 2.0.
3. **Wire EMG / pot.** Connect EMG front-end output to `A1`,
   potentiometer wiper to `A0`. EMG GND must be common with OpenRB GND.
4. **Wire button.** Push button between `D4` and `GND`.
5. **Wire LED.** OpenRB `D5` → 680 Ω → LED anode → cathode → GND.
6. **Power up + flash.** Upload `motor_control.ino`. Open serial
   monitor at 115200. You should see:
   ```
   # motor_control.ino - OpenRB-150 (stage 2)
   # MOTOR found ID=1 model=...
   # MOTOR found ID=2 model=...
   # MOTOR found ID=3 model=...
   # MODE 0 POWER
   ```
   The LED should be solid on.
7. **Test motor enumeration.** Send `e`. Expect three `# MOTOR ...`
   lines, one per ID, all reporting `ok`.
8. **Test wrist home.** On boot the firmware parks the wrist at 180°.
   Confirm visually. Then send `w0` → wrist rotates fully CW. Then
   `w180` → wrist returns CCW to home. Each should emit `# WRIST_AT`.
9. **Test soft fingers.** Send `f260` → M2 closes; with a foam block
   in the fingers, expect `# M2_STALL` once the load limit hits.
   Then `f30` → opens cleanly with `# M2_AT`.
10. **Test button.** Short-press the button. Expect `# BUTTON_SHORT`
    and `# MODE 1 TRIPOD`; LED switches to 1 Hz blink. Cycle through
    all four modes. Long-press once anywhere; expect `# BUTTON_LONG`
    then `# HOMED`.
11. **Test EMG.** With the pot midway and the EMG electrodes on,
    flex. Expect `# EMG_FLEX ... <smoothed> <threshold>`. Verify
    `smoothed > threshold`. Relax — expect `# EMG_RELEASE`. Adjust
    pot if it triggers on relaxed muscle (CW = raise threshold).
12. **Test POWER mode.** With LED solid (mode 0), flex EMG. M1 + M2
    should both close. Release. Both should open.
13. **Test TRIPOD mode.** Short-press to mode 1 (LED 1 Hz). Flex →
    only M1 closes; M2 stays open. Release → M1 opens.
14. **Test SCREWING mode.** Short-press to mode 3 (double-blink).
    Flex → `# SCREW_STEP 0 GRIP` (M1+M2 close). Relax + flex →
    `# SCREW_STEP 1 ROTATE_CW` (wrist 180°→0°). Relax + flex →
    `# SCREW_STEP 2 RELEASE` (open). Relax + flex →
    `# SCREW_STEP 3 ROTATE_CCW` (wrist 0°→180°). Next flex wraps to
    step 0.
15. **Test OBJ_RECOG mode end-to-end.** Requires that
    `data/calibration.npz` and `data/model.pkl` exist (run
    `python python/run.py` first to calibrate, then
    `python python/run.py train`). With those in place, run
    `python python/daemon.py`. Short-press to mode 2 (LED 4 Hz).
    Place an object. Flex. The daemon should print a `RESULT: <class>`
    line on stdout within ~2 s.

---

## 7. Tuning notes

- **EMG too jumpy / fires on relaxed muscle.** Turn pot CW. If you
  hit the ceiling and it still fires, increase `EMG_MIN_THRESHOLD`
  in the sketch, or extend `EMG_WINDOW` (heavier smoothing at the
  cost of latency).
- **EMG never fires on a clear flex.** Turn pot CCW. If pot is at
  floor and it still misses, verify the EMG front-end is powered and
  the electrodes are well-prepped (skin clean, dry, conductive gel).
- **Button bounces / misses presses.** Raise `BUTTON_DEBOUNCE_MS`
  toward 50 ms.
- **Long-press feels too short / too long.** Adjust
  `BUTTON_LONG_MIN_MS` (default 1500 ms).
- **M2 grip too weak.** Raise `M2_LOAD_LIMIT`. Confirm motor doesn't
  overheat after a few minutes of held grip.
- **M2 grip too forceful / cable binds.** Lower `M2_LOAD_LIMIT`.
- **Wrist rotation too slow.** Raise `WRIST_SPEED_RPM`. XC330 can
  handle 50+ RPM easily; 20 is a conservative default.
- **Wrist overshoots target.** Lower `WRIST_SPEED_RPM`, or accept
  `WRIST_REACHED_TOL_DEG` (currently 3°) as a small static error.
- **Grip motor (M1) breaking gears on stiff objects.** See section 8
  below — this is what the force fail-safe was added to prevent.

---

## 8. Grip motor (M1) force fail-safe

Added 2026-05-20 in response to gearbox failures on stiff
3D-printed PLA cubes. **READ THIS SECTION BEFORE COLLECTING DATA
ON STIFF OBJECTS.**

### What it does

The XM430's internal position PID will push at up to ~3 A against
any obstacle to reach the goal position. On rigid objects that
force is enough to strip the gearbox before the grasp's t2 capture
completes. The firmware now monitors `PRESENT_CURRENT` every
iteration of the grasp loop and, when `|current| > CURRENT_LIMIT_GRASP`
for `CURRENT_CONFIRM` consecutive samples, **freezes the motor at
its current encoder position** via `gotoDeg(readEncoderDeg())`.
The PID then holds with minimal current instead of continuing to
push.

> **Why current and not load?** `PRESENT_LOAD` on the XM430
> X-series always reads 0 — the register exists in the control
> table but is a leftover from older AX/MX-series motors and is
> not implemented on X-series. `PRESENT_CURRENT` (signed, units
> of ~2.69 mA each, range −2047..+2047) is the reliable torque
> proxy. An initial version of this safety used load and silently
> failed — that bug was caught and fixed during finger-pushback
> testing on 2026-05-20.

### What it does NOT do (important clarification)

Of the three "t1 detected" events the firmware emits, **only the
force fail-safe actually stops the motor**. The other two are pure
observation events used to label `t1_source` in the training CSV.

| Path | Lives in | Stops motor? | Used for |
|---|---|---|---|
| Encoder-stall t1 | Firmware | **No** — observation only | CSV `t1_source = 'encoder_stall'`, t1 timestamp |
| Magnetic-t1 | Python (`hand/grasp.py`) | **No** — and *can't*: no command path from laptop back to OpenRB | CSV `t1_source = 'magnetic'`, t1 timestamp |
| **Force fail-safe** | **Firmware** | **Yes** — `gotoDeg(currentEnc)` | Safety only. Emits `# FORCE_STOP` for diagnostics. Also emits a `# T1_STALL` so the Python pipeline still records a valid t1 if magnetic didn't fire. |

Encoder-stall is deliberately left observation-only so soft and
medium-stiff objects can still compress during the 800 ms HOLD as
they always have, **preserving the training-data distribution for
those classes**. Only objects that approach gear-damage load levels
trigger the freeze.

### Constants (in `openrb/motor_control/motor_control.ino`)

| Constant | Default | Effect |
|---|---|---|
| `CURRENT_LIMIT_GRASP` | 200 | Absolute `PRESENT_CURRENT` threshold in XM430 units (1 unit ≈ 2.69 mA, so 200 ≈ 540 mA). When `|current|` exceeds this for `CURRENT_CONFIRM` samples, motor freezes. XM430 stall current is ~855 in these units, so 200 leaves a comfortable margin. |
| `CURRENT_CONFIRM` | 3 | Consecutive over-limit samples needed to trigger. At ~100–150 Hz loop rate, 3 samples ≈ 20–30 ms — fast enough to save gears, slow enough to ignore single-sample noise. |
| `CONTACT_IGNORE_MS` | 150 (existing) | Suppresses the trip for the first 150 ms of the grasp to avoid false-positives from the motor's startup torque spike. |

### Diagnostic events

After every grasp (and every calibration sweep), the firmware
emits one new line right before `# GRASP_COMPLETE` / `# CALIB_COMPLETE`:

```
# PEAK_LOAD <peak_load> <peak_current>
```

`<peak_load>` will always be **0** on the XM430 (see the load-vs-
current note above). `<peak_current>` is the maximum absolute
`PRESENT_CURRENT` value seen during the entire close + HOLD window
(sign preserved). **Watch the second number for tuning.** The `?`
command also reports the most recent values:

```
> ?
# DIAG current_limit=200 current_confirm=3 peak_load_last=0 peak_current_last=187
```

Use this to tune `CURRENT_LIMIT_GRASP` without recompiling: do a
test grasp, send `?`, look at `peak_current_last`, decide if the
threshold should change.

When the safety trips, two extra lines appear:

```
# FORCE_STOP <ms> <enc_at_freeze> <load_at_trip> <current_at_trip>
# T1_STALL <ms> <enc_at_freeze>          (only if encoder-stall hadn't already fired)
```

`<load_at_trip>` will be 0 on XM430. `<current_at_trip>` is the
value that crossed the threshold.

During calibration only, a trip aborts the sweep entirely (since
calibration must run with an empty hand):

```
# FORCE_STOP <ms> <enc> <load> <current>
# CALIB_ABORTED <ms>
# PEAK_LOAD <peak_load> <peak_current>
```

If you see `# CALIB_ABORTED`, **the profile was not updated** — fix
whatever's in the hand and re-run calibration. `calibrate.py` will
time out after 60 s waiting for `# CALIB_COMPLETE`; this is the
expected failure mode.

### Tuning procedure — DO THIS BEFORE GRASPING ANY STIFF OBJECT

This progression is designed so a wrong threshold can never break
gears. Do every step.

**All "peak" values below refer to the SECOND number in the
`# PEAK_LOAD` line** (peak_current). The first number is the
broken `peak_load` reading and will always be 0.

**Step 1 — Empty grasp.** Send `g` with the hand fully open and
nothing in it. Expect: motor reaches 110° smoothly, no
`# FORCE_STOP` line, `# PEAK_LOAD 0 <small>` shows a low
peak_current (typical empirical value: ~30–60). If `# FORCE_STOP`
fires on empty: threshold is too low, raise `CURRENT_LIMIT_GRASP`
by 50 and reflash.

**Step 2 — Verify firmware constants.** Send `?`. Expect:
`# DIAG current_limit=200 current_confirm=3 peak_load_last=0
peak_current_last=<from step 1>`. Confirms the safety is compiled
in and active and that `peak_load_last=0` (PRESENT_LOAD is
non-functional on XM430 — this is expected).

**Step 3 — Finger pushback during grasp (THE CRITICAL TEST).**
Send `g`. As the hand closes, gently push one fingertip back with
your own finger. Expect: `# FORCE_STOP` fires within a fraction of
a second, you can feel the motor go limp, no further force
build-up. **This is your real safety verification.** If
`# FORCE_STOP` does NOT fire: pull your finger away immediately
(no gear risk — the only thing being loaded is your finger).
Check the resulting `# PEAK_LOAD 0 <N>` line — `<N>` is the current
your pushback generated. If `<N>` is above `CURRENT_LIMIT_GRASP`
but no trip fired, something is wrong with the firmware build —
re-verify with `?` that current_limit matches what you set. If `<N>`
is below the limit, lower `CURRENT_LIMIT_GRASP` to just under `<N>`
and reflash. **Never proceed to step 4 without passing step 3.**

**Step 4 — Foam block.** Place compressible foam in the gripper,
send `g`. Expect: HOLD completes normally, no trip. Typical
peak_current: ~80–150. If foam triggers the trip, raise threshold
by 50.

**Step 5 — Soft 3D-printed cube (TPU).** Hold a `cube_soft`
loosely in the gripper so you can pull it away. Send `g`. Expect:
HOLD completes normally. Typical peak_current: ~120–200. If trip
fires on soft cube: threshold is too low for soft objects, raise
by 50.

**Step 6 — Stiff 3D-printed cube (PLA), removable.** Hold a
`cube_stiff` *loosely* in the gripper. Send `g`. Expect:
`# FORCE_STOP` fires within ~100 ms of contact, no audible gear
stress, `# PEAK_LOAD 0 <N>` shows peak_current at approximately
`CURRENT_LIMIT_GRASP` + a small margin. If you hear ANY gear
strain: yank the cube out, lower `CURRENT_LIMIT_GRASP` by 30,
reflash, repeat from step 5.

After step 6 passes you can do full data collection on stiff
objects without supervision.

### Choosing the threshold from your observations

After running steps 1–5 you'll have peak_current values like:
- Empty peak: ~40
- Foam peak: ~120
- Soft cube peak: ~180

Set `CURRENT_LIMIT_GRASP` to **(highest safe-object peak) + ~30**.
For the example above, ~210 is a good value. Reflash, run step 6,
done.

### Effect on training data

The force fail-safe shifts the data distribution **only for
objects that would have triggered it** — typically just the stiff
cube and possibly the stiff cylinder. Empty, soft cube, soft
cylinder, and `no_object` grasps complete exactly as they did
before because their load never approaches the threshold.

For the affected classes, `enc_t2` will equal `enc_t1` (motor
frozen on contact, no progression during HOLD), and `load_t2` /
`current_t2` will be the *hold-after-freeze* values (lower) rather
than the *push-against-immovable* values (higher) the old model
was trained on.

**Recommended:** collect 1–2 new sessions on the affected classes
with the fail-safe in place, retrain. The old model will likely
still work acceptably for soft/empty grasps in the meantime.

---

## 9. Known open items

- LED patterns assume a single-colour LED. If you swap to RGB,
  switch the mode-indicator scheme from blink-rate to colour
  (and update `ledTick()`).
- The screwing sequence does not handle "user changed mode mid-step"
  cleanly — switching mode forces `screw_step = 0` so the next time
  you re-enter SCREWING you start from GRIP. If you need pause/resume
  semantics, add a "stash step" on mode change.
- The daemon currently has no UI. RESULT lines on stdout are the
  contract; a future UI process can `subprocess.Popen(['python',
  'daemon.py', '--quiet'])` and display per-class images on each
  RESULT line. The class-to-image mapping is alphabetical:
  `cube_soft.png`, `cube_stiff.png`, `cylinder_soft.png`,
  `cylinder_stiff.png`, `no_object.png`, `unknown_object.png`.

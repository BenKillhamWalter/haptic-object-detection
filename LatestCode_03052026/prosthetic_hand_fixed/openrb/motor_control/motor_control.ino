/*
 * ================================================================
 *  MOTOR CONTROL  -  OpenRB-150
 *  File: motor_control.ino
 *  Stage 2: full hand (grip + soft fingers + wrist) with EMG,
 *           mode-cycle button, and LED status indicator.
 * ================================================================
 *
 *  Role
 *  ----
 *  Motor + input service for the split-architecture prosthetic
 *  hand. Owns the Dynamixel bus (3 motors), streams motor state
 *  at 50 Hz, runs grasp / calibration / multi-mode behaviours on
 *  command, and emits event markers that the Python orchestrator
 *  uses for sample alignment and UI feedback.
 *
 *  Hardware
 *  --------
 *      Dynamixel bus (RS485 via OpenRB built-in bridge, 57600 baud,
 *      Protocol 2.0):
 *          ID 1  XM430-W350-R   grip (thumb + index)
 *          ID 2  XC330-T288     ring + little (cable-actuated soft)
 *          ID 3  XC330-T288     wrist (180 deg rotation)
 *      Inputs:
 *          A0    potentiometer (sets EMG threshold)
 *          A1    EMG analog input
 *          D4    push button (INPUT_PULLUP, active-low to GND)
 *          D5    LED          (active-high through 680 ohm to GND)
 *      Power: 12 V wall adapter through OpenRB barrel.
 *
 *  Motor ranges (clamped on every motion command)
 *  ----------------------------------------------
 *      ID 1  OPEN  =  66 deg     CLOSE = 110 deg
 *      ID 2  OPEN  =  30 deg     CLOSE = 260 deg     (cable bind/slack limits)
 *      ID 3  MIN   =   0 deg     MAX   = 180 deg     HOME = 180 deg
 *
 *  Wrist (ID 3) convention
 *  -----------------------
 *      Home = 180 deg. Screwing CW direction = decreasing encoder
 *      (180 -> 0). CCW back to home = increasing (0 -> 180).
 *
 *  Modes (cycled by short button press; LED indicates current mode)
 *  ---------------------------------------------------------------
 *  General actuation semantics (2026-05-20): EMG behaves as a
 *  push button -- only the RISING edge (smoothed crosses threshold
 *  going up) actuates motors. The falling edge is observed and
 *  reported as # EMG_RELEASE but does not drive any motor command.
 *
 *      0  POWER     LED solid              1st flex: close M1+M2
 *                                          2nd flex: open  M1+M2
 *                                          (toggles; EMG release
 *                                           is ignored)
 *      1  TRIPOD    LED 1 Hz blink         1st flex: close M1 only
 *                                          2nd flex: open  M1
 *                                          (M2 stays open throughout)
 *      2  OBJ_RECOG LED 4 Hz blink         flex emits # EMG_FLEX;
 *                                          the Python daemon sends
 *                                          'g' back, which runs the
 *                                          full grasp pipeline incl.
 *                                          the automatic reopen at
 *                                          # GRASP_COMPLETE. No
 *                                          second flex needed.
 *      3  SCREWING  LED double-blink       Tripod grasp + wrist rotation.
 *                                          M2 stays open throughout.
 *                                          On mode entry: pre-position
 *                                            the wrist to 270 deg
 *                                            (M1 + M2 stay open). The
 *                                            user cannot advance the
 *                                            state machine until the
 *                                            pre-position completes --
 *                                            triggers queued during
 *                                            the move are dropped.
 *                                          flex 1: close M1 to 97 deg
 *                                                  (no wrist motion;
 *                                                   the wrist is
 *                                                   already at 270 deg
 *                                                   from mode entry or
 *                                                   from the previous
 *                                                   auto-return)
 *                                          flex 2: rotate wrist
 *                                                  270 -> 30 via 180
 *                                                  (forces 240 deg
 *                                                   long path)
 *                                          flex 3: open M1, start 2-sec
 *                                                  timer
 *                                          (auto): rotate wrist
 *                                                  30 -> 270 via 180
 *                                                  -> ready for next
 *                                                     screwing cycle
 *
 *  EMG processing  (ported from latestWorkingCodeEMG.ino)
 *  ------------------------------------------------------
 *      125 Hz loop: differentiate, rectify, 80-sample moving avg.
 *      Threshold = analogRead(POT) / 4 (10-bit ADC -> 0..255).
 *      Rising edge above threshold      -> EMG_FLEX event
 *      Falling edge below (threshold-H) -> EMG_RELEASE event
 *      Hysteresis H = 8 to avoid bounce around the threshold.
 *      Proportional-PWM servo logic in the reference sketch is
 *      intentionally NOT ported (Dynamixels use discrete goals).
 *
 *  EMG input-source toggle (added 2026-05-22)
 *  ------------------------------------------
 *      A global flag emg_enabled gates EMG-triggered actuation.
 *      Default at boot is OFF (manual-trigger mode) so that EMG
 *      spikes during power-on, electrode swapping, or before the
 *      threshold pot is calibrated cannot drive the hand.
 *
 *      When emg_enabled == false:
 *        * Sampling, smoothing, and the # EMG <ms> ... sample stream
 *          continue normally (UI canvas stays live).
 *        * Rising/falling edges DO NOT emit # EMG_FLEX / # EMG_RELEASE
 *          and DO NOT set emg_flex_pending / emg_release_pending.
 *        * Rising edges emit a low-noise # EMG_EDGE_IGNORED line for
 *          diagnostics; nothing actuates.
 *
 *      When emg_enabled == true: behaviour is identical to the
 *      pre-2026-05-22 firmware.
 *
 *      The button's DOUBLE-press is a manual trigger and IS NOT
 *      gated by emg_enabled (see Button below). Manual actuation is
 *      always available, in any mode, in both states.
 *
 *      Toggle via the 'E' command (E0 / E1 / E?, see Serial protocol)
 *      or via the UI's EMG pill (server.py forwards the command).
 *
 *  Button
 *  ------
 *      30 ms debounce.
 *      Short press (< 400 ms)  -> cycle mode (deferred ~350 ms to
 *                                 disambiguate double-press)
 *      Double press (two short presses within BUTTON_DOUBLE_WINDOW_MS,
 *                    default 350 ms) -> manual trigger; routes into
 *                                       the same internal handler the
 *                                       EMG rising edge uses, i.e.
 *                                       emits # EMG_FLEX and sets
 *                                       emg_flex_pending so modeTick()
 *                                       runs the current mode's
 *                                       actuation exactly as if EMG
 *                                       had fired. Works in both
 *                                       EMG OFF and EMG ON states.
 *      Long  press (>= 1500 ms) -> abort / open all / wrist home
 *
 *  Motor 2 (cable-driven soft fingers) safety
 *  ------------------------------------------
 *      While closing, monitor PRESENT_LOAD. If |load| exceeds
 *      M2_LOAD_LIMIT for M2_LOAD_CONFIRM consecutive samples,
 *      freeze the goal at present position. This prevents:
 *        (a) the cable binding,
 *        (b) finger overload damage,
 *        (c) the position-PID winding up against the obstacle.
 *      Also break out of the close loop once the position error
 *      relative to the goal stops shrinking (catches the case
 *      where load is moderate but motion has stalled).
 *
 *  Serial protocol (115200 baud)
 *  -----------------------------
 *      Commands from Python (single ASCII char unless noted):
 *          p   -> "# pong"
 *          e   -> enumerate all motors (one "# MOTOR ..." line per ID)
 *          s   -> start streaming motor state ("# STREAM_ON")
 *          x   -> stop  streaming             ("# STREAM_OFF")
 *          o   -> M1 open
 *          c   -> M1 close (manual test only; no t1/t2 logic)
 *          h   -> home: open M1, open M2, wrist to HOME
 *          g   -> full grasp pipeline on M1 (open->close->t1->t2->reopen)
 *          r   -> calibration sweep on M1 (slow open->close, no t1/t2)
 *          a   -> abort all motion; torque off all motors
 *          m0..m3  -> force mode 0..3 (overrides button)
 *          w<deg>  -> manual wrist goal (clamped to [0,180]); e.g. w90
 *          f<deg>  -> manual M2 goal    (clamped to [30,260]); e.g. f200
 *          t       -> print "# EMG_STATUS smoothed=.. threshold=.. ..."
 *          j / k   -> start / stop "# EMG <ms> <smoothed> <threshold>"
 *                     periodic stream (~62 Hz). The Python UI server
 *                     (server.py) enables this for the live canvas;
 *                     headless daemon.py leaves it off.
 *          ?       -> print "# DIAG load_limit=.. peak_load_last=.. ..."
 *                     for force fail-safe tuning. See STAGE2_BRINGUP.md
 *                     section "Force fail-safe".
 *          R<id>   -> reboot a motor by ID (R1 / R2 / R3). Use after
 *                     a HARDWARE_ERROR_STATUS shutdown (motor stops
 *                     responding to position commands but still pings).
 *                     Takes ~2.5 s. Emits "# REBOOTING" then "# REBOOTED"
 *                     or "# REBOOT_FAIL".
 *          E0 / E1 -> disable / enable EMG-triggered actuation.
 *                     Boot default is E0 (manual-trigger mode). Sampling
 *                     and the # EMG sample stream are unaffected by
 *                     this command -- only the rising/falling edge
 *                     events and emg_flex_pending are gated. Double-
 *                     press manual triggering is always available
 *                     regardless of this state. Emits "# EMG_INPUT 0"
 *                     or "# EMG_INPUT 1".
 *          E?      -> print current EMG input state ("# EMG_INPUT <n>").
 *          I<id>   -> toggle DRIVE_MODE bit 0 for motor <id> (I2 / I3).
 *                     Use when a motor rotates the wrong direction.
 *                     Persists in EEPROM but setup() forces the
 *                     M<id>_DRIVE_MODE_DEFAULT constant on every boot,
 *                     so the firmware also needs updating to make the
 *                     change permanent across re-flashes.
 *
 *      Stream lines (50 Hz when streaming):
 *          M,<millis>,<enc_deg>,<load>,<current>     (motor 1, unchanged)
 *
 *      Event markers (existing, unchanged in format):
 *          # GRASP_START <ms>
 *          # T1_STALL <ms> <enc>
 *          # STABLE_DETECTED <ms> <enc>
 *          # T2_CAPTURE <ms> <enc> <load> <current>
 *          # GRASP_COMPLETE <ms>
 *          # GRASP_TIMEOUT <ms>
 *          # CALIB_START <ms>  /  # CALIB_COMPLETE <ms>
 *          # ABORTED <ms>
 *          # MOTOR <id> <model_no> ok | none
 *          # STREAM_ON  /  # STREAM_OFF  /  # pong
 *          # ERROR <description>
 *
 *      Event markers (new, additive):
 *          # MODE <n> <name>                 mode changed (or boot)
 *          # EMG_FLEX <ms> <smoothed> <threshold>
 *          # EMG_RELEASE <ms> <smoothed> <threshold>
 *          # WRIST_AT <ms> <enc_deg>          wrist reached goal
 *          # M2_AT    <ms> <enc_deg>          M2 reached goal
 *          # M2_STALL <ms> <enc_deg> <load>   M2 load-safety triggered
 *          # SCREW_STEP <n> <name> <ms>       screwing-mode step started:
 *                                             n=0 GRIP, n=1 CW, n=2 RELEASE
 *          # SCREW_AUTO_RETURN <ms>           screwing-mode automatic
 *                                             wrist-return step fired
 *                                             (~3 s after SCREW_STEP 2)
 *          # REBOOTING ID=<id> at <ms>        a Dynamixel REBOOT
 *                                             instruction was sent
 *          # REBOOTED  ID=<id> at <ms>        reboot succeeded; motor
 *                                             is re-initialised
 *          # REBOOT_FAIL ID=<id> at <ms>      motor didn't come back
 *                                             after reboot
 *          # DRIVE_MODE_RESET ID=<id> was=<old> now=<new>
 *                                             setup() detected DRIVE_MODE
 *                                             didn't match the default and
 *                                             rewrote the EEPROM register
 *          # DRIVE_MODE ID=<id> was=<old> now=<new> ...
 *                                             response to runtime I<id>
 *                                             toggle command
 *          # HOMED <ms>                       long-press home complete
 *          # BUTTON_SHORT <ms>  /  # BUTTON_LONG <ms>
 *          # BUTTON_DOUBLE <ms>               two short presses within
 *                                             BUTTON_DOUBLE_WINDOW_MS;
 *                                             routes to the same path
 *                                             as an EMG rising edge.
 *                                             modeTick() also emits a
 *                                             synthetic # EMG_FLEX
 *                                             line so downstream Python
 *                                             treats it identically.
 *          # EMG_INPUT <0|1>                  EMG-triggered actuation
 *                                             state. Emitted at boot,
 *                                             on every E0/E1 transition,
 *                                             and on E? query.
 *          # EMG_EDGE_IGNORED <ms> <smoothed> <threshold>
 *                                             EMG rising edge detected
 *                                             while emg_enabled=false.
 *                                             Diagnostic only -- no
 *                                             actuation, no pending flag.
 *          # FORCE_STOP <ms> <enc> <load> <current>
 *                                             grip force fail-safe tripped:
 *                                             motor frozen at <enc>, |current|
 *                                             exceeded CURRENT_LIMIT_GRASP.
 *                                             <load> is included for log
 *                                             completeness but reads 0 on
 *                                             XM430 X-series (see constants
 *                                             block). Always paired with a
 *                                             # T1_STALL line for Python
 *                                             pipeline compat.
 *          # PEAK_LOAD <peak_load> <peak_current>
 *                                             diagnostic at GRASP_COMPLETE /
 *                                             CALIB_COMPLETE: max |load| and
 *                                             |current| observed during the
 *                                             grasp window. peak_load reads
 *                                             0 on XM430 X-series; use
 *                                             peak_current for tuning
 *                                             CURRENT_LIMIT_GRASP.
 *          # CALIB_ABORTED <ms>               calibration tripped force
 *                                             fail-safe -- something was in
 *                                             the hand. Profile NOT updated.
 *          # DIAG current_limit=.. current_confirm=.. peak_load_last=.. ...
 *                                             '?' command response.
 *
 *  Required library: Dynamixel2Arduino by ROBOTIS.
 * ================================================================
 */

#include <Dynamixel2Arduino.h>

// ----------------------------------------------------------------
// SCREWING mode feature toggle (added 2026-05-22).
//
// Set to 1 to re-enable SCREWING in the normal mode cycle and via
// the 'm3' command. When 0 (current default), the button-cycle goes
// POWER -> TRIPOD -> OBJ_RECOG -> POWER (skipping SCREWING), and
// 'm3' is rejected with "# ERROR screwing disabled".
//
// All SCREWING state-machine code, constants, comments, and the
// auto-return path remain compiled and intact -- only the entry
// points into the mode are gated. Flip this to 1 and reflash to
// re-enable. Boot also emits "# SCREWING_ENABLED <0|1>" so the UI
// can gray out the SCREWING entry in the mode column accordingly.
// ----------------------------------------------------------------
#define SCREWING_ENABLED 0

// ----------------------------------------------------------------
// WRIST motion feature toggle (added 2026-05-22).
//
// Set to 1 to re-enable automatic wrist motion. When 0 (current
// default), the three regular modes (POWER, TRIPOD, OBJ_RECOG) do
// NOT move the wrist on mode entry, on homing, or at boot. This
// eliminates the ~7 s mode-switch delay caused by waiting for the
// wrist to reach 150 deg when the wrist hardware isn't ready.
//
// What's gated by this toggle:
//   * switchMode() wrist pre-position (any mode)
//   * homeAll() wrist re-home
//   * setup() boot-time wrist parking
//
// What's NOT gated (still functional when WRIST_ENABLED=0):
//   * 'w<deg>' command -- manual user wrist control
//   * 'R3' command     -- wrist reboot
//   * SCREWING wrist sequence -- unreachable while SCREWING_ENABLED=0
//   * Wrist ping / DRIVE_MODE reset at boot -- enumeration only,
//     no motion
//
// The wrist motor stays torque-off when disabled, so it can be
// rotated by hand. Flip to 1 and reflash to re-enable.
// ----------------------------------------------------------------
#define WRIST_ENABLED 0

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = -1;       // OpenRB-150 handles direction internally

// ----------------------------------------------------------------
// Pin assignments (see header comment)
// ----------------------------------------------------------------
const uint8_t PIN_POT    = A0;
const uint8_t PIN_EMG    = A1;
const uint8_t PIN_BUTTON = 4;     // D4, INPUT_PULLUP, active-low
const uint8_t STATUS_LED_PIN    = 5;     // D5, active-high, 680R inline

// ----------------------------------------------------------------
// Dynamixel IDs / bus
// ----------------------------------------------------------------
const uint8_t  MOTOR_ID       = 1;    // grip (XM430-W350-R)
const uint8_t  MOTOR2_ID      = 2;    // soft fingers (XC330-T288)
const uint8_t  MOTOR3_ID      = 3;    // wrist (XC330-T288)
const uint32_t DXL_BAUD       = 57600;
const float    DXL_PROTOCOL   = 2.0;

// ----------------------------------------------------------------
// Motor 1 (grip) -- unchanged from stage 1
// ----------------------------------------------------------------
const float OPEN_POS_DEG = 66.0f;
const float CLOSE_POS_DEG = 110.0f;
const float POWER_TRIPOD_M1_CLOSE_DEG = 115.0f;
const float GRASP_SPEED_RPM = 10.0f;
const float CALIB_SPEED_RPM = 1.0f; // slowed to give XIAO sensor
                                    // stream more samples per bin

// ----------------------------------------------------------------
// Motor 1 (grip) force fail-safe (added 2026-05-20 to prevent gear
// damage on stiff objects).
//
// NOTE on PRESENT_LOAD vs PRESENT_CURRENT (2026-05-20 fix):
// PRESENT_LOAD on the XM430 X-series always reads 0 -- the register
// exists in the control table but is a leftover from the older
// AX/MX-series and is not implemented on X-series. PRESENT_CURRENT
// (signed, units of 2.69 mA each, range -2047..+2047) is the
// reliable torque proxy. The fail-safe therefore monitors current,
// not load. Field PRESENT_LOAD is still tracked into peak_load_last
// for diagnostics, but it will read as 0 on XM430 -- the meaningful
// value in the # PEAK_LOAD <load> <current> line is the second one.
//
// XM430-W350 stall current is approximately 2300 mA = ~855 in
// these units. A safety threshold of 200 = ~540 mA, well below
// stall, gives a comfortable margin against gear strip.
//
// During every cmdGrasp / cmdCalib loop iteration we read
// PRESENT_CURRENT and, if |current| exceeds CURRENT_LIMIT_GRASP
// for CURRENT_CONFIRM consecutive samples (gated by
// CONTACT_IGNORE_MS to avoid the startup torque spike), we freeze
// the motor at its current encoder position via
// gotoDeg(readEncoderDeg()). The motor's internal PID then holds
// with minimal current instead of continuing to push.
//
// IMPORTANT: this is the ONLY mechanism that actually stops the
// motor on contact. Encoder-stall t1 and magnetic-t1 are pure
// observation events; they emit markers but do not change the
// motor goal. Encoder-stall is left observation-only on purpose so
// soft/medium objects can still compress during the 800 ms HOLD
// (preserving training-data distribution). Only objects that
// approach gear-damage current levels trigger the freeze.
//
// Tune empirically per the procedure in STAGE2_BRINGUP.md section
// "Force fail-safe". Verify with the finger-pushback test -- it
// MUST trip the safety before grasping any stiff object.
// ----------------------------------------------------------------
const int16_t  CURRENT_LIMIT_GRASP = 250;   // units of 2.69 mA (~540 mA)
const uint8_t  CURRENT_CONFIRM     = 3;     // consecutive over-limit samples

// ----------------------------------------------------------------
// Motor 2 (soft fingers, cable-actuated)
//
// Safety: monitors PRESENT_CURRENT (not PRESENT_LOAD -- PRESENT_LOAD
// always reads 0 on XC330 X-series, same gotcha as the XM430 on M1).
// XC330-T288's PRESENT_CURRENT unit is 1 mA per LSB, range +/-2047.
// Stall current is ~910 mA, so 300 = 300 mA leaves comfortable margin
// against gear or cable damage while still developing enough grip
// force for the soft fingers to wrap an object firmly.
//
// When |current| > M2_CURRENT_LIMIT for M2_CURRENT_CONFIRM samples,
// the m2Tick controller freezes the motor at its present position
// (gotoDeg(motor2, present_pos)) so the PID holds without escalating.
// ----------------------------------------------------------------
const float    M2_OPEN_DEG        = 100.0f;
const float    M2_CLOSE_DEG       = 260.0f;
const float    M2_SPEED_RPM       = 10.0f;
const int16_t  M2_CURRENT_LIMIT   = 200;    // PRESENT_CURRENT in mA (XC330);
                                            // tune empirically -- raise if
                                            // soft fingers don't wrap the
                                            // object firmly, lower if the
                                            // cable binds or motor warms up.
const uint8_t  M2_CURRENT_CONFIRM = 3;      // consecutive over-limit samples
const float    M2_REACHED_TOL_DEG = 2.0f;
const uint32_t M2_MOTION_TIMEOUT_MS = 4000;

// ----------------------------------------------------------------
// Motor 2 / Motor 3 drive-mode default. Dynamixel X-series stores
// rotation direction in the DRIVE_MODE register (EEPROM, address 10).
// Bit 0 = 0 -> Normal (CW with increasing goal). Bit 0 = 1 -> Reverse.
// If this register flips on its own (e.g. via Dynamixel Wizard or an
// EEPROM glitch), the motor rotates opposite of expectation. setup()
// reads DRIVE_MODE for each alive motor and forces it to the value
// below, so every boot starts in a known direction. The runtime
// 'I<id>' command toggles drive mode for live testing if your
// mechanical assembly needs the inverted polarity.
// ----------------------------------------------------------------
const uint8_t  M2_DRIVE_MODE_DEFAULT = 0;   // 0 = normal; flip if M2 rotates wrong
const uint8_t  M3_DRIVE_MODE_DEFAULT = 0;   // 0 = normal; flip if wrist rotates wrong

// ----------------------------------------------------------------
// Motor 3 (wrist) -- range and home updated 2026-05-20
//
//   Home (all 4 modes, mode-switch rest position):  150 deg
//   Screwing-cycle start (operational position):    270 deg
//   Screwing-cycle end (after rotation):             30 deg
//
// The wrist now operates over 0..360 deg (full single-turn).
// The screwing rotation from 270 to 30 is a 240 deg motion via
// 180 deg. X-series Position Control (mode 3) takes the SHORTEST
// path between two angles by default, which would be the 60 deg
// path via 0/360 -- wrong direction for screwing. The screwing
// sequence therefore commands an intermediate goal of 180 deg
// to force the long 240 deg path through the lower half of the
// rotation envelope.
// ----------------------------------------------------------------
// Wrist mechanical envelope. Tightened 2026-05-20 after the gears
// jammed beyond ~270 deg in an earlier run. All goal positions are
// clamped to [WRIST_MIN_DEG, WRIST_MAX_DEG] in wristMoveTo() and in
// the 'w<deg>' manual command. The operational set (30, 150, 180, 270)
// fits inside this envelope.
const float    WRIST_MIN_DEG          = 70.0f;
const float    WRIST_MAX_DEG          = 195.0f;
const float    WRIST_HOME_DEG         = 150.0f;
const float    SCREW_START_DEG        = 195.0f;
const float    SCREW_END_DEG          = 70.0f;
const float    SCREW_INTERMEDIATE_DEG = 150.0f;   // forces long-path rotation
const float    WRIST_SPEED_RPM        = 20.0f;
const float    WRIST_REACHED_TOL_DEG  = 3.0f;
const uint32_t WRIST_MOTION_TIMEOUT_MS = 5000;    // 240 deg at 20 RPM ~ 2 s;
                                                  // raised from 5 s to give
                                                  // headroom for slower
                                                  // future tuning.

// ----------------------------------------------------------------
// Screwing-mode motor 1 (grip) hard close limit -- partial close.
//
//   In screwing mode M1 closes to 97 deg, not to CLOSE_POS_DEG
//   (110 deg). 97 deg gives a firm pinch on a screwdriver shaft
//   without driving the fingertips together. The full 110 deg
//   close is reserved for object-recognition / power-grasp modes.
// ----------------------------------------------------------------
const float    SCREW_M1_CLOSE_DEG     = 97.0f;

// ----------------------------------------------------------------
// Screwing-mode auto-return delay.
//
//   After the user's third flex (open hand), the firmware waits
//   this long, then automatically rotates the wrist from 30 deg
//   back to 270 deg via 180 deg, so the wrist is positioned for
//   the next screwing cycle. The user does NOT need to flex for
//   this final transition; it is the only step in screwing mode
//   that is not user-driven.
//
//   Tightened 2026-05-22 from 3000 -> 2000 ms per UX request: the
//   user's hand is already open at this point and there is no
//   reason to wait longer than needed for the auto-return.
// ----------------------------------------------------------------
const uint32_t SCREW_AUTO_RETURN_MS   = 2000;

// ----------------------------------------------------------------
// Sampling
// ----------------------------------------------------------------
const uint32_t SAMPLE_INTERVAL_MS = 20;     // 50 Hz motor stream

// ----------------------------------------------------------------
// Encoder-stall t1 (one of two t1 paths; magnetic t1 is in Python).
// ----------------------------------------------------------------
const float    ENCODER_MOVING_MIN_DEG = 1.0f;
const float    ENCODER_STALL_DEG      = 0.3f;
const uint8_t  ENCODER_STALL_CONFIRM  = 4;
const uint32_t CONTACT_IGNORE_MS      = 150;

// Stable detection (encoder went still after t1) and t2 hold.
const float    ENCODER_STILL_DEG   = 0.5f;
const uint8_t  ENCODER_STILL_COUNT = 5;
const uint32_t HOLD_DURATION_MS    = 800;
const uint32_t GRASP_TIMEOUT_MS    = 5000;

// ----------------------------------------------------------------
// EMG processing -- port of latestWorkingCodeEMG.ino smoothing path.
//
// The algorithm matches the reference verbatim:
//   1. Read raw analog every EMG_INTERVAL_MS ms
//   2. Compute diff = raw - last_raw  (signed)
//   3. Rectify: rectified = abs(diff)
//   4. Push rectified into an EMG_WINDOW-sample ring buffer
//   5. Maintain a running sum; smoothed = sum / EMG_WINDOW
//   6. Compare smoothed against threshold (= pot reading / 4)
//
// The only deviation from the reference is the EMG_HYST term. The
// reference does level-based proportional control (close while
// above, open when below) so no hysteresis is needed. We do edge-
// triggered binary detection, so a hysteresis band prevents spurious
// re-triggers when the smoothed signal flickers near the threshold.
// EMG_HYST = 15 means: after a rising edge fires, the smoothed
// value must drop below (threshold - 15) before another rising
// edge can fire. Raise this if you see chattering, lower it if
// release-then-flex-again isn't being detected.
// ----------------------------------------------------------------
const uint8_t  EMG_INTERVAL_MS  = 8;     // ~ 125 Hz (matches reference delay(8))
const uint8_t  EMG_WINDOW       = 80;    // moving-avg window (~ 640 ms; matches reference FILTER_SIZE)
const int      EMG_HYST         = 15;    // release when smoothed < threshold - HYST
                                         // (raised 2026-05-20 from 8 to 15
                                         // for noise rejection on edge-toggle
                                         // modes -- single flex must mean
                                         // single state change)
const int      EMG_MIN_THRESHOLD = 10;   // pot floor; avoid spurious flexes
                                         // when pot is at zero

// ----------------------------------------------------------------
// Button
//
// BUTTON_DOUBLE_WINDOW_MS (added 2026-05-22) is the deferred-fire
// window for short presses. On the release of a short press we wait
// up to this long before committing the press as a "single short".
// If a NEW press starts within the window, it is promoted to a
// "double" instead. This is the only mechanism that lets us
// disambiguate single-press (mode cycle) from double-press (manual
// trigger that routes through the EMG-flex path).
//
// Side effect: every single short press now has ~350 ms added
// latency before the mode actually cycles. Acceptable for UX and
// well below the 1500 ms long-press threshold.
// ----------------------------------------------------------------
const uint32_t BUTTON_DEBOUNCE_MS      = 30;
const uint32_t BUTTON_SHORT_MAX_MS     = 400;
const uint32_t BUTTON_LONG_MIN_MS      = 1500;
const uint32_t BUTTON_DOUBLE_WINDOW_MS = 350;

// ----------------------------------------------------------------
// Modes
// ----------------------------------------------------------------
enum Mode {
  MODE_POWER     = 0,
  MODE_TRIPOD    = 1,
  MODE_OBJ_RECOG = 2,
  MODE_SCREWING  = 3,
  MODE_COUNT     = 4,
};

const char* modeName(Mode m) {
  switch (m) {
    case MODE_POWER:     return "POWER";
    case MODE_TRIPOD:    return "TRIPOD";
    case MODE_OBJ_RECOG: return "OBJ_RECOG";
    case MODE_SCREWING:  return "SCREWING";
    default:             return "?";
  }
}

// ----------------------------------------------------------------
// Globals
// ----------------------------------------------------------------
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

bool      streaming        = false;
bool      motor_alive      = false;   // ID 1
bool      motor2_alive     = false;   // ID 2
bool      motor3_alive     = false;   // ID 3
uint32_t  next_sample_ms   = 0;

// Mode state
Mode      current_mode     = MODE_POWER;
int       screw_step       = 0;       // 0=idle  1=gripped  2=screwed  3=post-open wait
uint32_t  screw_auto_return_at = 0;   // millis() timestamp for the 3-sec
                                      // auto-return to 270 deg; 0 = inactive

// Edge-toggle hand state for POWER / TRIPOD modes. Each EMG flex
// flips the corresponding flag and drives the motors to the new
// state. (EMG_RELEASE is intentionally ignored in these modes -- the
// new semantics are "first flex closes, second flex opens", not
// "flex closes / release opens".)
bool      hand_closed_power   = false;
bool      hand_closed_tripod  = false;

// EMG state
int       emg_buf[EMG_WINDOW];
int       emg_buf_idx      = 0;
long      emg_sum          = 0;
int       emg_last_raw     = 0;
int       emg_smoothed     = 0;
int       emg_threshold    = 0;
bool      emg_buf_ready    = false;
bool      emg_above        = false;
bool      emg_flex_pending = false;
bool      emg_release_pending = false;
uint32_t  emg_next_ms      = 0;

// EMG input-source toggle (added 2026-05-22). When false, emgTick()
// still samples / smooths / streams # EMG sample lines (for the UI
// canvas) but it suppresses # EMG_FLEX / # EMG_RELEASE events and
// does NOT set emg_flex_pending / emg_release_pending. Boot default
// is false (manual-trigger mode); toggled via the E0/E1 command.
//
// The button's double-press path is NOT gated by this flag -- it
// always emits a synthetic # EMG_FLEX and sets emg_flex_pending so
// manual triggering is available in every state.
bool      emg_enabled      = false;

// EMG sample stream (UI use). Off by default; toggled with 'j' / 'k'.
// When on, every other emgTick (~62 Hz) emits:
//   # EMG <ms> <smoothed> <threshold>
// Headless daemon.py does not enable this; only server.py does.
bool      emg_streaming    = false;
uint8_t   emg_stream_count = 0;
const uint8_t EMG_STREAM_DIVISOR = 2;   // 1=125 Hz, 2=62 Hz, 5=25 Hz

// Button state
bool      btn_raw_last     = false;
bool      btn_state        = false;       // debounced "pressed"
uint32_t  btn_last_change_ms = 0;
uint32_t  btn_press_start_ms = 0;
bool      btn_press_handled  = false;
bool      btn_short_pending  = false;
bool      btn_long_pending   = false;
bool      btn_double_pending = false;     // added 2026-05-22

// Deferred-fire short-press disambiguation (added 2026-05-22).
// On the release of a short press we set btn_short_deferred=true
// and remember the release time. buttonTick() then waits up to
// BUTTON_DOUBLE_WINDOW_MS for a second press:
//   * Second press starts within the window  -> promote to double
//     (clear the deferred flag, set btn_double_pending on the
//      next release if also short)
//   * Window expires with no second press    -> commit as single
//     (set btn_short_pending, clear deferred)
bool      btn_short_deferred       = false;
uint32_t  btn_short_deferred_at_ms = 0;
bool      btn_waiting_second_press = false;

// LED state -- driven by ledTick() from millis(); no extra state needed.

// Motor 2 controller state (non-blocking close/open with load safety)
bool      m2_moving        = false;
bool      m2_dir_closing   = false;
uint8_t   m2_stall_count   = 0;
float     m2_goal_deg      = M2_OPEN_DEG;
uint32_t  m2_motion_start_ms = 0;

// Motor 1 (grip) non-blocking force monitor state (added 2026-05-22).
//
// Until this was added, only cmdGrasp() and cmdCalib() force-protected
// M1 -- they have their own monitor inside a blocking while loop. The
// POWER/TRIPOD modes used m1ClosePower()->m1MoveTo()->bare gotoDeg()
// with NO monitor, so a stiff-object grasp from POWER/TRIPOD pushed
// the motor at full current until the gears stripped. Same gap for
// the manual 'c' command and for SCREWING step 0.
//
// m1Tick() (added to pumpInputsIfDue) reads PRESENT_CURRENT every
// loop iteration while m1_close_monitored is true. On overload it
// freezes M1 at the present encoder position -- identical mechanism
// and identical threshold to cmdGrasp's internal monitor, so all
// close paths now stop at the same point with the same force.
//
// m1_close_monitored is set by m1StartClose() (POWER/TRIPOD close,
// cmdClose, optionally screwing step 0) and cleared by:
//   * m1Open()
//   * switchMode() and homeAll() (defensive)
//   * top of cmdGrasp() / cmdCalib() (defensive; those have their
//     own monitor and use bare gotoDeg, so they need m1_close_monitored
//     to be false to avoid a spurious freeze during their initial
//     reopen)
bool      m1_close_monitored = false;
uint8_t   m1_overload_count  = 0;
uint32_t  m1_close_start_ms  = 0;

// Motor 1 (grip) force fail-safe diagnostics. Reset at GRASP_START
// / CALIB_START, updated every monitoring-loop iteration, emitted in
// the # PEAK_LOAD event at GRASP_COMPLETE, and reported by the '?'
// status command. peak_*_last stores the maximum *absolute* value
// observed (PRESENT_LOAD and PRESENT_CURRENT are both signed); we
// store the signed value at the peak so the sign is preserved.
int16_t   peak_load_last    = 0;
int16_t   peak_current_last = 0;

// ----------------------------------------------------------------
// Forward declarations (functions that call each other across
// sections, e.g. wristMoveTo -> homeAll -> wristMoveTo).
// ----------------------------------------------------------------
void pumpInputsIfDue();
void homeAll();
void cmdGrasp();
void m1Tick();   // M1 force monitor, defined after the mode helpers
                 // (added 2026-05-22); needed here because
                 // pumpInputsIfDue() calls it.

// ----------------------------------------------------------------
// Utility
// ----------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ----------------------------------------------------------------
// Low-level motor helpers (ID-parameterised; ID 1 keeps the
// original wrapper signatures for backward compatibility).
// ----------------------------------------------------------------
bool pingMotor(uint8_t id) {
  return dxl.ping(id);
}
bool pingMotor() { return pingMotor(MOTOR_ID); }   // legacy

float readEncoderDeg(uint8_t id) {
  return dxl.getPresentPosition(id, UNIT_DEGREE);
}
float readEncoderDeg() { return readEncoderDeg(MOTOR_ID); }   // legacy

int16_t readLoad(uint8_t id) {
  return dxl.readControlTableItem(PRESENT_LOAD, id);
}
int16_t readLoad() { return readLoad(MOTOR_ID); }   // legacy

int16_t readCurrent(uint8_t id) {
  return dxl.readControlTableItem(PRESENT_CURRENT, id);
}
int16_t readCurrent() { return readCurrent(MOTOR_ID); }   // legacy

void setProfileVelocityRPM(uint8_t id, float rpm) {
  // Profile velocity in 0.229 RPM units for X-series motors.
  uint32_t v = (uint32_t)(rpm / 0.229f);
  if (v < 1) v = 1;
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, v);
}
void setProfileVelocityRPM(float rpm) { setProfileVelocityRPM(MOTOR_ID, rpm); }  // legacy

void torqueOn(uint8_t id) {
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
}
void torqueOn() { torqueOn(MOTOR_ID); }   // legacy

void torqueOff(uint8_t id) {
  dxl.torqueOff(id);
}
void torqueOff() { torqueOff(MOTOR_ID); }   // legacy

void gotoDeg(uint8_t id, float deg) {
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}
void gotoDeg(float deg) { gotoDeg(MOTOR_ID, deg); }   // legacy

// ----------------------------------------------------------------
// Stream emission (motor 1 only; matches stage-1 schema)
// ----------------------------------------------------------------
void emitSample() {
  if (!motor_alive) return;
  Serial.print('M');
  Serial.print(','); Serial.print(millis());
  Serial.print(','); Serial.print(readEncoderDeg(), 2);
  Serial.print(','); Serial.print(readLoad());
  Serial.print(','); Serial.println(readCurrent());
}

void pumpStreamIfDue() {
  if (!streaming) return;
  uint32_t now = millis();
  if ((int32_t)(now - next_sample_ms) < 0) return;
  next_sample_ms += SAMPLE_INTERVAL_MS;
  emitSample();
}

// ----------------------------------------------------------------
// EMG -- read, smooth, edge-detect
// ----------------------------------------------------------------
void emgInit() {
  for (int i = 0; i < EMG_WINDOW; ++i) emg_buf[i] = 0;
  emg_sum          = 0;
  emg_buf_idx      = 0;
  emg_buf_ready    = false;
  emg_above        = false;
  emg_flex_pending = false;
  emg_release_pending = false;
  emg_last_raw     = analogRead(PIN_EMG);
}

void emgTick() {
  uint32_t now = millis();
  if ((int32_t)(now - emg_next_ms) < 0) return;
  emg_next_ms = now + EMG_INTERVAL_MS;

  int raw = analogRead(PIN_EMG);
  int diff = abs(raw - emg_last_raw);
  emg_last_raw = raw;

  // Rolling window sum (O(1) update)
  emg_sum -= emg_buf[emg_buf_idx];
  emg_buf[emg_buf_idx] = diff;
  emg_sum += diff;
  emg_buf_idx++;
  if (emg_buf_idx >= EMG_WINDOW) {
    emg_buf_idx   = 0;
    emg_buf_ready = true;
  }
  emg_smoothed = (int)(emg_sum / EMG_WINDOW);

  // Pot-driven threshold. 10-bit ADC (set in setup) -> divide by 4
  // to land in the original sketch's 0..255 range.
  int pot = analogRead(PIN_POT) >> 2;
  if (pot < EMG_MIN_THRESHOLD) pot = EMG_MIN_THRESHOLD;
  emg_threshold = pot;

  if (!emg_buf_ready) return;   // skip until window is filled

  // Sample stream for the UI canvas. Decimated to ~62 Hz so we don't
  // congest the serial queue or the openrb_evts deque (record_grasp
  // drains that deque, so a 125 Hz fill rate could starve t1/t2
  // events during a long grasp).
  if (emg_streaming) {
    if (++emg_stream_count >= EMG_STREAM_DIVISOR) {
      emg_stream_count = 0;
      Serial.print(F("# EMG "));
      Serial.print(now);             Serial.print(' ');
      Serial.print(emg_smoothed);    Serial.print(' ');
      Serial.println(emg_threshold);
    }
  }

  // Edge detection. When emg_enabled is true (the original behaviour),
  // rising edges emit # EMG_FLEX and set emg_flex_pending so modeTick()
  // actuates the current mode; falling edges emit # EMG_RELEASE for
  // observability. When emg_enabled is false (boot default), edges
  // are still tracked into emg_above so the next genuine state change
  // fires correctly once enabled, but no # EMG_FLEX / # EMG_RELEASE
  // events are emitted and no pending flags are set. Rising edges
  // while disabled emit a # EMG_EDGE_IGNORED diagnostic line so the
  // user can verify the EMG path is still working without arming any
  // actuation. Double-press manual triggering is unaffected by this
  // flag -- see modeTick().
  if (!emg_above && emg_smoothed > emg_threshold) {
    emg_above = true;
    if (emg_enabled) {
      emg_flex_pending = true;
      Serial.print(F("# EMG_FLEX "));
      Serial.print(now);            Serial.print(' ');
      Serial.print(emg_smoothed);   Serial.print(' ');
      Serial.println(emg_threshold);
    }
    // EMG disabled: silently ignore (no event emitted).
  } else if (emg_above && emg_smoothed < (emg_threshold - EMG_HYST)) {
    emg_above = false;
    if (emg_enabled) {
      emg_release_pending = true;
      Serial.print(F("# EMG_RELEASE "));
      Serial.print(now);            Serial.print(' ');
      Serial.print(emg_smoothed);   Serial.print(' ');
      Serial.println(emg_threshold);
    }
  }
}

bool consumeEMGFlex()    { if (emg_flex_pending)    { emg_flex_pending    = false; return true; } return false; }
bool consumeEMGRelease() { if (emg_release_pending) { emg_release_pending = false; return true; } return false; }

void flushEMG() {
  // Discard any edge that fired while we were inside a blocking op
  // (cmdGrasp / cmdCalib / wristMoveTo). Prevents queued flex events
  // from immediately re-triggering action when blocking work returns.
  emg_flex_pending    = false;
  emg_release_pending = false;
  // Re-sync the "above/below" state to current smoothed reading so
  // the next real edge fires correctly.
  emg_above = (emg_smoothed > emg_threshold);
}

// ----------------------------------------------------------------
// Button -- debounce + short / long / double press detection
// ----------------------------------------------------------------
//
// Short press   (< 400 ms)               -> cycle mode (deferred by
//                                            BUTTON_DOUBLE_WINDOW_MS so
//                                            we can distinguish from
//                                            a double press)
// Double press  (two short presses,      -> manual trigger, routes
//                second press starts        through the same path as
//                within the window)         an EMG rising edge
//                                            (see modeTick)
// Long press    (held >= 1500 ms)        -> homeAll / abort
//
// The 400..1500 ms band on a single press is ambiguous and is
// ignored, unchanged from the pre-2026-05-22 firmware.
// ----------------------------------------------------------------
void buttonTick() {
  uint32_t now = millis();
  bool raw = (digitalRead(PIN_BUTTON) == LOW);   // active-low (INPUT_PULLUP)

  if (raw != btn_raw_last) {
    btn_raw_last        = raw;
    btn_last_change_ms  = now;
  }
  if ((now - btn_last_change_ms) < BUTTON_DEBOUNCE_MS) return;

  if (raw != btn_state) {
    btn_state = raw;
    if (raw) {
      // press start
      btn_press_start_ms = now;
      btn_press_handled  = false;
      // If a previous short release is waiting for a possible second
      // press, the start of this press is what marks it a "double".
      // We don't yet know if this press will be short or long; that's
      // decided at release. Mark the "second press in flight" state.
      if (btn_short_deferred &&
          (now - btn_short_deferred_at_ms) <= BUTTON_DOUBLE_WINDOW_MS) {
        btn_waiting_second_press = true;
      }
    } else {
      // release
      if (!btn_press_handled) {
        uint32_t dur = now - btn_press_start_ms;
        if (dur >= BUTTON_LONG_MIN_MS) {
          // Long press completed at release. (Note: the held-long
          // branch below usually fires first; this is the fallback.)
          btn_long_pending = true;
          // Long press supersedes any deferred single waiting to
          // fire, since long-press semantics (homeAll) are stronger.
          btn_short_deferred       = false;
          btn_waiting_second_press = false;
          Serial.print(F("# BUTTON_LONG "));  Serial.println(now);
        } else if (dur < BUTTON_SHORT_MAX_MS) {
          if (btn_waiting_second_press) {
            // This was the SECOND short press of a double-press.
            // Consume the deferred single and emit a double instead.
            btn_short_deferred       = false;
            btn_waiting_second_press = false;
            btn_double_pending       = true;
            Serial.print(F("# BUTTON_DOUBLE ")); Serial.println(now);
          } else {
            // First short press. Defer the commit so we can see if
            // a second press follows within BUTTON_DOUBLE_WINDOW_MS.
            // Do NOT set btn_short_pending yet; that's done by the
            // window-expiry branch below.
            btn_short_deferred       = true;
            btn_short_deferred_at_ms = now;
            Serial.print(F("# BUTTON_SHORT ")); Serial.println(now);
          }
        }
        // 400..1500 ms is the "ambiguous" band -- ignored.
        // If we were waiting for a second press, the ambiguous release
        // clears the wait so the deferred single can still fire at
        // window expiry.
        else {
          btn_waiting_second_press = false;
        }
      }
    }
  } else if (btn_state && !btn_press_handled) {
    // Long press detection while still held -- fires immediately at
    // the threshold rather than waiting for release. Makes the abort
    // feel responsive even mid-grasp.
    if ((now - btn_press_start_ms) >= BUTTON_LONG_MIN_MS) {
      btn_long_pending  = true;
      btn_press_handled = true;
      // Long press supersedes any deferred single. The user is
      // clearly asking for home, not a mode cycle.
      btn_short_deferred       = false;
      btn_waiting_second_press = false;
      Serial.print(F("# BUTTON_LONG "));  Serial.println(now);
    }
  }

  // Commit a deferred short press once the double-press window
  // expires with no qualifying second press. We require !btn_state
  // so an in-progress press (which might be the second one) doesn't
  // trigger premature commit.
  if (btn_short_deferred && !btn_state && !btn_waiting_second_press
      && (now - btn_short_deferred_at_ms) > BUTTON_DOUBLE_WINDOW_MS) {
    btn_short_deferred = false;
    btn_short_pending  = true;
  }
}

bool consumeButtonShort()  { if (btn_short_pending)  { btn_short_pending  = false; return true; } return false; }
bool consumeButtonLong()   { if (btn_long_pending)   { btn_long_pending   = false; return true; } return false; }
bool consumeButtonDouble() { if (btn_double_pending) { btn_double_pending = false; return true; } return false; }

// Drop any queued short / long / double press, and any in-flight
// deferred-single / waiting-second-press state. Added 2026-05-22 for
// the screwing pre-positioning fix: switchMode() and homeAll() both
// call this so a trigger the user issued during the blocking wrist
// move doesn't immediately advance the state machine once the move
// completes.
void flushButtonEvents() {
  btn_short_pending        = false;
  btn_long_pending         = false;
  btn_double_pending       = false;
  btn_short_deferred       = false;
  btn_waiting_second_press = false;
}

// ----------------------------------------------------------------
// LED -- non-blocking pattern driver based on current_mode
// ----------------------------------------------------------------
void ledTick() {
  uint32_t t = millis();
  bool on;
  switch (current_mode) {
    case MODE_POWER:
      on = true;                                       // solid
      break;
    case MODE_TRIPOD:
      on = ((t % 1000) < 500);                          // 1 Hz, 50% duty
      break;
    case MODE_OBJ_RECOG:
      on = ((t % 250) < 125);                           // 4 Hz, 50% duty
      break;
    case MODE_SCREWING: {
      // double-blink: on 100 / off 100 / on 100 / off 700 = 1000 ms total
      uint32_t p = t % 1000;
      on = (p < 100) || (p >= 200 && p < 300);
      break;
    }
    default:
      on = false;
  }
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

// ----------------------------------------------------------------
// Motor 2 controller (non-blocking close with load-stall safety)
// ----------------------------------------------------------------
void m2StartMove(float goal_deg, bool closing) {
  if (!motor2_alive) return;
  goal_deg = clampf(goal_deg, M2_OPEN_DEG, M2_CLOSE_DEG);
  torqueOn(MOTOR2_ID);
  setProfileVelocityRPM(MOTOR2_ID, M2_SPEED_RPM);
  gotoDeg(MOTOR2_ID, goal_deg);
  m2_moving         = true;
  m2_dir_closing    = closing;
  m2_stall_count    = 0;
  m2_goal_deg       = goal_deg;
  m2_motion_start_ms = millis();
}

void m2StartClose() { m2StartMove(M2_CLOSE_DEG, true); }
void m2StartOpen()  { m2StartMove(M2_OPEN_DEG,  false); }

void m2Tick() {
  if (!m2_moving || !motor2_alive) return;

  float pos = readEncoderDeg(MOTOR2_ID);

  // Reached goal?
  if (fabsf(pos - m2_goal_deg) < M2_REACHED_TOL_DEG) {
    m2_moving = false;
    Serial.print(F("# M2_AT "));
    Serial.print(millis()); Serial.print(' ');
    Serial.println(pos, 2);
    return;
  }

  // Timeout?
  if ((millis() - m2_motion_start_ms) > M2_MOTION_TIMEOUT_MS) {
    // Freeze in place; treat as soft failure (no event spam).
    gotoDeg(MOTOR2_ID, pos);
    m2_moving = false;
    Serial.print(F("# M2_AT "));
    Serial.print(millis()); Serial.print(' ');
    Serial.println(pos, 2);
    return;
  }

  // Current-based safety -- only meaningful while closing. Opening
  // lets the cable go slack so current stays low.
  //
  // We use PRESENT_CURRENT (not PRESENT_LOAD) because PRESENT_LOAD
  // reads 0 on XC330 X-series. XC330 PRESENT_CURRENT is signed and
  // already in mA (1 mA per LSB). M2_CURRENT_LIMIT default 300 = 300 mA,
  // well under the XC330's ~910 mA stall current.
  if (m2_dir_closing) {
    int16_t curr = readCurrent(MOTOR2_ID);
    if (abs(curr) > M2_CURRENT_LIMIT) {
      m2_stall_count++;
      if (m2_stall_count >= M2_CURRENT_CONFIRM) {
        // Hold present position. gotoDeg(motor2, pos) keeps the PID
        // actively maintaining grip without driving further into the
        // obstacle -- this is what makes the soft fingers WRAP and
        // stop rather than keep pulling.
        gotoDeg(MOTOR2_ID, pos);
        m2_moving = false;
        Serial.print(F("# M2_STALL "));
        Serial.print(millis()); Serial.print(' ');
        Serial.print(pos, 2);   Serial.print(' ');
        Serial.println(curr);
      }
    } else {
      m2_stall_count = 0;
    }
  }
}

// ----------------------------------------------------------------
// Wrist (motor 3) -- blocking move with stream + input pumping
// ----------------------------------------------------------------
void wristMoveTo(float target_deg) {
  if (!motor3_alive) {
    Serial.println(F("# ERROR wrist not alive"));
    return;
  }
  target_deg = clampf(target_deg, WRIST_MIN_DEG, WRIST_MAX_DEG);
  torqueOn(MOTOR3_ID);
  setProfileVelocityRPM(MOTOR3_ID, WRIST_SPEED_RPM);
  gotoDeg(MOTOR3_ID, target_deg);

  uint32_t deadline = millis() + WRIST_MOTION_TIMEOUT_MS;
  while ((int32_t)(millis() - deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();          // keep EMG/button/LED alive during motion
    if (consumeButtonLong()) {  // allow long-press abort to interrupt
      homeAll();
      return;
    }
    delay(1);
    float pos = readEncoderDeg(MOTOR3_ID);
    if (fabsf(pos - target_deg) < WRIST_REACHED_TOL_DEG) break;
  }

  Serial.print(F("# WRIST_AT "));
  Serial.print(millis()); Serial.print(' ');
  Serial.println(readEncoderDeg(MOTOR3_ID), 2);
}

// ----------------------------------------------------------------
// Input pump -- runs EMG / button / LED ticks at high cadence.
// Safe to call from anywhere; each sub-tick has its own pacing.
// ----------------------------------------------------------------
void pumpInputsIfDue() {
  emgTick();      // self-paced at EMG_INTERVAL_MS
  buttonTick();   // self-paced (debounce internal)
  ledTick();      // cheap; runs every call
  m2Tick();       // non-blocking M2 controller
  m1Tick();       // non-blocking M1 force monitor (POWER/TRIPOD/'c')
}

// ----------------------------------------------------------------
// Mode + sequence logic
// ----------------------------------------------------------------
void emitMode() {
  Serial.print(F("# MODE "));
  Serial.print((int)current_mode); Serial.print(' ');
  Serial.println(modeName(current_mode));
}

void switchMode(Mode m) {
  current_mode = m;
  // Reset all mode-local state so we don't carry stale grip or
  // screw-cycle state across mode changes.
  screw_step           = 0;
  screw_auto_return_at = 0;
  hand_closed_power    = false;
  hand_closed_tripod   = false;
  // Disarm the M1 force monitor before commanding M1 open. Without
  // this, a leftover armed state from a previous POWER/TRIPOD close
  // could let m1Tick fire spuriously during the reopen.
  m1_close_monitored   = false;
  // Drive every motor back to its safe state.
  if (motor_alive)  { torqueOn();             setProfileVelocityRPM(GRASP_SPEED_RPM); gotoDeg(OPEN_POS_DEG); }
  if (motor2_alive) { m2StartOpen(); }
#if WRIST_ENABLED
  if (motor3_alive) {
    // Mode-dependent wrist pre-position. SCREWING parks at
    // SCREW_START_DEG (270 deg) so step 0 only has to close M1 -- the
    // wrist is already in the right place when the user issues the
    // first trigger. All other modes park at WRIST_HOME_DEG (150 deg).
    //
    // wristMoveTo() is blocking, but the flushEMG + flushButtonEvents
    // calls below drop any triggers the user issued during the move,
    // so they can't sneak the state machine ahead.
    float wrist_target = (m == MODE_SCREWING) ? SCREW_START_DEG : WRIST_HOME_DEG;
    wristMoveTo(wrist_target);
  }
#endif
  flushEMG();
  flushButtonEvents();
  emitMode();
}

void cycleMode() {
  Mode next = (Mode)(((int)current_mode + 1) % MODE_COUNT);
#if !SCREWING_ENABLED
  // Skip SCREWING in the cycle. POWER -> TRIPOD -> OBJ_RECOG -> POWER.
  if (next == MODE_SCREWING) next = MODE_POWER;
#endif
  switchMode(next);
}

void homeAll() {
  // Disarm the M1 force monitor before reopening (same reasoning as
  // in switchMode -- don't let a stale armed state freeze the reopen).
  m1_close_monitored   = false;
  if (motor_alive)  { torqueOn();             setProfileVelocityRPM(GRASP_SPEED_RPM); gotoDeg(OPEN_POS_DEG); }
  if (motor2_alive) { m2StartOpen(); }
#if WRIST_ENABLED
  if (motor3_alive) {
    torqueOn(MOTOR3_ID);
    setProfileVelocityRPM(MOTOR3_ID, WRIST_SPEED_RPM);
    gotoDeg(MOTOR3_ID, WRIST_HOME_DEG);   // 150 deg, non-blocking. We don't
                                          // call wristMoveTo() here because
                                          // its blocking loop checks for
                                          // long-press itself and could
                                          // recurse into homeAll().
  }
#endif
  screw_step           = 0;
  screw_auto_return_at = 0;
  hand_closed_power    = false;
  hand_closed_tripod   = false;
  flushEMG();
  flushButtonEvents();
  Serial.print(F("# HOMED ")); Serial.println(millis());
}

// Move M1 to any in-range goal position at GRASP_SPEED_RPM. Non-
// blocking; the motor's internal PID handles the motion. This is the
// raw "set the goal" helper -- no force monitor is armed. Use
// m1StartClose() for any motion that travels TOWARD the hand-closed
// direction, so the m1Tick non-blocking force monitor protects the
// gears against stiff objects.
void m1MoveTo(float deg) {
  if (!motor_alive) return;
  torqueOn();
  setProfileVelocityRPM(GRASP_SPEED_RPM);
  gotoDeg(deg);
}

// Arm the non-blocking M1 force monitor and start moving M1 toward
// the given (closing) goal. Use this for ALL paths that close M1
// outside of cmdGrasp/cmdCalib (which have their own internal
// monitors): POWER/TRIPOD mode close, cmdClose, etc. Uses the same
// threshold / confirm-count / contact-ignore values cmdGrasp uses,
// so the freeze-on-overload behaviour is identical to the 'g' path.
void m1StartClose(float deg) {
  if (!motor_alive) return;
  torqueOn();
  setProfileVelocityRPM(GRASP_SPEED_RPM);
  gotoDeg(deg);
  m1_close_monitored = true;
  m1_overload_count  = 0;
  m1_close_start_ms  = millis();
}

void m1ClosePower() { m1StartClose(POWER_TRIPOD_M1_CLOSE_DEG); }   // 115 deg
void m1Open() {
  // Disarm the monitor BEFORE we command the reopen, so the
  // motor-startup current spike on the reopen does not re-trigger
  // the freeze.
  m1_close_monitored = false;
  m1MoveTo(OPEN_POS_DEG);
}

// Non-blocking M1 force monitor. Runs from pumpInputsIfDue() each
// loop iteration. No-op unless an m1StartClose() arm-call is active.
// On overload -- |PRESENT_CURRENT| > CURRENT_LIMIT_GRASP for
// CURRENT_CONFIRM consecutive samples, gated by CONTACT_IGNORE_MS to
// suppress the motor's startup current spike -- it overwrites the
// goal with the present encoder position so the position PID holds
// at the contact point instead of continuing to push.
//
// Uses the SAME constants the in-cmdGrasp monitor uses (line ~1500),
// so behaviour matches the 'g' command exactly. Emits the same
// # FORCE_STOP wire format so server.py / UI render it identically.
void m1Tick() {
  if (!m1_close_monitored || !motor_alive) return;
  uint32_t now = millis();
  if ((now - m1_close_start_ms) < CONTACT_IGNORE_MS) return;
  int16_t curr = readCurrent();
  if (abs(curr) > CURRENT_LIMIT_GRASP) {
    m1_overload_count++;
    if (m1_overload_count >= CURRENT_CONFIRM) {
      float enc = readEncoderDeg();
      gotoDeg(enc);                  // freeze at present position
      m1_close_monitored = false;    // disarm; one freeze per close
      Serial.print(F("# FORCE_STOP "));
      Serial.print(now);            Serial.print(' ');
      Serial.print(enc, 2);         Serial.print(' ');
      Serial.print((int16_t)0);     Serial.print(' ');  // load placeholder
      Serial.println(curr);
    }
  } else {
    m1_overload_count = 0;
  }
}

void modeTick() {
  // Consume button events first -- they take priority over EMG.
  if (consumeButtonLong()) {
    homeAll();
    return;
  }
  if (consumeButtonShort()) {
    cycleMode();
    return;
  }
  // Double-press is the manual-trigger path. It routes into the same
  // internal handler the EMG rising edge uses -- it emits a synthetic
  // # EMG_FLEX (so server.py / daemon.py treat it identically and the
  // OBJ_RECOG classifier still dispatches) and sets emg_flex_pending
  // so the per-mode switch below runs the same code as a real flex.
  //
  // This path is NOT gated by emg_enabled. Manual triggering is
  // always available in any mode, in both EMG OFF and EMG ON states.
  if (consumeButtonDouble()) {
    Serial.print(F("# EMG_FLEX "));
    Serial.print(millis());         Serial.print(' ');
    Serial.print(emg_smoothed);     Serial.print(' ');
    Serial.println(emg_threshold);
    emg_flex_pending = true;
    // Fall through to the EMG-flex consumer below.
  }

  bool flex    = consumeEMGFlex();
  bool release = consumeEMGRelease();
  (void)release;   // ignored across all modes -- EMG acts as a push
                   // button (rising edge = trigger); falling edge does
                   // not actuate motors. See header "EMG actuation".

  switch (current_mode) {

    // ------------------------------------------------------------
    // POWER -- first flex closes M1 + M2, second flex opens both.
    // EMG_RELEASE has no effect.
    // ------------------------------------------------------------
    case MODE_POWER:
      if (flex) {
        if (!hand_closed_power) {
          m1ClosePower();      // M1 to 110 deg
          m2StartClose();      // M2 toward 260 deg (load-safety guarded)
          hand_closed_power = true;
        } else {
          m1Open();
          m2StartOpen();
          hand_closed_power = false;
        }
      }
      break;

    // ------------------------------------------------------------
    // TRIPOD -- same toggle as POWER but M2 stays open.
    // ------------------------------------------------------------
    case MODE_TRIPOD:
      if (flex) {
        if (!hand_closed_tripod) {
          m1ClosePower();
          hand_closed_tripod = true;
        } else {
          m1Open();
          hand_closed_tripod = false;
        }
      }
      break;

    // ------------------------------------------------------------
    // OBJ_RECOG -- firmware passive. The # EMG_FLEX event was
    // already emitted by emgTick(); the Python daemon owns the
    // classifier dispatch and sends 'g' back over serial, which
    // runs cmdGrasp() (including the automatic reopen at the end
    // of the grasp -- no second flex needed to open).
    // ------------------------------------------------------------
    case MODE_OBJ_RECOG:
      (void)flex;
      break;

    // ------------------------------------------------------------
    // SCREWING -- 3 user-driven steps + 1 automatic step.
    // Rewritten 2026-05-22 to pre-position the wrist at mode entry
    // instead of during step 0 (was causing simultaneous grip-close
    // + wrist-rotate behaviour and adding ~750 ms of perceived
    // trigger latency).
    //
    //   on mode entry              switchMode() pre-positions the
    //                              wrist at SCREW_START_DEG (270 deg)
    //                              with M1 + M2 both open. Triggers
    //                              queued during the blocking move are
    //                              flushed (flushButtonEvents) so the
    //                              user cannot skip ahead into step 1.
    //
    //   step 0 (idle, wrist at     trigger -> close M1 to 97 deg ONLY.
    //           270 deg)                      No wrist motion; the
    //                                         wrist is already at
    //                                         270 deg from mode entry
    //                                         or the previous auto-
    //                                         return. -> step 1
    //   step 1 (gripped)           trigger -> rotate wrist 270 -> 30
    //                                         via 180 (forces 240 deg
    //                                         long path) -> step 2
    //   step 2 (screwed)           trigger -> open M1 (immediately),
    //                                         start 2-sec auto-return
    //                                         timer -> step 3
    //   step 3 (post-open wait)    NO trigger action -- waits for the
    //                              timer to elapse, then auto-rotates
    //                              wrist 30 -> 270 via 180 -> step 0
    //
    // Trigger source: either the EMG rising edge (when EMG actuation
    // is enabled) or the manual button double-press (always available).
    // Both feed emg_flex_pending through the consumer above, so this
    // state machine is driven identically from both inputs.
    //
    // EMG_RELEASE is ignored throughout. Flex events queued during a
    // blocking wristMoveTo are flushed at the end of each step so
    // they don't immediately advance the next step.
    // ------------------------------------------------------------
    case MODE_SCREWING: {

      // Automatic step 3 -> step 0 transition. Runs every modeTick
      // iteration; fires once the 3-sec timer elapses. We return
      // early after firing so a flex event already consumed this
      // tick can't also advance into step 0 in the same iteration.
      if (screw_step == 3 && screw_auto_return_at != 0
          && (int32_t)(millis() - screw_auto_return_at) >= 0) {
        screw_auto_return_at = 0;
        Serial.print(F("# SCREW_AUTO_RETURN "));
        Serial.println(millis());
        wristMoveTo(SCREW_INTERMEDIATE_DEG);              // via 180
        if (current_mode == MODE_SCREWING) {              // bail out if
          wristMoveTo(SCREW_START_DEG);                   // long-press
        }                                                 // homed us
        screw_step = 0;
        flushEMG();
        return;
      }

      // EMG-driven steps. Flex during step 3 (post-open wait) is
      // ignored so the user can't skip the auto-return.
      if (flex && screw_step < 3) {
        switch (screw_step) {
          case 0:
            // GRIP: close M1 to 97 deg only (tripod-style).
            // M2 (soft fingers) deliberately stays open in screwing
            // mode -- the user grips the screwdriver with thumb +
            // index only, like a tripod grasp.
            //
            // The wrist is NOT moved here -- it was already pre-
            // positioned to SCREW_START_DEG (270 deg) when the user
            // cycled into SCREWING (switchMode), or by the previous
            // cycle's auto-return at step 3. Moving the wrist here
            // would block ~750 ms and cause "grip closes while wrist
            // rotates" -- both behaviours we want to avoid.
            //
            // m1MoveTo() is non-blocking, so the next trigger can
            // advance to step 1 (the actual wrist rotation) with
            // minimal added latency on top of the EMG / button input
            // path. Updated 2026-05-22.
            Serial.print(F("# SCREW_STEP 0 GRIP "));
            Serial.println(millis());
            m1MoveTo(SCREW_M1_CLOSE_DEG);                 // 97 deg
            screw_step = 1;
            break;
          case 1:
            Serial.print(F("# SCREW_STEP 1 CW "));
            Serial.println(millis());
            wristMoveTo(SCREW_INTERMEDIATE_DEG);          // via 180
            if (current_mode == MODE_SCREWING) {
              wristMoveTo(SCREW_END_DEG);                 // 30 deg
            }
            screw_step = 2;
            break;
          case 2:
            // RELEASE: open M1 only (M2 was never closed in
            // screwing mode -- tripod-style grip).
            Serial.print(F("# SCREW_STEP 2 RELEASE "));
            Serial.println(millis());
            m1Open();
            screw_step = 3;
            screw_auto_return_at = millis() + SCREW_AUTO_RETURN_MS;
            break;
        }
        flushEMG();
      }
      break;
    }

    default: break;
  }
}

// ----------------------------------------------------------------
// Existing motor-1 commands (preserved)
// ----------------------------------------------------------------
void cmdOpen() {
  if (!motor_alive) { Serial.println(F("# ERROR motor not alive")); return; }
  torqueOn();
  setProfileVelocityRPM(GRASP_SPEED_RPM);
  gotoDeg(OPEN_POS_DEG);
}

void cmdClose() {
  if (!motor_alive) { Serial.println(F("# ERROR motor not alive")); return; }
  // Use m1StartClose so the non-blocking m1Tick monitor protects the
  // gears if 'c' is sent against a stiff object. Matches the safety
  // of the 'g' command's internal monitor. Updated 2026-05-22.
  m1StartClose(CLOSE_POS_DEG);
}

void cmdHome() {
  // Extended in stage 2 to home all three motors, not just M1.
  homeAll();
}

void cmdAbort() {
  torqueOff(MOTOR_ID);
  if (motor2_alive) torqueOff(MOTOR2_ID);
  if (motor3_alive) torqueOff(MOTOR3_ID);
  m2_moving = false;
  Serial.print(F("# ABORTED ")); Serial.println(millis());
}

// Run one complete grasp: open -> close + monitor -> t2 -> reopen.
// Wait loops now pump EMG/button/LED so the UI stays alive and a
// long-press abort can interrupt mid-grasp.
void cmdGrasp() {
  if (!motor_alive) { Serial.println(F("# ERROR motor not alive")); return; }

  // Disarm the non-blocking m1Tick monitor before our own monitor
  // takes over. Without this, a stale m1_close_monitored from a prior
  // POWER/TRIPOD close or a prior 'c' could spuriously freeze us
  // during this initial reopen.
  m1_close_monitored = false;

  // Step 1: ensure open and settled.
  torqueOn();
  setProfileVelocityRPM(GRASP_SPEED_RPM);
  gotoDeg(OPEN_POS_DEG);
  uint32_t wait_until = millis() + 800;
  while ((int32_t)(millis() - wait_until) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    if (consumeButtonLong()) { homeAll(); return; }
  }

  // Step 2: emit GRASP_START and begin closing.
  uint32_t t_start = millis();
  Serial.print(F("# GRASP_START ")); Serial.println(t_start);

  // Reset peak-load diagnostics for this grasp.
  peak_load_last    = 0;
  peak_current_last = 0;

  float enc_at_start = readEncoderDeg();
  gotoDeg(CLOSE_POS_DEG);

  // Step 3: monitor for t1 (encoder stall) and stable detection.
  bool     t1_fired      = false;
  uint8_t  stall_count   = 0;
  uint8_t  still_count   = 0;
  bool     stable_fired  = false;
  uint32_t stable_time   = 0;
  float    last_enc      = enc_at_start;

  // Force fail-safe state -- LOCAL to this grasp invocation. The
  // freeze runs independently of t1_fired so it still triggers even
  // when the encoder-stall path fires first (which is the common
  // case on stiff objects, and was the original gear-breaking bug).
  bool     motor_frozen           = false;
  uint8_t  current_overload_count = 0;

  uint32_t grasp_deadline = t_start + GRASP_TIMEOUT_MS;

  while ((int32_t)(millis() - grasp_deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    if (consumeButtonLong()) { homeAll(); return; }
    delay(1);

    float enc = readEncoderDeg();
    float denc = fabsf(enc - last_enc);

    // Track peak |load| and |current| across the entire grasp window
    // for the # PEAK_LOAD diagnostic (emitted at GRASP_COMPLETE) and
    // the '?' status command.
    int16_t load_now = readLoad();
    int16_t curr_now = readCurrent();
    if (abs(load_now) > abs(peak_load_last))    peak_load_last    = load_now;
    if (abs(curr_now) > abs(peak_current_last)) peak_current_last = curr_now;

    // ---- FORCE FAIL-SAFE (the ONLY mechanism that stops the motor) ----
    // Runs unconditionally every iteration -- NOT gated by t1_fired.
    // Reason: on stiff objects the encoder-stall path fires first,
    // sets t1_fired, and would gate this check off. We need this
    // check to keep running so that when current climbs past the
    // limit (after the motor pushes against the now-stalled
    // obstacle), we still freeze. Gated only by CONTACT_IGNORE_MS to
    // suppress the motor's startup torque spike, and by motor_frozen
    // to avoid re-issuing the freeze every iteration.
    //
    // We use PRESENT_CURRENT (not PRESENT_LOAD) because PRESENT_LOAD
    // always reads 0 on XM430 X-series. See the constants block
    // above for the full explanation.
    if (!motor_frozen) {
      uint32_t since_start = millis() - t_start;
      if (since_start >= CONTACT_IGNORE_MS
          && abs(curr_now) > CURRENT_LIMIT_GRASP) {
        current_overload_count++;
        if (current_overload_count >= CURRENT_CONFIRM) {
          motor_frozen = true;
          // Freeze: set goal to the current encoder position. The
          // XM430's internal PID will hold this position with minimal
          // current instead of continuing to push toward CLOSE_POS_DEG.
          gotoDeg(enc);
          // FORCE_STOP shows the encoder, the (broken) load reading,
          // and the (real) current reading. The current value is what
          // tripped the safety.
          Serial.print(F("# FORCE_STOP "));
          Serial.print(millis()); Serial.print(' ');
          Serial.print(enc, 2);   Serial.print(' ');
          Serial.print(load_now); Serial.print(' ');
          Serial.println(curr_now);
          // Emit T1_STALL too if the encoder-stall path hasn't already
          // fired, so the Python pipeline (grasp.py looks for this
          // event name) registers a valid t1 instead of failing with
          // "no t1 detected" -- this can happen when magnetic-t1 also
          // misses (stiff silicone, soft contact patch).
          if (!t1_fired) {
            t1_fired = true;
            Serial.print(F("# T1_STALL "));
            Serial.print(millis()); Serial.print(' ');
            Serial.println(enc, 2);
          }
        }
      } else {
        current_overload_count = 0;
      }
    }

    // ---- Encoder-stall t1 path (OBSERVATION ONLY -- no freeze) ----
    // Detection unchanged from before. We deliberately do NOT freeze
    // the motor here so soft and medium-stiff objects can compress
    // during the 800 ms HOLD as they always have (preserves the
    // training-data distribution for those classes). The force
    // fail-safe above is the only mechanism that stops the motor.
    if (!t1_fired) {
      uint32_t since_start = millis() - t_start;
      bool moved_enough    = fabsf(enc - enc_at_start) >= ENCODER_MOVING_MIN_DEG;
      bool past_ignore     = since_start >= CONTACT_IGNORE_MS;
      if (moved_enough && past_ignore && denc < ENCODER_STALL_DEG) {
        stall_count++;
        if (stall_count >= ENCODER_STALL_CONFIRM) {
          t1_fired = true;
          Serial.print(F("# T1_STALL "));
          Serial.print(millis()); Serial.print(' ');
          Serial.println(enc, 2);
        }
      } else {
        stall_count = 0;
      }
    }

    // Stable detection: encoder still for N samples after t1.
    if (t1_fired && !stable_fired) {
      if (denc < ENCODER_STILL_DEG) {
        still_count++;
        if (still_count >= ENCODER_STILL_COUNT) {
          stable_fired = true;
          stable_time  = millis();
          Serial.print(F("# STABLE_DETECTED "));
          Serial.print(stable_time); Serial.print(' ');
          Serial.println(enc, 2);
        }
      } else {
        still_count = 0;
      }
    }

    // After stable detection wait HOLD_DURATION_MS and capture t2.
    if (stable_fired) {
      if ((millis() - stable_time) >= HOLD_DURATION_MS) {
        uint32_t t2 = millis();
        float    enc2  = readEncoderDeg();
        int16_t  load2 = readLoad();
        int16_t  cur2  = readCurrent();
        Serial.print(F("# T2_CAPTURE "));
        Serial.print(t2);   Serial.print(' ');
        Serial.print(enc2, 2); Serial.print(' ');
        Serial.print(load2); Serial.print(' ');
        Serial.println(cur2);
        break;
      }
    }

    last_enc = enc;
  }

  if (!stable_fired) {
    Serial.print(F("# GRASP_TIMEOUT ")); Serial.println(millis());
  }

  // Step 4: reopen.
  gotoDeg(OPEN_POS_DEG);
  uint32_t reopen_deadline = millis() + 1000;
  while ((int32_t)(millis() - reopen_deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    if (consumeButtonLong()) { homeAll(); return; }
  }

  // Diagnostic: peak load / current observed during this grasp.
  // Watch this in the serial monitor while tuning -- soft objects
  // should peak well below LOAD_LIMIT_GRASP, stiff objects should
  // peak right at it (because the freeze caps them there).
  Serial.print(F("# PEAK_LOAD "));
  Serial.print(peak_load_last); Serial.print(' ');
  Serial.println(peak_current_last);

  Serial.print(F("# GRASP_COMPLETE ")); Serial.println(millis());
}

// Calibration sweep: slow open -> close on M1, no t1/t2 logic.
void cmdCalib() {
  if (!motor_alive) { Serial.println(F("# ERROR motor not alive")); return; }

  // Disarm the non-blocking m1Tick monitor before cmdCalib's own
  // monitor takes over (same reasoning as cmdGrasp above).
  m1_close_monitored = false;

  torqueOn();
  setProfileVelocityRPM(GRASP_SPEED_RPM);
  gotoDeg(OPEN_POS_DEG);
  uint32_t open_deadline = millis() + 1500;
  while ((int32_t)(millis() - open_deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    if (consumeButtonLong()) { homeAll(); return; }
  }

  Serial.print(F("# CALIB_START ")); Serial.println(millis());
  // Reset peak diagnostics for the calibration sweep.
  peak_load_last    = 0;
  peak_current_last = 0;

  setProfileVelocityRPM(CALIB_SPEED_RPM);
  gotoDeg(CLOSE_POS_DEG);

  uint32_t calib_t_start          = millis();
  uint32_t deadline               = millis() + 30000;
  uint8_t  current_overload_count = 0;
  bool     aborted_on_force       = false;

  while ((int32_t)(millis() - deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    if (consumeButtonLong()) { homeAll(); return; }
    delay(1);
    float enc = readEncoderDeg();

    // Force fail-safe during calibration. Calibration must be run
    // with an empty hand, so any current build-up indicates an
    // object was left in place by mistake. Abort cleanly.
    // Uses PRESENT_CURRENT (not PRESENT_LOAD; see constants block).
    int16_t load_now = readLoad();
    int16_t curr_now = readCurrent();
    if (abs(load_now) > abs(peak_load_last))    peak_load_last    = load_now;
    if (abs(curr_now) > abs(peak_current_last)) peak_current_last = curr_now;

    uint32_t since_start = millis() - calib_t_start;
    if (since_start >= CONTACT_IGNORE_MS
        && abs(curr_now) > CURRENT_LIMIT_GRASP) {
      current_overload_count++;
      if (current_overload_count >= CURRENT_CONFIRM) {
        // Freeze, abort, reopen. Don't emit CALIB_COMPLETE because
        // the profile is invalid (something was in the way).
        gotoDeg(enc);
        Serial.print(F("# FORCE_STOP "));
        Serial.print(millis()); Serial.print(' ');
        Serial.print(enc, 2);   Serial.print(' ');
        Serial.print(load_now); Serial.print(' ');
        Serial.println(curr_now);
        Serial.print(F("# CALIB_ABORTED ")); Serial.println(millis());
        aborted_on_force = true;
        break;
      }
    } else {
      current_overload_count = 0;
    }

    if (fabsf(enc - CLOSE_POS_DEG) < 0.5f) break;
  }

  setProfileVelocityRPM(GRASP_SPEED_RPM);
  gotoDeg(OPEN_POS_DEG);
  uint32_t reopen_deadline = millis() + 2000;
  while ((int32_t)(millis() - reopen_deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    if (consumeButtonLong()) { homeAll(); return; }
  }

  // Diagnostic peak for the sweep (useful when verifying the sweep
  // is truly empty -- empty calibration should peak well under 50).
  Serial.print(F("# PEAK_LOAD "));
  Serial.print(peak_load_last); Serial.print(' ');
  Serial.println(peak_current_last);

  if (!aborted_on_force) {
    Serial.print(F("# CALIB_COMPLETE ")); Serial.println(millis());
  }
}

// ----------------------------------------------------------------
// Command parsing helpers (for multi-char commands like m1, w90, f200)
// ----------------------------------------------------------------
// Read up to a small number of digits/sign/dot from Serial with a
// short timeout. Used for the numeric tail of m/w/f.
bool readNumberArg(float& out) {
  char buf[12];
  size_t n = 0;
  uint32_t deadline = millis() + 30;
  while (n < sizeof(buf) - 1 && millis() < deadline) {
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == '-' || c == '.' || (c >= '0' && c <= '9')) {
        buf[n++] = (char)Serial.read();
        deadline = millis() + 30;   // got a digit, extend timeout
      } else {
        break;
      }
    }
  }
  if (n == 0) return false;
  buf[n] = '\0';
  out = atof(buf);
  return true;
}

// ----------------------------------------------------------------
// Motor recovery helpers
// ----------------------------------------------------------------
//
// Dynamixel motors shut down when HARDWARE_ERROR_STATUS sets a bit
// (overload, overheat, encoder error, etc). The motor stays powered
// but refuses to apply torque until it's rebooted or until the
// HARDWARE_ERROR_STATUS is cleared. The cleanest recovery is the
// REBOOT instruction, which dxl.reboot(id) sends. Reboot takes ~1 s
// and resets EVERY runtime register to defaults (operating mode,
// torque, profile velocity, goal position), so the motor needs to
// be fully re-initialised afterwards.
void rebootMotor(uint8_t id, bool* alive_flag) {
  Serial.print(F("# REBOOTING ID=")); Serial.print(id);
  Serial.print(F(" at ")); Serial.println(millis());
  // Best-effort: try torque-off so the reboot is clean, ignore errors.
  dxl.torqueOff(id);
  delay(20);
  dxl.reboot(id);
  // Reboot takes around 800-1500 ms on X-series. Wait, then re-ping.
  uint32_t deadline = millis() + 2500;
  while ((int32_t)(millis() - deadline) < 0) {
    pumpStreamIfDue();
    pumpInputsIfDue();
    delay(10);
  }
  bool alive = dxl.ping(id);
  if (alive_flag) *alive_flag = alive;
  if (!alive) {
    Serial.print(F("# REBOOT_FAIL ID=")); Serial.print(id);
    Serial.print(F(" at ")); Serial.println(millis());
    return;
  }
  // Re-init: set operating mode, restore drive mode, torque on, park.
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  if (id == MOTOR2_ID) {
    dxl.writeControlTableItem(DRIVE_MODE, id, M2_DRIVE_MODE_DEFAULT);
  } else if (id == MOTOR3_ID) {
    dxl.writeControlTableItem(DRIVE_MODE, id, M3_DRIVE_MODE_DEFAULT);
  }
  delay(30);
  dxl.torqueOn(id);
  // Send the motor back to its mode-appropriate safe position.
  if (id == MOTOR_ID) {
    setProfileVelocityRPM(id, GRASP_SPEED_RPM);
    gotoDeg(id, OPEN_POS_DEG);
  } else if (id == MOTOR2_ID) {
    setProfileVelocityRPM(id, M2_SPEED_RPM);
    gotoDeg(id, M2_OPEN_DEG);
    m2_moving = false;
  } else if (id == MOTOR3_ID) {
    setProfileVelocityRPM(id, WRIST_SPEED_RPM);
    gotoDeg(id, WRIST_HOME_DEG);
  }
  Serial.print(F("# REBOOTED ID=")); Serial.print(id);
  Serial.print(F(" at ")); Serial.println(millis());
}

// Toggle DRIVE_MODE bit 0 for a motor (0 -> 1, or 1 -> 0). Used to
// fix the "motor rotates opposite direction" situation at runtime
// without recompiling. The new mode persists in EEPROM, so the
// change survives a power cycle -- but setup() also forces the
// default at boot, so if you want the change to be permanent you
// must also update M2_DRIVE_MODE_DEFAULT / M3_DRIVE_MODE_DEFAULT in
// the firmware constants.
void toggleDriveMode(uint8_t id) {
  uint8_t cur = (uint8_t)dxl.readControlTableItem(DRIVE_MODE, id);
  uint8_t new_mode = (cur & 0x01) ? (cur & ~0x01) : (cur | 0x01);
  dxl.torqueOff(id);
  delay(20);
  dxl.writeControlTableItem(DRIVE_MODE, id, new_mode);
  delay(30);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  Serial.print(F("# DRIVE_MODE ID="));      Serial.print(id);
  Serial.print(F(" was="));                 Serial.print(cur);
  Serial.print(F(" now="));                 Serial.print(new_mode);
  Serial.print(F(" -- update M"));          Serial.print(id);
  Serial.println(F("_DRIVE_MODE_DEFAULT in firmware to persist."));
}

// ----------------------------------------------------------------
// Command dispatch
// ----------------------------------------------------------------
void emitMotorEnum(uint8_t id, bool alive) {
  Serial.print(F("# MOTOR "));
  Serial.print(id); Serial.print(' ');
  if (alive) {
    Serial.print(dxl.getModelNumber(id)); Serial.print(' ');
    Serial.println(F("ok"));
  } else {
    Serial.println(F("none"));
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'p': Serial.println(F("# pong")); break;

    case 'e':
      emitMotorEnum(MOTOR_ID,  motor_alive);
      emitMotorEnum(MOTOR2_ID, motor2_alive);
      emitMotorEnum(MOTOR3_ID, motor3_alive);
      break;

    case 's':
      streaming      = true;
      next_sample_ms = millis();
      Serial.println(F("# STREAM_ON"));
      break;
    case 'x':
      streaming = false;
      Serial.println(F("# STREAM_OFF"));
      break;

    case 'o': cmdOpen();  break;
    case 'c': cmdClose(); break;
    case 'h': cmdHome();  break;
    case 'g': cmdGrasp(); break;
    case 'r': cmdCalib(); break;
    case 'a': cmdAbort(); break;

    case 't':
      // EMG/pot status snapshot. Useful for verifying which way to
      // rotate the pot for max threshold, and for tuning EMG without
      // having to actually flex.
      Serial.print(F("# EMG_STATUS smoothed="));
      Serial.print(emg_smoothed);
      Serial.print(F(" threshold="));
      Serial.print(emg_threshold);
      Serial.print(F(" pot_raw="));
      Serial.print(analogRead(PIN_POT));
      Serial.print(F(" emg_raw="));
      Serial.print(analogRead(PIN_EMG));
      Serial.print(F(" above="));
      Serial.println(emg_above ? 1 : 0);
      break;

    case 'j':
      // Start EMG sample stream for the UI canvas.
      emg_streaming    = true;
      emg_stream_count = 0;
      Serial.println(F("# EMG_STREAM_ON"));
      break;
    case 'k':
      emg_streaming = false;
      Serial.println(F("# EMG_STREAM_OFF"));
      break;

    case '?':
      // Force-fail-safe status snapshot. Prints the live threshold
      // values and the peaks observed during the most recent grasp
      // or calibration sweep. Used for threshold tuning without
      // recompiling -- run a test grasp, then '?' to see the peak.
      //
      // Note: peak_load_last will be 0 on XM430 X-series motors
      // because PRESENT_LOAD isn't implemented there. The meaningful
      // value for tuning CURRENT_LIMIT_GRASP is peak_current_last.
      Serial.print(F("# DIAG current_limit="));   Serial.print(CURRENT_LIMIT_GRASP);
      Serial.print(F(" current_confirm="));       Serial.print(CURRENT_CONFIRM);
      Serial.print(F(" peak_load_last="));        Serial.print(peak_load_last);
      Serial.print(F(" peak_current_last="));     Serial.println(peak_current_last);
      break;

    case 'R': {
      // Reboot a motor by ID. Recovers from HARDWARE_ERROR_STATUS
      // (overload/overheat shutdown) without a full power cycle.
      // Usage: R1, R2, R3. Takes ~2.5 s to complete.
      float v;
      if (!readNumberArg(v)) {
        Serial.println(F("# ERROR R requires motor id (1, 2, or 3)"));
        break;
      }
      int id = (int)v;
      if (id == MOTOR_ID)       rebootMotor(MOTOR_ID,  &motor_alive);
      else if (id == MOTOR2_ID) rebootMotor(MOTOR2_ID, &motor2_alive);
      else if (id == MOTOR3_ID) rebootMotor(MOTOR3_ID, &motor3_alive);
      else Serial.println(F("# ERROR R: unknown motor id"));
      break;
    }

    case 'I': {
      // Toggle DRIVE_MODE bit 0 for a motor at runtime. Used to fix
      // "motor rotates the wrong direction" without recompiling.
      // Usage: I2 toggles M2 direction, I3 toggles M3 direction.
      // Persists in EEPROM; also update M<n>_DRIVE_MODE_DEFAULT in
      // the firmware so the change survives a re-flash.
      float v;
      if (!readNumberArg(v)) {
        Serial.println(F("# ERROR I requires motor id (2 or 3)"));
        break;
      }
      int id = (int)v;
      if (id == MOTOR2_ID && motor2_alive)      toggleDriveMode(MOTOR2_ID);
      else if (id == MOTOR3_ID && motor3_alive) toggleDriveMode(MOTOR3_ID);
      else Serial.println(F("# ERROR I: motor not alive or unsupported"));
      break;
    }

    case 'E': {
      // EMG input-source toggle (added 2026-05-22).
      //   E0 -> disable EMG-triggered actuation (manual-trigger mode)
      //   E1 -> enable  EMG-triggered actuation
      //   E? -> print current state
      // Always emits "# EMG_INPUT <0|1>" so the server / UI stay synced.
      //
      // Sampling and the # EMG sample stream are unaffected by this
      // command. Only edge events (# EMG_FLEX / # EMG_RELEASE) and the
      // emg_flex_pending / emg_release_pending flags are gated.
      //
      // The button's double-press manual trigger is NOT gated by this
      // command -- manual triggering is always available.
      uint32_t deadline = millis() + 30;
      char arg = 0;
      while (millis() < deadline) {
        if (Serial.available()) {
          char c = Serial.peek();
          if (c == '0' || c == '1' || c == '?') {
            arg = (char)Serial.read();
            break;
          } else {
            break;   // unknown trailing char -- leave it for next dispatch
          }
        }
      }
      if (arg == '0') {
        emg_enabled = false;
        flushEMG();   // drop any flag that was set before the toggle
      } else if (arg == '1') {
        emg_enabled = true;
        flushEMG();   // start clean; require a fresh rising edge
      } else if (arg == '?') {
        // query only -- no state change
      } else {
        Serial.println(F("# ERROR E requires 0, 1, or ?"));
        break;
      }
      Serial.print(F("# EMG_INPUT "));
      Serial.println(emg_enabled ? 1 : 0);
      break;
    }

    case 'm': {
      float v;
      if (!readNumberArg(v)) {
        Serial.println(F("# ERROR m requires mode index 0..3"));
        break;
      }
      int idx = (int)v;
      if (idx < 0 || idx >= MODE_COUNT) {
        Serial.println(F("# ERROR mode out of range"));
        break;
      }
#if !SCREWING_ENABLED
      if (idx == MODE_SCREWING) {
        Serial.println(F("# ERROR screwing disabled"));
        break;
      }
#endif
      switchMode((Mode)idx);
      break;
    }

    case 'w': {
      float deg;
      if (!readNumberArg(deg)) {
        Serial.println(F("# ERROR w requires angle in deg"));
        break;
      }
      wristMoveTo(deg);   // clamps internally
      break;
    }

    case 'f': {
      float deg;
      if (!readNumberArg(deg)) {
        Serial.println(F("# ERROR f requires angle in deg"));
        break;
      }
      m2StartMove(deg, deg > readEncoderDeg(MOTOR2_ID));
      break;
    }

    case '\n': case '\r': case ' ': case '\t': break;
    default:
      Serial.print(F("# ERROR unknown command '"));
      Serial.print(c); Serial.println(F("'"));
      break;
  }
}

// ----------------------------------------------------------------
// SETUP / LOOP
// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {}

  Serial.println(F("# ============================================="));
  Serial.println(F("# motor_control.ino  -  OpenRB-150  (stage 2)"));
  Serial.println(F("# Grip ID1 + Soft ID2 + Wrist ID3 + EMG + button + LED"));
  Serial.println(F("# ============================================="));

  // GPIO + ADC
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN,    OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  analogReadResolution(10);   // keep pot/EMG in the 0..1023 range that
                              // matches the original EMG reference sketch

  emgInit();

  // Dynamixel bus
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  delay(50);

  // Probe each motor independently. Missing motors are tolerated --
  // their handlers no-op so the rest of the system stays usable.
  motor_alive  = pingMotor(MOTOR_ID);
  motor2_alive = pingMotor(MOTOR2_ID);
  motor3_alive = pingMotor(MOTOR3_ID);

  if (motor_alive) {
    Serial.print(F("# MOTOR found ID=")); Serial.print(MOTOR_ID);
    Serial.print(F(" model="));           Serial.println(dxl.getModelNumber(MOTOR_ID));
    dxl.torqueOff(MOTOR_ID);
    dxl.setOperatingMode(MOTOR_ID, OP_POSITION);
  } else {
    Serial.println(F("# ERROR motor 1 (grip) not responding at boot."));
    Serial.println(F("# Check RS485/TTL bridge wiring and 12 V supply."));
  }
  if (motor2_alive) {
    Serial.print(F("# MOTOR found ID=")); Serial.print(MOTOR2_ID);
    Serial.print(F(" model="));           Serial.println(dxl.getModelNumber(MOTOR2_ID));
    dxl.torqueOff(MOTOR2_ID);
    // Force DRIVE_MODE to the configured default so we get the same
    // rotation direction every boot regardless of what previously
    // wrote to this EEPROM register. Only writes if the current
    // value differs (EEPROM wear protection).
    uint8_t m2_drive = (uint8_t)dxl.readControlTableItem(DRIVE_MODE, MOTOR2_ID);
    if (m2_drive != M2_DRIVE_MODE_DEFAULT) {
      dxl.writeControlTableItem(DRIVE_MODE, MOTOR2_ID, M2_DRIVE_MODE_DEFAULT);
      delay(30);
      Serial.print(F("# DRIVE_MODE_RESET ID=")); Serial.print(MOTOR2_ID);
      Serial.print(F(" was="));                  Serial.print(m2_drive);
      Serial.print(F(" now="));                  Serial.println(M2_DRIVE_MODE_DEFAULT);
    }
    dxl.setOperatingMode(MOTOR2_ID, OP_POSITION);
  } else {
    Serial.println(F("# WARN motor 2 (soft fingers) not responding."));
  }
  if (motor3_alive) {
    Serial.print(F("# MOTOR found ID=")); Serial.print(MOTOR3_ID);
    Serial.print(F(" model="));           Serial.println(dxl.getModelNumber(MOTOR3_ID));
    dxl.torqueOff(MOTOR3_ID);
    // Same drive-mode forcing as M2.
    uint8_t m3_drive = (uint8_t)dxl.readControlTableItem(DRIVE_MODE, MOTOR3_ID);
    if (m3_drive != M3_DRIVE_MODE_DEFAULT) {
      dxl.writeControlTableItem(DRIVE_MODE, MOTOR3_ID, M3_DRIVE_MODE_DEFAULT);
      delay(30);
      Serial.print(F("# DRIVE_MODE_RESET ID=")); Serial.print(MOTOR3_ID);
      Serial.print(F(" was="));                  Serial.print(m3_drive);
      Serial.print(F(" now="));                  Serial.println(M3_DRIVE_MODE_DEFAULT);
    }
    dxl.setOperatingMode(MOTOR3_ID, OP_POSITION);
#if WRIST_ENABLED
    // Park wrist at home position on boot.
    torqueOn(MOTOR3_ID);
    setProfileVelocityRPM(MOTOR3_ID, WRIST_SPEED_RPM);
    gotoDeg(MOTOR3_ID, WRIST_HOME_DEG);
#endif
    // WRIST_ENABLED=0: motor stays torque-off so the wrist can be
    // rotated by hand. 'w<deg>' is still functional for explicit
    // user control if you want to power-test the wrist.
  } else {
    Serial.println(F("# WARN motor 3 (wrist) not responding."));
  }

  // Boot announce current mode so the laptop daemon can sync.
  emitMode();

  // Boot announce EMG input state. Always 0 (manual-trigger mode) at
  // power-on so EMG spikes during startup / electrode swapping / before
  // pot calibration cannot drive the hand. server.py captures this
  // event and broadcasts it to the UI so the EMG pill renders OFF.
  Serial.print(F("# EMG_INPUT "));
  Serial.println(emg_enabled ? 1 : 0);

  // Boot announce SCREWING feature toggle so the UI knows whether to
  // expose the SCREWING entry in the mode column. Source of truth is
  // the SCREWING_ENABLED define at the top of this file.
  Serial.print(F("# SCREWING_ENABLED "));
  Serial.println(SCREWING_ENABLED ? 1 : 0);
}

void loop() {
  // Drain incoming commands.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;
    handleCommand(c);
  }
  // Periodic background work.
  pumpStreamIfDue();
  pumpInputsIfDue();
  // Run per-mode action handler. This is what turns EMG/button edges
  // into motor commands.
  modeTick();
}

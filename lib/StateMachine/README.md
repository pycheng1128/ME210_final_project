# StateMachine — Robot FSM for ME210 Final Project

This library implements a **non-blocking finite state machine (FSM)** that controls the robot's full competition sequence: loading, wall-alignment, line-following, launching, and returning to the end zone.

---

## State Diagram

```
                          ┌──────────────────┐
                          │  IDLE_FOR_LOADING │◄──────────────────────┐
                          │  (wait 5 s)       │                       │
                          └────────┬─────────┘                       │
                                   │ timer expires                   │
                                   ▼                                 │
                          ┌──────────────────────┐                   │
                          │ TURN_ALIGN_TO_LEFT   │                   │
                          │ WALL                  │                   │
                          └────────┬─────────────┘                   │
                                   │ aligned + front clear           │
                                   ▼                                 │
                          ┌──────────────────────┐                   │
                          │ MOVE_FORWARD_AFTER   │                   │
                          │ ALIGN (timed fwd)    │                   │
                          └────────┬─────────────┘                   │
                                   │ timer expires                   │
                                   ▼                                 │
                          ┌──────────────────────┐                   │
                          │ SHIFT_RIGHT_TO       │                   │
                          │ CENTER_LINE          │                   │
                          └────────┬─────────────┘                   │
                                   │ line sensor centered            │
                                   ▼                                 │
                          ┌──────────────────────┐                   │
                          │ MOVE_FORWARD_TO      │                   │
                          │ HOG_LINE             │                   │
                          └────────┬─────────────┘                   │
                                   │ all 5 line sensors = black      │
                                   ▼                                 │
                          ┌──────────────────────┐                   │
                          │ LAUNCH               │                   │
                          │ (stepper one-shot)   │                   │
                          └────────┬─────────────┘                   │
                                   │ stepper cycle complete          │
                                   ▼                                 │
                          ┌──────────────────────┐                   │
                          │ RETURN_TO_END_ZONE   │───────────────────┘
                          │ (3 sub-states)       │  shift-left done
                          └──────────────────────┘

      Any state ──(timeout)──► FAULT
      Any state ──(E-stop)───► FAULT  (requires 'r' to recover)
      FAULT ─────('r' key)───► IDLE_FOR_LOADING
```

---

## States in Detail

### 1. `IDLE_FOR_LOADING`

| Item | Detail |
|------|--------|
| **Purpose** | Allow the operator to load the projectile. |
| **Actions** | Stop all motors (drivetrain + stepper). |
| **Transition** | After `FSM_LOAD_IDLE_MS` (5 s) → `TURN_ALIGN_TO_LEFT_WALL` |

### 2. `TURN_ALIGN_TO_LEFT_WALL`

| Item | Detail |
|------|--------|
| **Purpose** | Rotate until the robot is parallel to the left wall. |
| **Sensors** | Two left-side ultrasonic sensors (front + back). |
| **Logic** | Compute `left_diff = uss_front − uss_back`. Rotate CW or CCW to drive diff → 0. If the front USS is blocked, rotate CW to clear. |
| **Transition** | `|left_diff| ≤ FSM_PARALLEL_TOLERANCE_CM` **and** front clear → `MOVE_FORWARD_AFTER_ALIGN` |
| **Fault** | N consecutive invalid USS reads (`FSM_USS_FAULT_MAX_CONSEC`) → `FAULT` |

### 3. `MOVE_FORWARD_AFTER_ALIGN`

| Item | Detail |
|------|--------|
| **Purpose** | Drive forward for a fixed duration after wall alignment before searching for the center line. |
| **Actions** | Drive forward at `FSM_FORWARD_AFTER_ALIGN_RPM`. |
| **Transition** | After `FSM_FORWARD_AFTER_ALIGN_MS` (2 s) → `SHIFT_RIGHT_TO_CENTER_LINE` |

### 4. `SHIFT_RIGHT_TO_CENTER_LINE`

| Item | Detail |
|------|--------|
| **Purpose** | Strafe right until the line sensors detect the center line. |
| **Sensors** | 5-bit line sensor mask. |
| **Logic** | Strafe right at `FSM_SHIFT_RIGHT_RPM`. |
| **Transition** | Line mask = `00100` or `01110` (centered) → `MOVE_FORWARD_TO_HOG_LINE` |

### 5. `MOVE_FORWARD_TO_HOG_LINE`

| Item | Detail |
|------|--------|
| **Purpose** | Drive forward along the center line until the hog line is reached. |
| **Sensors** | 5-bit line sensor mask for lateral correction. |
| **Logic** | Drive forward at `FSM_FORWARD_TO_HOG_RPM` with strafe correction from `lineFollowStrafeCorrectionRpm()`. |
| **Transition** | Line mask = `11111` (all sensors see black = hog line) → `LAUNCH` |

### 6. `LAUNCH`

| Item | Detail |
|------|--------|
| **Purpose** | Fire the stepper motor to launch the projectile. |
| **Actions** | 1. Stop drivetrain. 2. Start stepper launch cycle **once** (one-shot guard). 3. Call `updateStepperMotor()` each loop to advance. |
| **Transition** | `checkStepperLaunchCycleComplete()` returns `true` → `RETURN_TO_END_ZONE` |

### 7. `RETURN_TO_END_ZONE`

This state uses **three non-blocking sub-states**:

| Sub-State | Action | Exit Condition |
|-----------|--------|----------------|
| `FOLLOW_LINE` | Line-follow forward at `FSM_RETURN_LINE_FOLLOW_RPM` | Front USS ≤ `FSM_RETURN_FRONT_TARGET_CM` |
| `TURN_180` | Timed CW rotation at `FSM_RETURN_TURN_RPM` | `FSM_RETURN_TURN_180_MS` elapsed |
| `SHIFT_LEFT` | Timed left strafe at `FSM_RETURN_SHIFT_LEFT_RPM` | `FSM_RETURN_SHIFT_LEFT_30CM_MS` elapsed → `IDLE_FOR_LOADING` |

A `default` case in the sub-state switch transitions to `FAULT` for defensive safety.

### 8. `FAULT`

| Item | Detail |
|------|--------|
| **Purpose** | Safety latch — halts all actuators. |
| **Entry** | Any state timeout (`FSM_STATE_TIMEOUT_MS`), excessive USS invalid reads, E-Stop activation, or corrupted sub-state. |
| **Recovery** | Send `'r'` over serial (while E-stop is released) → `IDLE_FOR_LOADING` |

---

## Global Guards

These are checked **every loop iteration** before any state handler runs:

1. **E-Stop** (`Mobility_IsEStopped()`): Immediately stops all motors (drivetrain **and** stepper) and latches into `FAULT`. The robot will **not** auto-resume; the operator must send `'r'` over serial after the E-Stop is released.
2. **State Timeout** (`FSM_STATE_TIMEOUT_MS = 12 s`): If any single state runs longer than this, transition to `FAULT`.

### E-Stop Safety Chain

The E-Stop provides **two layers** of protection:

| Layer | Location | Response Time | Action |
|-------|----------|---------------|--------|
| **Hardware** | `checkEStop()` in `mobility_driver.cpp` | Every `Mobility_Update()` call | Immediately kills DC motors **and** stepper motor |
| **FSM** | `updateStateMachine()` E-Stop check | Next FSM loop iteration | Latches state to `FAULT`, requires `'r'` to recover |

---

## File Structure

```
lib/StateMachine/
├── README.md                  ← this file
└── src/
    ├── state_machine.h        ← public API (initStateMachine, updateStateMachine)
    ├── state_machine.cpp      ← FSM implementation + per-state handler functions
    └── state_machine_config.h ← all tunable parameters (speeds, timeouts, thresholds)
```

---

## Tuning Parameters

All FSM-specific parameters live in `state_machine_config.h`. Global parameters (PID gains, USS threshold, FSM timing) live in `include/config.h`.

### Timing (in `config.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `FSM_LOAD_IDLE_MS` | `5000` | How long to wait in IDLE before starting |
| `FSM_STATE_TIMEOUT_MS` | `70000` | Max time in any state before FAULT |
| `FSM_PARALLEL_TOLERANCE_CM` | `0.4` | USS diff threshold to consider "aligned" |

### Speeds (in `state_machine_config.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `FSM_ALIGN_ROTATE_RPM` | `15.0` | Rotation speed during wall alignment |
| `FSM_FORWARD_AFTER_ALIGN_RPM` | `20.0` | Forward speed after alignment |
| `FSM_SHIFT_RIGHT_RPM` | `15.0` | Strafe speed while searching for center line |
| `FSM_SHIFT_ROTATE_RPM` | `30.0` | Rotational trim during right shift |
| `FSM_FORWARD_TO_HOG_RPM` | `30.0` | Forward speed to hog line |
| `FSM_LINE_FOLLOW_STRAFE_RPM` | `8.0` | Lateral correction during line following |
| `FSM_RETURN_LINE_FOLLOW_RPM` | `18.0` | Forward speed during return |
| `FSM_RETURN_TURN_RPM` | `15.0` | Rotation speed for 180° turn |
| `FSM_RETURN_SHIFT_LEFT_RPM` | `15.0` | Left strafe speed during return |

### Thresholds & Durations (in `state_machine_config.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `FSM_FORWARD_AFTER_ALIGN_MS` | `2000` | Duration to drive forward after align |
| `FSM_RETURN_FRONT_TARGET_CM` | `15.0` | Front USS distance to start return maneuver |
| `FSM_RETURN_TURN_180_MS` | `1300` | Duration of 180° timed turn |
| `FSM_RETURN_SHIFT_LEFT_30CM_MS` | `1800` | Duration of final left strafe (~30 cm) |
| `FSM_USS_FAULT_MAX_CONSEC` | `10` | Consecutive invalid USS reads before FAULT |
| `FSM_FAULT_LOG_INTERVAL_MS` | `1000` | How often FAULT prints status to serial |

### Launch (in `state_machine_config.h`)

| Macro | Default | Description |
|-------|---------|-------------|
| `FSM_LAUNCH_CLOCKWISE` | `1` | Stepper direction (1 = CW) |
| `FSM_LAUNCH_CYCLE_STEPS` | `1600` | Steps per launch cycle |

---

## Usage

```cpp
#include <state_machine.h>

void setup() {
  Serial.begin(SERIAL_BAUD);    // must be called first
  Mobility_Init();
  initUss();
  initLineSensor();
  initStepperMotor();
  initStateMachine();
}

void loop() {
  updateStateMachine();   // call every loop — fully non-blocking
  Mobility_Update();      // PID update for drivetrain
}
```

---

## Key Design Decisions

1. **Non-blocking**: Every state handler returns immediately. No `delay()` or `while` loops. Timed actions use `millis()` comparisons.
2. **One sensor read per loop**: `readSensors()` captures all sensor data into a `SensorFrame` struct at the top of `updateStateMachine()`, so handlers see a consistent snapshot.
3. **Round-robin USS**: Only one ultrasonic sensor is read per loop iteration (~30 ms max blocking) instead of all three (~90 ms). All three sensors are refreshed within 3 × `USS_READ_INTERVAL_MS`.
4. **Handler-per-state pattern**: Each state has its own function (`handleIdleForLoading`, `handleLaunch`, etc.), keeping `updateStateMachine()` as a clean dispatcher.
5. **One-shot launch guard**: The stepper is started exactly once via a `launch_started` flag, preventing accidental re-triggering. The `cycle_complete` flag is cleared on `stopStepperMotor()` to prevent stale flags from skipping future launches.
6. **USS fault counter**: Transient invalid reads are tolerated; only `FSM_USS_FAULT_MAX_CONSEC` consecutive invalids trigger a fault.
7. **E-Stop latches to FAULT**: E-Stop does not auto-resume. The operator must send `'r'` over serial after releasing the E-Stop button. The stepper motor is also killed immediately at the hardware level when E-Stop triggers.

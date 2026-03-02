/**
 * @file state_machine.cpp
 * @brief Non-blocking robot FSM for ME210 final project (v2)
 *
 * Changes from v1 (backups/StateMachine):
 *   - USS invalid-read fault counter (FSM_USS_FAULT_MAX_CONSEC)
 *   - LAUNCH: check completion FIRST, guard against stepper restart
 *   - RETURN_TO_END_ZONE: non-blocking sub-states instead of while loops
 *   - Line-follow strafe correction extracted into shared helper
 */

#include <Arduino.h>
#include <line_sensor.h>
#include <mobility_driver.h>
#include <stepper_motor.h>
#include <uss.h>
#include <math.h>

#include "state_machine.h"
#include "state_machine_config.h"

namespace {

/* ── State enums ─────────────────────────────────────────────────── */

enum class RobotState : uint8_t {
  IDLE_FOR_LOADING = 0,
  TURN_ALIGN_TO_LEFT_WALL,
  SHIFT_RIGHT_TO_CENTER_LINE,
  MOVE_FORWARD_TO_HOG_LINE,
  LAUNCH,
  RETURN_TO_END_ZONE,
  FAULT
};

enum class ReturnSubState : uint8_t {
  FOLLOW_LINE = 0,
  TURN_180,
  SHIFT_LEFT
};

/* ── Sensor snapshot ─────────────────────────────────────────────── */

struct SensorFrame {
  float   left_uss_front_cm    = 0.0f;
  float   left_uss_back_cm     = 0.0f;
  float   front_uss_cm         = 0.0f;
  uint8_t line_mask_5b         = 0;
  bool    launch_cycle_complete = false;
  bool    stop_commanded        = false;
  bool    recover_commanded     = false;
};

/* ── Runtime state ───────────────────────────────────────────────── */

struct Runtime {
  RobotState    state            = RobotState::IDLE_FOR_LOADING;
  unsigned long state_enter_ms   = 0UL;

  // RETURN_TO_END_ZONE sub-state tracking.
  ReturnSubState return_sub      = ReturnSubState::FOLLOW_LINE;
  unsigned long  sub_enter_ms    = 0UL;

  // USS invalid-read consecutive counter.
  uint8_t uss_invalid_consec     = 0;

  // LAUNCH guard — prevents restarting a stepper cycle.
  bool launch_started            = false;
};

Runtime g_rt{};

/* ── Helpers ─────────────────────────────────────────────────────── */

const char* toString(RobotState s) {
  switch (s) {
    case RobotState::IDLE_FOR_LOADING:          return "IDLE_FOR_LOADING";
    case RobotState::TURN_ALIGN_TO_LEFT_WALL:   return "TURN_ALIGN_TO_LEFT_WALL";
    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE:return "SHIFT_RIGHT_TO_CENTER_LINE";
    case RobotState::MOVE_FORWARD_TO_HOG_LINE:  return "MOVE_FORWARD_TO_HOG_LINE";
    case RobotState::LAUNCH:                    return "LAUNCH";
    case RobotState::RETURN_TO_END_ZONE:        return "RETURN_TO_END_ZONE";
    case RobotState::FAULT:                     return "FAULT";
    default:                                    return "UNKNOWN";
  }
}

SensorFrame readSensors() {
  SensorFrame in{};
  in.left_uss_front_cm    = ussLeftFrontCm();
  in.left_uss_back_cm     = ussLeftRearCm();
  in.front_uss_cm         = ussFrontCm();
  in.line_mask_5b         = lineMask5b();
  in.launch_cycle_complete = checkStepperLaunchCycleComplete();
  in.stop_commanded       = Mobility_IsEStopped();

  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == 'r' || ch == 'R') {
      in.recover_commanded = true;
    }
  }
  return in;
}

unsigned long stateElapsedMs(unsigned long now_ms) {
  return now_ms - g_rt.state_enter_ms;
}

bool stateTimedOut(unsigned long now_ms) {
  return stateElapsedMs(now_ms) > FSM_STATE_TIMEOUT_MS;
}

void setState(RobotState next, unsigned long now_ms) {
  if (next == g_rt.state) return;

  g_rt.state          = next;
  g_rt.state_enter_ms = now_ms;

  // Reset per-state runtime fields on entry.
  g_rt.uss_invalid_consec = 0;

  if (next == RobotState::RETURN_TO_END_ZONE) {
    g_rt.return_sub  = ReturnSubState::FOLLOW_LINE;
    g_rt.sub_enter_ms = now_ms;
  }
  if (next == RobotState::LAUNCH) {
    g_rt.launch_started = false;
  }

  Serial.print(F("[FSM] Enter -> "));
  Serial.println(toString(g_rt.state));
}

/**
 * Compute strafe correction RPM from a 5-bit line mask.
 * Positive = shift left, negative = shift right.
 */
float lineFollowStrafeCorrectionRpm(uint8_t mask) {
  switch (mask) {
    case 0b00100:  // centered
    case 0b01110:
      return 0.0f;

    case 0b01000:  // line left of center
    case 0b11000:
    case 0b10000:
    case 0b11100:
    case 0b01100:
      return FSM_LINE_FOLLOW_STRAFE_RPM;

    case 0b00010:  // line right of center
    case 0b00011:
    case 0b00001:
    case 0b00111:
    case 0b00110:
      return -FSM_LINE_FOLLOW_STRAFE_RPM;

    default:       // no line / ambiguous
      return 0.0f;
  }
}

/* ── Per-state handler functions ─────────────────────────────────── */

void handleIdleForLoading(const SensorFrame& in, unsigned long now_ms) {
  Mobility_StopAll();
  stopStepperMotor();

  if (stateElapsedMs(now_ms) >= FSM_LOAD_IDLE_MS) {
    setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
  }
}

void handleTurnAlignToLeftWall(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  // USS validity — fault after N consecutive invalids.
  if ((in.left_uss_front_cm <= 0.0f) || (in.left_uss_back_cm <= 0.0f)) {
    g_rt.uss_invalid_consec++;
    Mobility_StopAll();
    if (g_rt.uss_invalid_consec >= FSM_USS_FAULT_MAX_CONSEC) {
      Serial.println(F("[FSM] USS invalid count exceeded — FAULT."));
      setState(RobotState::FAULT, now_ms);
    }
    return;
  }
  g_rt.uss_invalid_consec = 0;

  const float left_diff_cm = in.left_uss_front_cm - in.left_uss_back_cm;
  const bool  front_clear  = !ussFrontTriggered();

  // Aligned + front clear → transition.
  if ((fabsf(left_diff_cm) <= FSM_PARALLEL_TOLERANCE_CM) && front_clear) {
    Mobility_StopAll();
    setState(RobotState::SHIFT_RIGHT_TO_CENTER_LINE, now_ms);
    return;
  }

  // Front not clear → rotate CW away from wall.
  if (!front_clear) {
    Mobility_RotateCW(FSM_ALIGN_ROTATE_RPM);
    return;
  }

  // Align: CCW if front sensor reads farther, else CW.
  if (left_diff_cm > 0.0f) {
    Mobility_RotateCCW(FSM_ALIGN_ROTATE_RPM);
  } else {
    Mobility_RotateCW(FSM_ALIGN_ROTATE_RPM);
  }
}

void handleShiftRightToCenterLine(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  // const float left_diff_cm = in.left_uss_front_cm - in.left_uss_back_cm;
  const bool  centered     =
      (in.line_mask_5b == 0b00100) || (in.line_mask_5b == 0b01110);

  // Centered → forward to hog line.
  if (centered) {
    Mobility_StopAll();
    setState(RobotState::MOVE_FORWARD_TO_HOG_LINE, now_ms);
    return;
  }

  // Strafe right with rotation trim to stay parallel.
  // float rot_trim = 0.0f;
  // if (left_diff_cm > FSM_PARALLEL_TOLERANCE_CM) {
  //   rot_trim = FSM_SHIFT_ROTATE_RPM;
  // } else if (left_diff_cm < -FSM_PARALLEL_TOLERANCE_CM) {
  //   rot_trim = -FSM_SHIFT_ROTATE_RPM;
  // }

  Mobility_Drive(0.0f, -FSM_SHIFT_RIGHT_RPM, 0.0f);
}

void handleMoveForwardToHogLine(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  // Hog line = all 5 sensors see black.
  if (in.line_mask_5b == 0b11111) {
    Mobility_StopAll();
    setState(RobotState::LAUNCH, now_ms);
    return;
  }

  Mobility_Drive(FSM_FORWARD_TO_HOG_RPM,
                 lineFollowStrafeCorrectionRpm(in.line_mask_5b),
                 0.0f);
}

void handleLaunch(const SensorFrame& in, unsigned long now_ms) {
  // 1. Stop drivetrain.
  Mobility_StopAll();

  // 2. Check completion FIRST — transition immediately.
  if (in.launch_cycle_complete) {
    setState(RobotState::RETURN_TO_END_ZONE, now_ms);
    return;
  }

  // 3. Start stepper exactly once.
  if (!isStepperMotorBusy() && !g_rt.launch_started) {
    startStepperLaunchCycle(
        static_cast<bool>(FSM_LAUNCH_CLOCKWISE),
        static_cast<uint16_t>(FSM_LAUNCH_CYCLE_STEPS));
    g_rt.launch_started = true;
  }

  // 4. Advance stepper each loop.
  updateStepperMotor();
}

void handleReturnToEndZone(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  switch (g_rt.return_sub) {

    /* ── FOLLOW_LINE: line-follow until front USS ≤ target ── */
    case ReturnSubState::FOLLOW_LINE: {
      const bool reached =
          (in.front_uss_cm > 0.0f) &&
          (in.front_uss_cm <= FSM_RETURN_FRONT_TARGET_CM);

      if (reached) {
        Mobility_StopAll();
        g_rt.return_sub  = ReturnSubState::TURN_180;
        g_rt.sub_enter_ms = now_ms;
        return;
      }

      Mobility_Drive(FSM_RETURN_LINE_FOLLOW_RPM,
                     lineFollowStrafeCorrectionRpm(in.line_mask_5b),
                     0.0f);
      break;
    }

    /* ── TURN_180: timed CW rotation ────────────────────── */
    case ReturnSubState::TURN_180: {
      if ((now_ms - g_rt.sub_enter_ms) >= FSM_RETURN_TURN_180_MS) {
        Mobility_StopAll();
        g_rt.return_sub  = ReturnSubState::SHIFT_LEFT;
        g_rt.sub_enter_ms = now_ms;
        return;
      }

      Mobility_RotateCW(FSM_RETURN_TURN_RPM);
      break;
    }

    /* ── SHIFT_LEFT: timed left strafe ──────────────────── */
    case ReturnSubState::SHIFT_LEFT: {
      if ((now_ms - g_rt.sub_enter_ms) >= FSM_RETURN_SHIFT_LEFT_30CM_MS) {
        Mobility_StopAll();
        setState(RobotState::IDLE_FOR_LOADING, now_ms);
        return;
      }

      Mobility_StrafeLeft(FSM_RETURN_SHIFT_LEFT_RPM);
      break;
    }

    default:
      Mobility_StopAll();
      setState(RobotState::FAULT, now_ms);
      break;
  }
}

void handleFault(const SensorFrame& in, unsigned long now_ms) {
  static unsigned long last_fault_log_ms = 0UL;

  Mobility_StopAll();
  stopStepperMotor();

  if ((now_ms - last_fault_log_ms) >= FSM_FAULT_LOG_INTERVAL_MS) {
    Serial.println(F("[FSM] FAULT latched. Send 'r' to recover."));
    last_fault_log_ms = now_ms;
  }

  if (in.recover_commanded && !Mobility_IsEStopped()) {
    setState(RobotState::IDLE_FOR_LOADING, now_ms);
  }
}

}  // namespace

/* ═══════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════ */

void initStateMachine() {
  g_rt = Runtime{};            // zero-init everything
  g_rt.state_enter_ms = millis();

  Serial.print(F("[FSM] Init -> "));
  Serial.println(F("IDLE_FOR_LOADING"));
}

void updateStateMachine() {
  const unsigned long now_ms = millis();

  // Refresh sensor / actuator caches once per loop.
  const bool need_uss =
    (g_rt.state == RobotState::TURN_ALIGN_TO_LEFT_WALL) ||
    ((g_rt.state == RobotState::RETURN_TO_END_ZONE) &&
     (g_rt.return_sub == ReturnSubState::FOLLOW_LINE));

  if (need_uss) {
    updateUss();
  }
  updateLineSensor();

  const SensorFrame in = readSensors();

  /* ── Global override: E-stop → latch into FAULT ─────────────── */
  if (in.stop_commanded) {
    Mobility_StopAll();
    stopStepperMotor();
    if (g_rt.state != RobotState::FAULT) {
      Serial.println(F("[FSM] E-STOP — entering FAULT. Send 'r' to recover."));
      setState(RobotState::FAULT, now_ms);
    }
    return;
  }

  /* ── Global guard: state timeout → FAULT ─────────────────────── */
  if ((g_rt.state != RobotState::FAULT) && stateTimedOut(now_ms)) {
    setState(RobotState::FAULT, now_ms);
    return;
  }

  /* ── Dispatch to per-state handler ──────────────────────────── */
  switch (g_rt.state) {
    case RobotState::IDLE_FOR_LOADING:           handleIdleForLoading(in, now_ms);          break;
    case RobotState::TURN_ALIGN_TO_LEFT_WALL:    handleTurnAlignToLeftWall(in, now_ms);     break;
    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE: handleShiftRightToCenterLine(in, now_ms);  break;
    case RobotState::MOVE_FORWARD_TO_HOG_LINE:   handleMoveForwardToHogLine(in, now_ms);    break;
    case RobotState::LAUNCH:                     handleLaunch(in, now_ms);                  break;
    case RobotState::RETURN_TO_END_ZONE:         handleReturnToEndZone(in, now_ms);         break;
    case RobotState::FAULT:                      handleFault(in, now_ms);                   break;
    default:                                     setState(RobotState::FAULT, now_ms);       break;
  }
}

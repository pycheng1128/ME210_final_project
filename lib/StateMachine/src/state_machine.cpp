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
  PRE_ALIGN_ROTATE_CCW,
  TURN_ALIGN_TO_LEFT_WALL,
  MOVE_FORWARD_AFTER_ALIGN,
  SHIFT_RIGHT_TO_CENTER_LINE,
  MOVE_FORWARD_TO_HOG_LINE,
  LAUNCH,
  RETURN_TO_END_ZONE,
  FAULT
};

enum class ReturnSubState : uint8_t {
  BACKWARD_SPEEDUP = 0,  // High-speed backward for speedup distance
  RECENTER,              // Strafe back to center line (cycle 2/3 only)
  BACKWARD_TO_LINE,      // Normal backward with line-follow
  SHIFT_LEFT,
  STOPPED
};

enum class FwdHogSubState : uint8_t {
  SPEEDUP = 0,        // High-speed forward phase
  LATERAL_SHIFT,      // Strafe left/right (cycle 2/3 only)
  LINE_FOLLOW         // Normal speed line-follow to hogline
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

/* ── Encoder helpers (4-motor averaging for mecanum kinematics) ──── */

struct EncoderSnapshot {
  long fl = 0, fr = 0, bl = 0, br = 0;
};

EncoderSnapshot captureEncoders() {
  EncoderSnapshot s;
  s.fl = Mobility_GetEncoderCount(MOB_FL);
  s.fr = Mobility_GetEncoderCount(MOB_FR);
  s.bl = Mobility_GetEncoderCount(MOB_BL);
  s.br = Mobility_GetEncoderCount(MOB_BR);
  return s;
}

/**
 * Pure forward encoder delta — cancels strafe & rotation components.
 * Uses motor inversions {-1,+1,-1,+1}: fwd = (d_FR + d_BR - d_FL - d_BL) / 4
 */
long forwardEncoderDelta(const EncoderSnapshot& s) {
  return ( (Mobility_GetEncoderCount(MOB_FR) - s.fr)
         + (Mobility_GetEncoderCount(MOB_BR) - s.br)
         - (Mobility_GetEncoderCount(MOB_FL) - s.fl)
         - (Mobility_GetEncoderCount(MOB_BL) - s.bl) ) / 4;
}

/**
 * Pure strafe encoder delta — cancels forward & rotation components.
 * strafe = (d_FL + d_FR - d_BL - d_BR) / 4
 */
long strafeEncoderDelta(const EncoderSnapshot& s) {
  return ( (Mobility_GetEncoderCount(MOB_FL) - s.fl)
         + (Mobility_GetEncoderCount(MOB_FR) - s.fr)
         - (Mobility_GetEncoderCount(MOB_BL) - s.bl)
         - (Mobility_GetEncoderCount(MOB_BR) - s.br) ) / 4;
}

/* ── Runtime state ───────────────────────────────────────────────── */

struct Runtime {
  RobotState    state            = RobotState::IDLE_FOR_LOADING;
  unsigned long state_enter_ms   = 0UL;

  // RETURN_TO_END_ZONE sub-state tracking.
  ReturnSubState return_sub      = ReturnSubState::BACKWARD_TO_LINE;
  unsigned long  sub_enter_ms    = 0UL;
  uint8_t       return_line_consec       = 0;
  unsigned long return_last_line_check_ms = 0UL;

  // PRE_ALIGN debounce — consecutive wall-detected readings required.
  uint8_t       pre_align_consec       = 0;
  unsigned long pre_align_last_check_ms = 0UL;

  // Alignment debounce — consecutive aligned readings required.
  uint8_t       uss_aligned_consec   = 0;
  unsigned long last_aligned_check_ms = 0UL;

  // LAUNCH guard — prevents restarting a stepper cycle.
  bool launch_started            = false;

  // Set after first alignment — skip PRE_ALIGN/TURN_ALIGN on later cycles.
  bool has_aligned               = false;

  // Speed-up phase: 4-motor encoder snapshots at state entry.
  EncoderSnapshot fwd_start_enc{};
  EncoderSnapshot ret_start_enc{};

  // MOVE_FORWARD_TO_HOG_LINE sub-state tracking.
  uint8_t         hogline_cycle    = 0;
  FwdHogSubState  fwd_hog_sub     = FwdHogSubState::SPEEDUP;
  EncoderSnapshot lateral_start_enc{};
};

Runtime g_rt{};

/* ── Helpers ─────────────────────────────────────────────────────── */

const char* toString(RobotState s) {
  switch (s) {
    case RobotState::IDLE_FOR_LOADING:          return "IDLE_FOR_LOADING";
    case RobotState::PRE_ALIGN_ROTATE_CCW:      return "PRE_ALIGN_ROTATE_CCW";
    case RobotState::TURN_ALIGN_TO_LEFT_WALL:   return "TURN_ALIGN_TO_LEFT_WALL";
    case RobotState::MOVE_FORWARD_AFTER_ALIGN:  return "MOVE_FORWARD_AFTER_ALIGN";
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
  g_rt.pre_align_consec        = 0;
  g_rt.pre_align_last_check_ms = now_ms;
  g_rt.uss_aligned_consec      = 0;
  g_rt.last_aligned_check_ms   = now_ms;

  if (next == RobotState::MOVE_FORWARD_TO_HOG_LINE) {
    g_rt.fwd_start_enc = captureEncoders();
    g_rt.fwd_hog_sub   = FwdHogSubState::SPEEDUP;
    g_rt.hogline_cycle++;
  }
  if (next == RobotState::RETURN_TO_END_ZONE) {
    g_rt.return_sub  = ReturnSubState::BACKWARD_SPEEDUP;
    g_rt.sub_enter_ms = now_ms;
    g_rt.return_line_consec = 0;
    g_rt.return_last_line_check_ms = now_ms;
    g_rt.ret_start_enc = captureEncoders();
  }
  if (next == RobotState::LAUNCH) {
    g_rt.launch_started = false;
  }
  if (next == RobotState::MOVE_FORWARD_AFTER_ALIGN) {
    g_rt.has_aligned = true;
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
    if (g_rt.has_aligned) {
      // Already aligned from a previous cycle — skip straight to forward.
      setState(RobotState::MOVE_FORWARD_AFTER_ALIGN, now_ms);
    } else {
      setState(RobotState::PRE_ALIGN_ROTATE_CCW, now_ms);
    }
  }
}

void handlePreAlignRotateCcw(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  // Both left sensors must return a valid reading within trigger range
  // to count as "wall detected", AND the front sensor must have no echo.
  const bool left_front_in_range = (in.left_uss_front_cm > 0.0f)
                                && (in.left_uss_front_cm < USS_THRESHOLD_CM);
  const bool left_back_in_range  = (in.left_uss_back_cm > 0.0f)
                                && (in.left_uss_back_cm < USS_THRESHOLD_CM);
  const bool front_no_echo       = (in.front_uss_cm <= 0.0f);

  // Debounce: require FSM_PRE_ALIGN_CONSEC_REQUIRED consecutive hits,
  // rate-limited to one check per USS cycle, to avoid stale-data glitches.
  if (left_front_in_range && left_back_in_range && front_no_echo) {
    if ((now_ms - g_rt.pre_align_last_check_ms) >= FSM_ALIGN_DEBOUNCE_MS) {
      g_rt.pre_align_consec++;
      g_rt.pre_align_last_check_ms = now_ms;
    }
    if (g_rt.pre_align_consec >= FSM_PRE_ALIGN_CONSEC_REQUIRED) {
      Mobility_StopAll();
      setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
      return;
    }
  } else {
    g_rt.pre_align_consec = 0;
    g_rt.pre_align_last_check_ms = now_ms;
  }

  if (stateElapsedMs(now_ms) >= FSM_ALIGN_NO_ECHO_TIMEOUT_MS) {
    Mobility_StopAll();
    Serial.println(F("[FSM] PRE_ALIGN timeout — FAULT."));
    setState(RobotState::FAULT, now_ms);
    return;
  }

  // Hardcoded exploration direction.
  Mobility_RotateCCW(FSM_ALIGN_SEARCH_ROTATE_RPM);
}

void handleTurnAlignToLeftWall(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  const bool left_front_valid = (in.left_uss_front_cm > 0.0f);
  const bool left_back_valid  = (in.left_uss_back_cm > 0.0f);

  // Recovery mode: if either left USS is invalid, keep searching by CCW rotation.
  if (!left_front_valid || !left_back_valid) {
    g_rt.uss_aligned_consec = 0;
    g_rt.last_aligned_check_ms = now_ms;

    Mobility_RotateCCW(FSM_ALIGN_SEARCH_ROTATE_RPM);
    return;
  }

  const float left_diff_cm = in.left_uss_front_cm - in.left_uss_back_cm;
  const bool  front_clear  = !ussFrontTriggered();

  // Aligned + front clear → require N consecutive hits, rate-limited to USS cycle.
  if ((fabsf(left_diff_cm) <= FSM_PARALLEL_TOLERANCE_CM) && front_clear) {
    if ((now_ms - g_rt.last_aligned_check_ms) >= FSM_ALIGN_DEBOUNCE_MS) {
      g_rt.uss_aligned_consec++;
      g_rt.last_aligned_check_ms = now_ms;
    }
    if (g_rt.uss_aligned_consec >= FSM_ALIGN_CONSEC_REQUIRED) {
      Mobility_StopAll();
      setState(RobotState::MOVE_FORWARD_AFTER_ALIGN, now_ms);
      return;
    }
  } else {
    g_rt.uss_aligned_consec = 0;
    g_rt.last_aligned_check_ms = now_ms;
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

void handleMoveForwardAfterAlign(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  // Timed forward drive — transition when duration expires.
  if (stateElapsedMs(now_ms) >= FSM_FORWARD_AFTER_ALIGN_MS) {
    Mobility_StopAll();
    setState(RobotState::SHIFT_RIGHT_TO_CENTER_LINE, now_ms);
    return;
  }

  Mobility_Drive(FSM_FORWARD_AFTER_ALIGN_RPM, 0.0f, 0.0f);
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

  // Hog line = all 5 sensors (or 4 rightmost) see black → transition regardless of sub-state.
  if (in.line_mask_5b == 0b11111 || in.line_mask_5b == 0b01111) {
    Mobility_StopAll();
    setState(RobotState::LAUNCH, now_ms);
    return;
  }

  switch (g_rt.fwd_hog_sub) {

    /* ── SPEEDUP: high-speed forward for FSM_SPEEDUP_DISTANCE_CM ── */
    case FwdHogSubState::SPEEDUP: {
      const long enc_traveled = labs(forwardEncoderDelta(g_rt.fwd_start_enc));
      if (enc_traveled >= FSM_SPEEDUP_ENCODER_COUNTS) {
        Mobility_StopAll();
        // Cycle 2 → shift left, cycle 3 → shift right, otherwise skip.
        if (g_rt.hogline_cycle == 2 || g_rt.hogline_cycle == 3) {
          g_rt.fwd_hog_sub = FwdHogSubState::LATERAL_SHIFT;
          g_rt.lateral_start_enc = captureEncoders();
        } else {
          g_rt.fwd_hog_sub = FwdHogSubState::LINE_FOLLOW;
        }
        return;
      }

      Mobility_Drive(FSM_SPEEDUP_RPM,
                     lineFollowStrafeCorrectionRpm(in.line_mask_5b),
                     0.0f);
      break;
    }

    /* ── LATERAL_SHIFT: strafe left (cycle 2) or right (cycle 3) ── */
    case FwdHogSubState::LATERAL_SHIFT: {
      const long lat_traveled = labs(strafeEncoderDelta(g_rt.lateral_start_enc));
      if (lat_traveled >= FSM_LATERAL_SHIFT_ENCODER_COUNTS) {
        Mobility_StopAll();
        g_rt.fwd_hog_sub = FwdHogSubState::LINE_FOLLOW;
        return;
      }

      // Cycle 2 → left (+vy), cycle 3 → right (-vy).
      const float strafe_vy = (g_rt.hogline_cycle == 2)
                              ? FSM_LATERAL_SHIFT_RPM
                              : -FSM_LATERAL_SHIFT_RPM;
      Mobility_Drive(0.0f, strafe_vy, 0.0f);
      break;
    }

    /* ── LINE_FOLLOW: normal speed with line-follow correction ── */
    case FwdHogSubState::LINE_FOLLOW: {
      Mobility_Drive(FSM_FORWARD_TO_HOG_RPM,
                     lineFollowStrafeCorrectionRpm(in.line_mask_5b),
                     0.0f);
      break;
    }
  }
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

bool isReturnEndLineMask(uint8_t mask) {
  return mask == 0b11000
      || mask == 0b11100
      || mask == 0b11110;
}

void handleReturnToEndZone(const SensorFrame& in, unsigned long now_ms) {
  stopStepperMotor();

  switch (g_rt.return_sub) {

    /* ── BACKWARD_SPEEDUP: fast backward for FSM_SPEEDUP_DISTANCE_CM ── */
    case ReturnSubState::BACKWARD_SPEEDUP: {
      const long ret_traveled = labs(forwardEncoderDelta(g_rt.ret_start_enc));
      if (ret_traveled >= FSM_SPEEDUP_ENCODER_COUNTS) {
        Mobility_StopAll();
        // Cycle 2/3 shifted laterally → recenter before continuing.
        if (g_rt.hogline_cycle == 2 || g_rt.hogline_cycle == 3) {
          g_rt.return_sub = ReturnSubState::RECENTER;
        } else {
          g_rt.return_sub = ReturnSubState::BACKWARD_TO_LINE;
        }
        return;
      }

      Mobility_Drive(-FSM_SPEEDUP_RPM,
                     lineFollowStrafeCorrectionRpm(in.line_mask_5b),
                     0.0f);
      break;
    }

    /* ── RECENTER: strafe back to center line using line sensor ── */
    case ReturnSubState::RECENTER: {
      const bool centered = (in.line_mask_5b == 0b00100) || (in.line_mask_5b == 0b01110);
      if (centered) {
        Mobility_StopAll();
        g_rt.return_sub = ReturnSubState::BACKWARD_TO_LINE;
        return;
      }

      // Cycle 2 shifted left on forward → shift right to recenter.
      // Cycle 3 shifted right on forward → shift left to recenter.
      const float strafe_vy = (g_rt.hogline_cycle == 2)
                              ? -FSM_LATERAL_SHIFT_RPM
                              :  FSM_LATERAL_SHIFT_RPM;
      Mobility_Drive(0.0f, strafe_vy, 0.0f);
      break;
    }

    /* ── BACKWARD_TO_LINE: normal backward with line-follow until end line ── */
    case ReturnSubState::BACKWARD_TO_LINE: {
      // Debounce: require N consecutive left-side line detections.
      if (isReturnEndLineMask(in.line_mask_5b)) {
        if ((now_ms - g_rt.return_last_line_check_ms) >= FSM_ALIGN_DEBOUNCE_MS) {
          g_rt.return_line_consec++;
          g_rt.return_last_line_check_ms = now_ms;
        }
        if (g_rt.return_line_consec >= FSM_RETURN_LINE_CONSEC_REQUIRED) {
          Mobility_StopAll();
          g_rt.return_sub   = ReturnSubState::SHIFT_LEFT;
          g_rt.sub_enter_ms = now_ms;
          return;
        }
      } else {
        g_rt.return_line_consec = 0;
        g_rt.return_last_line_check_ms = now_ms;
      }

      Mobility_Drive(-FSM_RETURN_LINE_FOLLOW_RPM,
                     lineFollowStrafeCorrectionRpm(in.line_mask_5b),
                     0.0f);
      break;
    }

    /* ── SHIFT_LEFT: timed left strafe, then back to IDLE ── */
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
    (g_rt.state == RobotState::IDLE_FOR_LOADING) ||
    (g_rt.state == RobotState::PRE_ALIGN_ROTATE_CCW) ||
    (g_rt.state == RobotState::TURN_ALIGN_TO_LEFT_WALL);

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
    case RobotState::PRE_ALIGN_ROTATE_CCW:       handlePreAlignRotateCcw(in, now_ms);       break;
    case RobotState::TURN_ALIGN_TO_LEFT_WALL:    handleTurnAlignToLeftWall(in, now_ms);     break;
    case RobotState::MOVE_FORWARD_AFTER_ALIGN:   handleMoveForwardAfterAlign(in, now_ms);   break;
    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE: handleShiftRightToCenterLine(in, now_ms);  break;
    case RobotState::MOVE_FORWARD_TO_HOG_LINE:   handleMoveForwardToHogLine(in, now_ms);    break;
    case RobotState::LAUNCH:                     handleLaunch(in, now_ms);                  break;
    case RobotState::RETURN_TO_END_ZONE:         handleReturnToEndZone(in, now_ms);         break;
    case RobotState::FAULT:                      handleFault(in, now_ms);                   break;
    default:                                     setState(RobotState::FAULT, now_ms);       break;
  }
}

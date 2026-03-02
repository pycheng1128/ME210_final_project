#include <Arduino.h>
#include <line_sensor.h>
#include <mobility_driver.h>
#include <stepper_motor.h>
#include <uss.h>
#include <math.h>

#include "state_machine.h"
#include "state_machine_config.h"

namespace {

enum class RobotState : uint8_t {
  IDLE_FOR_LOADING = 0,
  TURN_ALIGN_TO_LEFT_WALL,
  SHIFT_RIGHT_TO_CENTER_LINE,
  MOVE_FORWARD_TO_HOG_LINE,
  LAUNCH,
  RETURN_TO_END_ZONE,
  FAULT
};

struct SensorFrame {
  // Ultrasonic readings in cm.
  float left_uss_front_cm = 0.0f;
  float left_uss_back_cm = 0.0f;
  float front_uss_cm = 0.0f;

  // 5-bit line mask, bit4 = leftmost ... bit0 = rightmost.
  // Bit value 1 means black line is detected.
  uint8_t line_mask_5b = 0;

  bool launch_cycle_complete = false;
  bool end_zone_reached = false;
  bool stop_commanded = false;
};

struct Runtime {
  RobotState state = RobotState::IDLE_FOR_LOADING;
  unsigned long state_enter_ms = 0UL;
};

Runtime g_rt{};

const char* toString(RobotState state) {
  switch (state) {
    case RobotState::IDLE_FOR_LOADING: return "IDLE_FOR_LOADING";
    case RobotState::TURN_ALIGN_TO_LEFT_WALL: return "TURN_ALIGN_TO_LEFT_WALL";
    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE: return "SHIFT_RIGHT_TO_CENTER_LINE";
    case RobotState::MOVE_FORWARD_TO_HOG_LINE: return "MOVE_FORWARD_TO_HOG_LINE";
    case RobotState::LAUNCH: return "LAUNCH";
    case RobotState::RETURN_TO_END_ZONE: return "RETURN_TO_END_ZONE";
    case RobotState::FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

SensorFrame readSensors() {
  SensorFrame in{};

  // Snapshot all currently available subsystem outputs.
  in.left_uss_front_cm = ussLeftFrontCm();
  in.left_uss_back_cm = ussLeftRearCm();
  in.front_uss_cm = ussFrontCm();
  in.line_mask_5b = lineMask5b();
  in.launch_cycle_complete = checkStepperLaunchCycleComplete();

  // TODO: connect end-zone detection and stop-command interfaces.
  return in;
}

unsigned long stateElapsedMs(unsigned long now_ms) {
  return now_ms - g_rt.state_enter_ms;
}

bool stateTimedOut(unsigned long now_ms) {
  return stateElapsedMs(now_ms) > FSM_STATE_TIMEOUT_MS;
}

void setState(RobotState next, unsigned long now_ms) {
  if (next == g_rt.state) {
    return;
  }

  g_rt.state = next;
  g_rt.state_enter_ms = now_ms;

  Serial.print(F("[FSM] Enter -> "));
  Serial.println(toString(g_rt.state));
}

}  // namespace

void initStateMachine() {
  g_rt.state = RobotState::IDLE_FOR_LOADING;
  g_rt.state_enter_ms = millis();

  Serial.print(F("[FSM] Init -> "));
  Serial.println(F("IDLE_FOR_LOADING"));
}

void updateStateMachine() {
  const unsigned long now_ms = millis();

  // Refresh sensor and actuator module caches once per loop.
  updateUss();
  updateLineSensor();

  const SensorFrame in = readSensors();

  // TODO: wire global override(s), e.g. remote stop command and E-stop behavior.
  if (in.stop_commanded) {
    setState(RobotState::IDLE_FOR_LOADING, now_ms);
    return;
  }

  if (stateTimedOut(now_ms)) {
    setState(RobotState::FAULT, now_ms);
    return;
  }

  switch (g_rt.state) {
    case RobotState::IDLE_FOR_LOADING: {
      Mobility_StopAll();
      stopStepperMotor();

      if (stateElapsedMs(now_ms) >= FSM_LOAD_IDLE_MS) {
        setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
      }
      break;
    }

    case RobotState::TURN_ALIGN_TO_LEFT_WALL: {
      stopStepperMotor();

      // Only left USS pair is used for wall-parallel alignment.
      // If distance smaller than 0.0f, stop motors
      if ((in.left_uss_front_cm <= 0.0f) || (in.left_uss_back_cm <= 0.0f)) {
        Mobility_StopAll();
        Serial.println(F("[FSM] Error: left USS distance <= 0 cm."));
        break;
      }

      const float left_diff_cm = in.left_uss_front_cm - in.left_uss_back_cm;
      const bool front_clear = !ussFrontTriggered();

      // Require both: left sensors aligned AND front USS sees no nearby wall.
      if ((fabsf(left_diff_cm) <= FSM_PARALLEL_TOLERANCE_CM) && front_clear) {
        Mobility_StopAll();
        setState(RobotState::SHIFT_RIGHT_TO_CENTER_LINE, now_ms);
        break;
      }

      // If front USS is triggered, robot is still pointing toward the wall.
      if (!front_clear) {
        Mobility_RotateCW(FSM_ALIGN_ROTATE_RPM);
        break;
      }

      // If the left front USS detects larger distance, turn CCW, else CW
      if (left_diff_cm > 0.0f) {
        Mobility_RotateCCW(FSM_ALIGN_ROTATE_RPM);
      } else {
        Mobility_RotateCW(FSM_ALIGN_ROTATE_RPM);
      }
      break;
    }

    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE: {
      stopStepperMotor();

      if ((in.left_uss_front_cm <= 0.0f) || (in.left_uss_back_cm <= 0.0f)) {
        Mobility_StopAll();
        Serial.println(F("[FSM] Error: left USS distance <= 0 cm during shift."));
        break;
      }

      const float left_diff_cm = in.left_uss_front_cm - in.left_uss_back_cm;
      const bool front_clear = !ussFrontTriggered();
      const bool centered_on_middle_line =
          (in.line_mask_5b == 0b00100) || (in.line_mask_5b == 0b01110);

      if (centered_on_middle_line) {
        Mobility_StopAll();
        setState(RobotState::MOVE_FORWARD_TO_HOG_LINE, now_ms);
        break;
      }

      // If geometry drifts too much, go back to dedicated wall alignment first.
      // For not reliable right shift
      if (!front_clear || (fabsf(left_diff_cm) > FSM_SHIFT_REALIGN_DIFF_CM)) {
        Mobility_StopAll();
        setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
        break;
      }
      
      float rotation_correction_rpm = 0.0f;
      if (left_diff_cm > FSM_PARALLEL_TOLERANCE_CM) {
        rotation_correction_rpm = FSM_SHIFT_ROTATE_RPM;
      } else if (left_diff_cm < -FSM_PARALLEL_TOLERANCE_CM) {
        rotation_correction_rpm = -FSM_SHIFT_ROTATE_RPM;
      }

      // Strafe right while continuously trimming orientation with USS error.
      Mobility_Drive(0.0f, -FSM_SHIFT_RIGHT_RPM, rotation_correction_rpm);
      break;
    }

    case RobotState::MOVE_FORWARD_TO_HOG_LINE: {
      stopStepperMotor();

      // Use line sensor only in this state.
      if (in.line_mask_5b == 0b11111) {
        Mobility_StopAll();
        setState(RobotState::LAUNCH, now_ms);
        break;
      }

      float strafe_correction_rpm = 0.0f;
      switch (in.line_mask_5b) {
        // Centered on line.
        case 0b00100:
        case 0b01110:
          strafe_correction_rpm = 0.0f;
          break;

        // Line appears left of center -> shift left while moving forward.
        case 0b01000:
        case 0b11000:
        case 0b10000:
        case 0b11100:
        case 0b01100:
          strafe_correction_rpm = FSM_LINE_FOLLOW_STRAFE_RPM;
          break;

        // Line appears right of center -> shift right while moving forward.
        case 0b00010:
        case 0b00011:
        case 0b00001:
        case 0b00111:
        case 0b00110:
          strafe_correction_rpm = -FSM_LINE_FOLLOW_STRAFE_RPM;
          break;

        // No line / ambiguous pattern: rely on stable straight tracking.
        default:
          strafe_correction_rpm = 0.0f;
          break;
      }

      Mobility_Drive(FSM_FORWARD_TO_HOG_RPM, strafe_correction_rpm, 0.0f);
      break;
    }

    case RobotState::LAUNCH: {
      // Keep drivetrain stopped during launch.
      Mobility_StopAll();
      updateStepperMotor();

      // Start one launch cycle once; stepper progresses via updateStepperMotor().
      if (!isStepperMotorBusy() && !in.launch_cycle_complete) {
        startStepperLaunchCycle(
            static_cast<bool>(FSM_LAUNCH_CLOCKWISE),
            static_cast<uint16_t>(FSM_LAUNCH_CYCLE_STEPS));
      }

      // Toggle happens at updateStepperMotor()
      if (in.launch_cycle_complete) {
        setState(RobotState::RETURN_TO_END_ZONE, now_ms);
      }
      break;
    }

    case RobotState::RETURN_TO_END_ZONE: {
      stopStepperMotor();

      // Follow middle line until front USS reaches target distance.
      const bool reached_return_trigger =
          (in.front_uss_cm > 0.0f) &&
          (in.front_uss_cm <= FSM_RETURN_FRONT_TARGET_CM);

      if (!reached_return_trigger) {
        float strafe_correction_rpm = 0.0f;
        switch (in.line_mask_5b) {
          case 0b00100:
          case 0b01110:
            strafe_correction_rpm = 0.0f;
            break;
          case 0b01000:
          case 0b11000:
          case 0b10000:
          case 0b11100:
          case 0b01100:
            strafe_correction_rpm = FSM_LINE_FOLLOW_STRAFE_RPM;
            break;
          case 0b00010:
          case 0b00011:
          case 0b00001:
          case 0b00111:
          case 0b00110:
            strafe_correction_rpm = -FSM_LINE_FOLLOW_STRAFE_RPM;
            break;
          default:
            strafe_correction_rpm = 0.0f;
            break;
        }

        Mobility_Drive(FSM_FORWARD_TO_HOG_RPM, strafe_correction_rpm, 0.0f);
        break;
      }

      // Hardcoded 180-degree turn.
      Mobility_StopAll();
      unsigned long motion_start_ms = millis();
      while ((millis() - motion_start_ms) < FSM_RETURN_TURN_180_MS) {
        Mobility_RotateCW(FSM_RETURN_TURN_RPM);
        Mobility_Update();
      }

      // Hardcoded ~30 cm left shift.
      Mobility_StopAll();
      motion_start_ms = millis();
      while ((millis() - motion_start_ms) < FSM_RETURN_SHIFT_LEFT_30CM_MS) {
        Mobility_StrafeLeft(FSM_RETURN_SHIFT_LEFT_RPM);
        Mobility_Update();
      }

      Mobility_StopAll();
      setState(RobotState::IDLE_FOR_LOADING, millis());
      break;
    }

    case RobotState::FAULT:
    default: {
      // TODO: force a safe stop and wait for operator recovery command.
      break;
    }
  }
}

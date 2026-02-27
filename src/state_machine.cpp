#include <Arduino.h>

// Private to only this file state_machine.cpp
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

struct StateMachineConfig {
  unsigned long load_idle_ms = 10000UL;
  unsigned long state_timeout_ms = 12000UL;

  // Keep this below 0.5 cm as requested.
  float left_uss_match_tol_cm = 0.4f;
};

struct SensorFrame {
  // Ultrasonic readings in cm.
  float left_uss_front_cm = 0.0f;
  float left_uss_back_cm = 0.0f;
  float front_uss_cm = 0.0f;  // reserved for future threshold tuning

  // 5-bit line mask, bit4 = leftmost ... bit0 = rightmost.
  // bit value 1 means black line is detected.
  uint8_t line_mask_5b = 0;

  bool launch_cycle_complete = false;
  bool end_zone_reached = false;
  bool stop_commanded = false;
};

struct Runtime {
  RobotState state = RobotState::IDLE_FOR_LOADING;
  unsigned long state_enter_ms = 0UL;
};

// Create StateMachineConfig and Runtime instances
StateMachineConfig g_cfg;
Runtime g_rt;

const char* toString(RobotState s) {
  switch (s) {
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

bool isParallelToLeftWall(const SensorFrame& in) {
  const float left_diff = fabsf(in.left_uss_front_cm - in.left_uss_back_cm);
  return left_diff <= g_cfg.left_uss_match_tol_cm;
}

bool isCenteredOnMiddleLine(const SensorFrame& in) {
  // Accept thick-tape pattern around center as requested.
  return (in.line_mask_5b == 0b00100) || (in.line_mask_5b == 0b01110);
}

bool isAtHogLine(const SensorFrame& in) {
  // Instant detection is acceptable per requirement.
  return in.line_mask_5b == 0b11111;
}

bool timedOut(unsigned long now_ms) {
  return (now_ms - g_rt.state_enter_ms) > g_cfg.state_timeout_ms;
}

SensorFrame readSensors() {
  SensorFrame in{};
  // TODO(sensor module): populate ultrasonic and line data here.
  // TODO(command module): populate stop command and end-zone status.
  return in;
}

void enterState(RobotState next, unsigned long now_ms) {
  if (next == g_rt.state) {
    return;
  }
  g_rt.state = next;
  g_rt.state_enter_ms = now_ms;

  Serial.print(F("[FSM] Enter -> "));
  Serial.println(toString(g_rt.state));
}

void cmdStopAllMotion() {
  // TODO(navigation/motor module): stop all drivetrain outputs.
}

void cmdTurnAlignCounterClockwise() {
  // TODO(navigation module): counter-clockwise turning command.
  // Keep turning until parallel-to-left-wall guard is true.
}

void cmdShiftRightWithParallelHold() {
  // TODO(navigation module): shift right while maintaining wall parallelism.
}

void cmdForwardWithLineTracking() {
  // TODO(navigation module): move forward and keep center line tracked.
}

void cmdLaunch() {
  // TODO(launcher module): fire one cycle.
}

void cmdReturnToEndZone() {
  // TODO(navigation module): navigate back to starting end zone for reload.
}

}  // namespace

void initStateMachine() {
  g_rt.state = RobotState::IDLE_FOR_LOADING;
  g_rt.state_enter_ms = millis();

  Serial.print(F("[FSM] Init -> "));
  Serial.println(toString(g_rt.state));
}

void updateStateMachine() {
  const unsigned long now_ms = millis();
  const SensorFrame in = readSensors();

  // Global safety / override transitions.
  if (in.stop_commanded) {
    cmdStopAllMotion();
    enterState(RobotState::IDLE_FOR_LOADING, now_ms);
    return;
  }
  if (timedOut(now_ms)) {
    cmdStopAllMotion();
    enterState(RobotState::FAULT, now_ms);
    return;
  }

  switch (g_rt.state) {
    case RobotState::IDLE_FOR_LOADING: {
      cmdStopAllMotion();
      if ((now_ms - g_rt.state_enter_ms) >= g_cfg.load_idle_ms) {
        enterState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
      }
      break;
    }

    case RobotState::TURN_ALIGN_TO_LEFT_WALL: {
      cmdTurnAlignCounterClockwise();
      if (isParallelToLeftWall(in)) {
        enterState(RobotState::SHIFT_RIGHT_TO_CENTER_LINE, now_ms);
      }
      break;
    }

    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE: {
      cmdShiftRightWithParallelHold();

      // Constantly enforce parallel orientation.
      if (!isParallelToLeftWall(in)) {
        enterState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
        break;
      }
      if (isCenteredOnMiddleLine(in)) {
        enterState(RobotState::MOVE_FORWARD_TO_HOG_LINE, now_ms);
      }
      break;
    }

    case RobotState::MOVE_FORWARD_TO_HOG_LINE: {
      cmdForwardWithLineTracking();

      // Constantly enforce parallel orientation while moving.
      if (!isParallelToLeftWall(in)) {
        enterState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
        break;
      }
      if (isAtHogLine(in)) {
        enterState(RobotState::LAUNCH, now_ms);
      }
      break;
    }

    case RobotState::LAUNCH: {
      cmdLaunch();
      if (in.launch_cycle_complete) {
        enterState(RobotState::RETURN_TO_END_ZONE, now_ms);
      }
      break;
    }

    case RobotState::RETURN_TO_END_ZONE: {
      cmdReturnToEndZone();
      if (in.end_zone_reached) {
        enterState(RobotState::IDLE_FOR_LOADING, now_ms);
      }
      break;
    }

    case RobotState::FAULT:
    default: {
      cmdStopAllMotion();
      break;
    }
  }
}

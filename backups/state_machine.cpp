#include <Arduino.h>
#include <uss.h>
#include <stepper_motor.h>
#include <line_sensor.h>

namespace {

// must write RobotState::...
enum class RobotState : uint8_t {
  IDLE_FOR_LOADING = 0,
  TURN_ALIGN_TO_LEFT_WALL,
  SHIFT_RIGHT_TO_CENTER_LINE,
  MOVE_FORWARD_TO_HOG_LINE,
  LAUNCH,
  RETURN_TO_END_ZONE,
  FAULT
};

constexpr unsigned long kLoadIdleMs = 10000UL;
constexpr unsigned long kStateTimeoutMs = 12000UL;
constexpr float kLeftUssMatchToleranceCm = 0.4f;  // Keep below 0.5 cm.
constexpr uint16_t kLaunchCycleSteps = 200;
constexpr bool kLaunchClockwise = true;

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

// const SensorFrame& in: pass by original object, not a copy
bool isParallelToLeftWall(const SensorFrame& in) {
  const float left_diff = fabsf(in.left_uss_front_cm - in.left_uss_back_cm);
  return left_diff <= kLeftUssMatchToleranceCm;
}

bool isCenteredOnMiddleLine(const SensorFrame& in) {
  // Accept both narrow and thick center-line patterns.
  return (in.line_mask_5b == 0b00100) || (in.line_mask_5b == 0b01110);
}

bool isAtHogLine(const SensorFrame& in) {
  // Instant detection is acceptable per requirement.
  return in.line_mask_5b == 0b11111;
}

SensorFrame readSensors() {
  SensorFrame in{};

  // USS snapshot.
  in.left_uss_front_cm = ussLeftFrontCm();
  in.left_uss_back_cm = ussLeftRearCm();
  in.front_uss_cm = ussFrontCm();

  // Line sensor snapshot.
  in.line_mask_5b = lineMask5b();

  // Stepper launch completion event (one-shot).
  in.launch_cycle_complete = checkStepperLaunchCycleComplete();

  // TODO(command module): populate stop command and end-zone status.
  // TODO(navigation/localization module): populate end_zone_reached.
  return in;
}

unsigned long stateElapsedMs(unsigned long now_ms) {
  return now_ms - g_rt.state_enter_ms;
}

bool stateTimedOut(unsigned long now_ms) {
  return stateElapsedMs(now_ms) > kStateTimeoutMs;
}

void setState(RobotState next, unsigned long now_ms) {
  // return void if target state is the same as current state
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
  Serial.println(toString(g_rt.state));
}

void updateStateMachine() {
  const unsigned long now_ms = millis();

  // Refresh sensor modules once per loop.
  updateUss();
  updateLineSensor();

  // Run stepper non-blocking update each loop.
  updateStepperMotor();

  const SensorFrame in = readSensors();

  // Global overrides.
  if (in.stop_commanded) {
    // TODO(navigation/motor): stop all motion.
    stopStepperMotor();
    setState(RobotState::IDLE_FOR_LOADING, now_ms);
    return;
  }
  if (stateTimedOut(now_ms)) {
    // TODO(navigation/motor): stop all motion.
    stopStepperMotor();
    setState(RobotState::FAULT, now_ms);
    return;
  }

  switch (g_rt.state) {
    case RobotState::IDLE_FOR_LOADING: {
      // TODO(navigation/motor): stop all motion.
      if (stateElapsedMs(now_ms) >= kLoadIdleMs) {
        setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
      }
      break;
    }

    case RobotState::TURN_ALIGN_TO_LEFT_WALL: {
      // TODO(navigation): turn counter-clockwise until aligned.
      if (isParallelToLeftWall(in)) {
        setState(RobotState::SHIFT_RIGHT_TO_CENTER_LINE, now_ms);
      }
      break;
    }

    case RobotState::SHIFT_RIGHT_TO_CENTER_LINE: {
      // TODO(navigation): shift right while holding parallel orientation.

      // Constantly enforce parallel orientation.
      if (!isParallelToLeftWall(in)) {
        setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
        break;
      }
      if (isCenteredOnMiddleLine(in)) {
        setState(RobotState::MOVE_FORWARD_TO_HOG_LINE, now_ms);
      }
      break;
    }

    case RobotState::MOVE_FORWARD_TO_HOG_LINE: {
      // TODO(navigation): move forward with line tracking.

      // Constantly enforce parallel orientation while moving.
      if (!isParallelToLeftWall(in)) {
        setState(RobotState::TURN_ALIGN_TO_LEFT_WALL, now_ms);
        break;
      }
      if (isAtHogLine(in)) {
        setState(RobotState::LAUNCH, now_ms);
      }
      break;
    }

    case RobotState::LAUNCH: {
      // Start one launch cycle once; updateStepperMotor() drives it asynchronously.
      if (!isStepperMotorBusy() && !in.launch_cycle_complete) {
        startStepperLaunchCycle(kLaunchClockwise, kLaunchCycleSteps);
      }
      if (in.launch_cycle_complete) {
        setState(RobotState::RETURN_TO_END_ZONE, now_ms);
      }
      break;
    }

    case RobotState::RETURN_TO_END_ZONE: {
      // TODO(navigation): return to end zone.
      if (in.end_zone_reached) {
        setState(RobotState::IDLE_FOR_LOADING, now_ms);
      }
      break;
    }

    case RobotState::FAULT:
    default: {
      // TODO(navigation/motor): stop all motion.
      break;
    }
  }
}

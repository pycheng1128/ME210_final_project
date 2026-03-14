# Mobility System PID Control Strategy

This document details the PID (Proportional-Integral-Derivative) control strategy implemented in this project to regulate the wheel speeds of the 4-wheel Mecanum drive system. The core logic resides in the `MobilitySystem` library (`lib/MobilitySystem/src/mobility_driver.cpp`).

## Core Architecture
The PID controller runs at a fixed periodic interval (`MOB_PID_INTERVAL_MS`), called from the non-blocking main loop via `Mobility_Update()`. Four independent PI(D) controllers manage the individual wheel speeds by continuously tracking encoder feedback against the target RPM commands.

## Key Mechanisms & Features

### 1. Speed Measurement & Low-Pass Filtering
Encoder pulses are tracked via hardware interrupts, updating a volatile counter. During each PID interval, the raw RPM is calculated from the delta in encoder ticks. Because calculating velocity from discrete ticks can introduce measurement noise (especially at lower speeds), a **first-order Low-Pass Filter** is applied to smooth the measured speed:

```cpp
Filtered_RPM += MOB_LPF_ALPHA * (Raw_RPM - Filtered_RPM);
```
`MOB_LPF_ALPHA` provides a balance between rapid dynamic response and noise rejection.

### 2. PI(D) Control Equation
Our control loop relies on the Proportional and Integral terms （we are not using KD）
```cpp
Error = Target_RPM - Filtered_RPM;
Output = (Kp * Error) + (Ki * Integral) + (Kd * Derivative);
```

#### The Role of the Derivative (D) Term
The **Derivative (D)** term calculates the *rate of change* of the error over time. Its main purpose is to "predict" the future error trend. If the system is approaching the target RPM very quickly, the D-term acts as a dampener (like a shock absorber), applying an opposing force to prevent the speed from overshooting the target and oscillating. In our robot, the Derivative term (`MOB_PID_KD`) is typically set to `0` (effectively making this a PI controller). 
Calculating velocity directly from discrete encoder ticks is inherently noisy. Because the D-term relies on the *derivative* (rate of change) of the speed, it is extremely sensitive to measurement noise. Spikes in the calculated speed (due to jitter or missed ticks) would cause errors in the PWM output. The use of the Low-Pass Filter (`MOB_LPF_ALPHA`) helps, but PI control alone is usually more than sufficient for stable wheel speed regulation. Attuaclly we already set a safe slow down state to avoid the robot crossing the hogline.

### 3. Integral Saturation
To prevent the integral term from accumulating infinitely when the motors are saturated (e.g., commanded to a speed they physically cannot reach, or momentarily stalled), the system uses **Conditional Integration**:
The integral accumulator is paused if both of these conditions are met:
1. The integral component is saturated (`|Integral * Ki| >= MOB_PWM_MAX（255）`).
2. The current error shares the same sign as the integral (which would push it deeper into windup).

Furthermore, a hard clamp restricts the integral term strictly between `- MOB_PID_INTEGRAL_MAX` and `+ MOB_PID_INTEGRAL_MAX` as a failsafe.

### 4. Safety Brake on Direction Reversal
The motor cannot suddenly reverse direction, which would cause a large current spike and potentially damage the motor drivers and gearboxes (stated in motor specification). To protect the motor drivers and gearboxes, the system implements a **dynamic braking state**:
If the user commands a direction reversal (e.g., switching from Forward to Backward) while the motor is still spinning above `MOB_REVERSAL_RPM_THRESH`:
1. The target RPM is temporarily forced to `0` to brake the wheel.
2. The controller waits until the measured `Filtered_RPM` drops below `MOB_STOPPED_RPM_THRESH` (e.g., 2 RPM).
3. Only then is the new requested setpoint applied, smoothly accelerating the robot in the opposite direction.

### 5. Deadzone Compensation
Physical DC brushed motors require a minimum applied voltage to overcome static friction before they begin moving. If the PID controller computes a non-zero output, the system applies a **feedforward minimum PWM** (`MOB_PWM_DEADZONE`). This ensures the system reacts immediately to small control efforts without stalling at low velocities.

### 6. Mecanum Scaling
At the higher kinematics level, if the sum of forward, strafing, and rotational vectors causes any distinct wheel to exceed `MOB_MAX_RPM`, the requested RPMs for **all four wheels** are proportionally scaled down. This ensures the PID controllers receive achievable setpoints and the robot maintains its intended motion vector rather than drifting unpredictably.

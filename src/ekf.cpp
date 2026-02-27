/*
Arduino-friendly EKF for 2D diff-drive localization with gyro-bias estimation

State x = [X, Y, theta, b_g]^T
  X,Y    : position (meters)
  theta  : heading (radians)
  b_g    : gyro bias (rad/s)

Inputs each loop:
  - dL, dR  : left/right wheel travel since last update (meters)
  - gyroZ   : measured yaw rate (rad/s)
  - dt      : timestep (seconds)

Measurements (optional events):
  - Line constraint: e.g., "centerline detected => Y = Y_line"
  - IR bearing: z = bearing_to_beacon (radians), if you have it
    (Often easiest: make your IR processing output a bearing estimate.)
*/

#include <Arduino.h>
#include <math.h>

// ------------------------ Utilities ------------------------

static inline float wrapPi(float a) {
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

// 4x4 matrix, 4x1 vector helpers (small + explicit => Arduino-friendly)
struct EKF4 {
  float x[4];      // state
  float P[4][4];   // covariance

  // Tunables (start with these and tune)
  float q_xy;      // process noise for X,Y (m^2 per step-ish)
  float q_th;      // process noise for theta (rad^2 per step-ish)
  float q_bg;      // gyro bias random walk ( (rad/s)^2 per step-ish )

  EKF4() : q_xy(1e-6f), q_th(1e-5f), q_bg(1e-7f) {
    // init state
    x[0]=0; x[1]=0; x[2]=0; x[3]=0;
    // init covariance (moderate uncertainty)
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) P[i][j]=0;
    P[0][0]=0.05f*0.05f;   // 5 cm
    P[1][1]=0.05f*0.05f;
    P[2][2]=(10.0f*M_PI/180.0f)*(10.0f*M_PI/180.0f); // 10 deg
    P[3][3]=(0.05f)*(0.05f); // 0.05 rad/s bias uncertainty
  }

  // ------------------------ Prediction ------------------------
  // Diff-drive kinematics:
  // dS = (dR + dL)/2
  // dTheta_enc = (dR - dL)/wheelbase (optional, if you trust it)
  // We’ll use gyro for heading integration + bias estimate:
  // theta <- theta + (gyroZ - b_g) * dt
  void predict(float dL, float dR, float gyroZ, float dt) {
    const float dS = 0.5f * (dR + dL);

    // Use gyro for heading increment (best if wheels slip)
    const float omega = gyroZ - x[3];
    const float dTh = omega * dt;

    // Use mid-point heading for translation
    const float th_mid = x[2] + 0.5f * dTh;

    // State update
    x[0] += dS * cosf(th_mid);
    x[1] += dS * sinf(th_mid);
    x[2] = wrapPi(x[2] + dTh);
    // x[3] (bias) stays same in prediction (random walk modeled in Q)

    // Jacobian F = ∂f/∂x (4x4)
    // x,y depend on theta and bias through th_mid and dTh
    // Approx: dTh = (gyroZ - b)*dt => ∂dTh/∂b = -dt
    // th_mid = theta + 0.5*dTh => ∂th_mid/∂theta = 1, ∂th_mid/∂b = -0.5*dt
    const float s = sinf(th_mid);
    const float c = cosf(th_mid);

    float F[4][4];
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) F[i][j]=0.0f;
    F[0][0]=1.0f; F[1][1]=1.0f; F[2][2]=1.0f; F[3][3]=1.0f;

    // ∂X/∂theta = -dS*sin(th_mid) * ∂th_mid/∂theta = -dS*sin(th_mid)
    // ∂X/∂b     = -dS*sin(th_mid) * ∂th_mid/∂b     = -dS*sin(th_mid)*(-0.5*dt) = +0.5*dS*sin(th_mid)*dt
    // ∂Y/∂theta =  dS*cos(th_mid)
    // ∂Y/∂b     =  dS*cos(th_mid)*(-0.5*dt) = -0.5*dS*cos(th_mid)*dt
    F[0][2] = -dS * s;
    F[0][3] =  0.5f * dS * s * dt;
    F[1][2] =  dS * c;
    F[1][3] = -0.5f * dS * c * dt;

    // theta update: theta <- theta + (gyroZ - b)*dt => ∂theta/∂b = -dt
    F[2][3] = -dt;

    // Process noise Q (simple diagonal)
    float Q[4][4] = {0};
    Q[0][0] = q_xy;
    Q[1][1] = q_xy;
    Q[2][2] = q_th;
    Q[3][3] = q_bg;

    // Covariance update: P <- F P F^T + Q
    float FP[4][4] = {0};
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        float sum=0;
        for (int k=0;k<4;k++) sum += F[i][k]*P[k][j];
        FP[i][j]=sum;
      }
    }
    float FPFt[4][4] = {0};
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        float sum=0;
        for (int k=0;k<4;k++) sum += FP[i][k]*F[j][k]; // FPF^T, using FPFt[i][j]=sum_k FP[i][k]*F[j][k]
        FPFt[i][j]=sum;
      }
    }
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        P[i][j] = FPFt[i][j] + Q[i][j];
      }
    }
  }

  // ------------------------ 1D Measurement Update ------------------------
  // Generic scalar EKF update for measurement z = h(x) + v
  // Needs: innovation r, H (1x4), R
  void updateScalar(float r, const float H[4], float R) {
    // S = H P H^T + R (scalar)
    float S = R;
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        S += H[i] * P[i][j] * H[j];
      }
    }
    if (S < 1e-12f) return;

    // K = P H^T / S (4x1)
    float K[4];
    for (int i=0;i<4;i++){
      float sum=0;
      for (int j=0;j<4;j++) sum += P[i][j] * H[j];
      K[i] = sum / S;
    }

    // State update: x <- x + K*r
    for (int i=0;i<4;i++) x[i] += K[i] * r;
    x[2] = wrapPi(x[2]);

    // Cov update: P <- (I - K H) P  (Joseph form is safer but heavier)
    float IminusKH[4][4];
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        IminusKH[i][j] = (i==j ? 1.0f : 0.0f) - K[i]*H[j];
      }
    }
    float newP[4][4] = {0};
    for (int i=0;i<4;i++){
      for (int j=0;j<4;j++){
        float sum=0;
        for (int k=0;k<4;k++) sum += IminusKH[i][k]*P[k][j];
        newP[i][j]=sum;
      }
    }
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) P[i][j]=newP[i][j];
  }

  // ------------------------ Tape Line Updates ------------------------
  // Example: center line means Y = Y_line. So z = Y_line, h(x)=Y.
  // Innovation r = z - h(x) = Y_line - Y
  void updateLineY(float Y_line, float sigmaY) {
    const float r = Y_line - x[1];
    const float H[4] = {0, 1, 0, 0};
    updateScalar(r, H, sigmaY*sigmaY);
  }

  // Example: hog line means X = X_line. So z = X_line, h(x)=X.
  void updateLineX(float X_line, float sigmaX) {
    const float r = X_line - x[0];
    const float H[4] = {1, 0, 0, 0};
    updateScalar(r, H, sigmaX*sigmaX);
  }

  // ------------------------ IR Beacon Bearing Update (Optional) ------------------------
  // If you can estimate bearing alpha (radians) to a known beacon at (bx, by):
  // h(x) = wrapPi( atan2(by - Y, bx - X) - theta )
  // z = alpha_meas, so innovation r = wrapPi(z - h(x)).
  void updateBeaconBearing(float alpha_meas, float bx, float by, float sigmaAlpha) {
    const float dx = bx - x[0];
    const float dy = by - x[1];
    const float q = dx*dx + dy*dy;
    if (q < 1e-6f) return;

    const float bearing_world = atan2f(dy, dx);
    const float h = wrapPi(bearing_world - x[2]);
    const float r = wrapPi(alpha_meas - h);

    // H = ∂h/∂x = [ ∂/∂X atan2(dy,dx), ∂/∂Y atan2(dy,dx), ∂/∂theta(-1), ∂/∂b(0) ]
    // atan2 derivatives:
    // ∂bearing/∂X =  dy / (dx^2+dy^2) * (+1) ?  Actually bearing = atan2(dy, dx), dx = bx - X => ∂dx/∂X = -1
    // ∂bearing/∂X = ∂bearing/∂dx * ∂dx/∂X = (-dy/q)*(-1) = +dy/q
    // ∂bearing/∂Y: dy = by - Y, ∂dy/∂Y=-1 => ∂bearing/∂Y = (dx/q)*(-1) = -dx/q
    float H[4];
    H[0] =  dy / q;
    H[1] = -dx / q;
    H[2] = -1.0f;
    H[3] =  0.0f;

    updateScalar(r, H, sigmaAlpha*sigmaAlpha);
  }
};

// ------------------------ Example integration ------------------------

// You must provide these from your hardware:
float readGyroZ_rad_s();      // IMU gyro z (rad/s)
float readWheelDeltaL_m();    // meters since last loop
float readWheelDeltaR_m();    // meters since last loop
bool  detectCenterLine();     // your line sensor logic
bool  detectHogLine();        // your line sensor logic
float computeIRBearing_rad(); // optional: your IR processing
bool  hasIRBearing();         // optional

// Known field geometry (set your coordinate frame)
static const float Y_CENTERLINE = 0.0f;     // meters
static const float X_HOGLINE    = 0.50f;    // example
static const float BEACON_X     = 1.00f;    // example
static const float BEACON_Y     = 0.00f;    // example

EKF4 ekf;

unsigned long lastMicros = 0;

void setup() {
  Serial.begin(115200);

  // Optional: bias calibration at startup (robot stationary)
  // Average gyro for 1-2 seconds:
  float sum = 0;
  const int N = 200;
  for (int i=0;i<N;i++){
    sum += readGyroZ_rad_s();
    delay(5);
  }
  ekf.x[3] = sum / N; // initial gyro bias estimate

  lastMicros = micros();
}

void loop() {
  const unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0) dt = 1e-3f;
  lastMicros = now;

  // --- Read inputs
  const float dL = readWheelDeltaL_m();
  const float dR = readWheelDeltaR_m();
  const float gyroZ = readGyroZ_rad_s();

  // --- EKF predict
  ekf.predict(dL, dR, gyroZ, dt);

  // --- EKF updates from tape events (event-based = robust)
  if (detectCenterLine()) {
    ekf.updateLineY(Y_CENTERLINE, /*sigmaY=*/0.01f); // 1 cm
  }
  if (detectHogLine()) {
    ekf.updateLineX(X_HOGLINE, /*sigmaX=*/0.015f); // 1.5 cm
  }

  // --- Optional IR bearing update
  if (hasIRBearing()) {
    float alpha = computeIRBearing_rad();
    ekf.updateBeaconBearing(alpha, BEACON_X, BEACON_Y, /*sigmaAlpha=*/(5.0f*M_PI/180.0f));
  }

  // --- Print pose occasionally
  static int ctr=0;
  if (++ctr % 20 == 0) {
    Serial.print("X "); Serial.print(ekf.x[0], 4);
    Serial.print("  Y "); Serial.print(ekf.x[1], 4);
    Serial.print("  th "); Serial.print(ekf.x[2]*180.0f/M_PI, 2);
    Serial.print(" deg  b_g "); Serial.println(ekf.x[3], 5);
  }
}

/* ------------------------ Notes / Tuning Cheatsheet ------------------------

1) Units:
   - Use meters for X,Y and wheel deltas.
   - Use radians for theta, rad/s for gyro.

2) If your encoder deltas are noisy, increase ekf.q_xy and/or initial P.
   If your heading drifts too much between tape hits, increase tape update frequency
   and use updateLineY / updateLineX.

3) Key parameters to tune:
   - ekf.q_bg (gyro bias random walk): too small => bias won’t adapt; too big => bias chases noise.
   - sigmaX/sigmaY for tape: smaller => stronger snap-to-line; too small can cause jumps if detection is jittery.
   - sigmaAlpha for IR bearing: keep it large unless your bearing estimate is very reliable.

4) If you don’t have a bearing estimate from IR yet:
   - Skip updateBeaconBearing and rely on tape updates.
   - Later, add IR update as "heading alignment" when you intentionally point at beacon.

-------------------------------------------------------------------------- */

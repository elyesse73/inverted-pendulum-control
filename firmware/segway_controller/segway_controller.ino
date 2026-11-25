// ============================================================================
// Self-Stabilizing Inverted Pendulum Robot (Segway) - Real-Time Controller
// Control loop: 200 Hz (5 ms period) over 400 kHz I2C (Qwiic Mux)
// Sensors:      SparkFun BMI270 6-axis IMU + Dual AS5600 magnetic encoders
// Control law:  3-state Luenberger observer + 4-state feedback U = -K*(x - x_ref)
//               with acceleration/velocity-bounded trajectory generation
// ============================================================================

#include "AS5600.h"
#include "MecatroUtils.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// --- Hardware & Telemetry Configuration ---
#define WIFI_SSID "ArduinoMecatro6"
#define WIFI_PASSWRD "password11"

constexpr unsigned int kControlLoopPeriodMs = 5;  // 200 Hz
constexpr double kDt = static_cast<double>(kControlLoopPeriodMs) / 1000.0;
constexpr uint8_t kMuxAddress = 0x70;
constexpr uint8_t kImuMuxPort = 0;
constexpr uint8_t kRightEncoderPort = 3;
constexpr uint8_t kLeftEncoderPort = 4;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint32_t kSerialBaud = 1000000;

// --- Physical Parameters & Constants ---
constexpr double kWheelRadiusM = 0.04;          // rho [m]
constexpr double kImuHeightM = 0.176974;        // d_i [m]
constexpr double kGravity = 9.81;               // g [m/s^2]
constexpr double kSupplyVoltageV = 12.0;        // Nominal battery voltage [V]
constexpr double kDegToRad = 0.017453292519943; // pi / 180

// --- Operating Mode & Trajectory Bounds ---
// false: vertical stabilization around origin; true: velocity/position tracking
constexpr bool kEnableTrajectoryTracking = false;
constexpr double kUserVelocityMps = 0.10;       // Desired forward velocity [m/s]
constexpr double kMaxVelocityMps = 0.10;        // V_max [m/s]
constexpr double kMaxAccelMps2 = 0.25;          // A_max [m/s^2]

// --- Luenberger Observer Matrices: z = [p, xi, theta]^T ---
constexpr double kObsA[3][3] = {
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, kGravity },
  { 0.0, 0.0, 0.0 }
};
constexpr double kObsB[3][2] = {
  { -kImuHeightM, 0.0 },
  { 0.0,          1.0 },
  { 1.0,          0.0 }
};
constexpr double kObsC[3] = { 1.0 / kWheelRadiusM, 0.0, -1.0 };

// Hardware-validated observer and state-feedback gains (see matlab/control_synthesis.m)
constexpr double kObserverGainL[3] = { 1.3, 11.2, 3.3 };
constexpr double kStateFeedbackK[4] = { -1.5, -80.0, -80.482, -17.112 };

// Optional yaw PD gains (disabled by default; set non-zero to activate differential yaw control)
constexpr double kYawKp = 0.0;
constexpr double kYawKd = 0.0;

// --- Hardware Drivers ---
BMI270 imu;
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);
QWIICMUX multiplexer;

// --- Runtime State ---
double rightAngleZeroRad = 0.0;
double leftAngleZeroRad = 0.0;

double gyroPitchRadPerSec = 0.0;  // omega [rad/s]
double gyroYawRadPerSec = 0.0;    // r [rad/s]
double accelLongitudinal = 0.0;   // BMI270 raw Y acceleration (in g, matched to empirical L)
double meanWheelAngleRad = 0.0;   // y_e = (alpha_l + alpha_r) / 2 [rad]

double observerState[3] = { 0.0, 0.0, 0.0 };  // [p_hat, xi_hat, theta_hat]
double state[4] = { 0.0, 0.0, 0.0, 0.0 };     // [p, u, theta, omega]
double targetState[4] = { 0.0, 0.0, 0.0, 0.0 };

double cmdVelocityMps = 0.0;
double refPositionM = 0.0;
double yawAngleRad = 0.0;

static inline double clampValue(double value, double low, double high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static void readSensors() {
  multiplexer.setPort(kImuMuxPort);
  imu.getSensorData();

  // Frame convention: Y-forward, Z-up -> pitch rate on -X, yaw rate on +Z.
  // Note: SparkFun BMI270 reports accelY in [g]; observer gains L were tuned empirically
  // with this scaling on the physical robot.
  accelLongitudinal = static_cast<double>(imu.data.accelY);
  gyroPitchRadPerSec = -static_cast<double>(imu.data.gyroX) * kDegToRad;
  gyroYawRadPerSec = static_cast<double>(imu.data.gyroZ) * kDegToRad;

  multiplexer.setPort(kRightEncoderPort);
  const double rightAngleRad =
      rightEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS - rightAngleZeroRad;

  multiplexer.setPort(kLeftEncoderPort);
  const double leftAngleRad =
      leftEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS - leftAngleZeroRad;

  meanWheelAngleRad = 0.5 * (rightAngleRad + leftAngleRad);
}

static void updateLuenbergerObserver() {
  const double obsInput[2] = { gyroPitchRadPerSec, accelLongitudinal };

  double outputEstimate = 0.0;
  for (int i = 0; i < 3; ++i) {
    outputEstimate += kObsC[i] * observerState[i];
  }
  const double innovation = meanWheelAngleRad - outputEstimate;

  double stateDot[3] = { 0.0, 0.0, 0.0 };
  for (int i = 0; i < 3; ++i) {
    double pred = 0.0;
    for (int j = 0; j < 3; ++j) {
      pred += kObsA[i][j] * observerState[j];
    }
    for (int j = 0; j < 2; ++j) {
      pred += kObsB[i][j] * obsInput[j];
    }
    stateDot[i] = pred + kObserverGainL[i] * innovation;
  }

  // Forward Euler integration at 200 Hz
  for (int i = 0; i < 3; ++i) {
    observerState[i] += stateDot[i] * kDt;
  }
}

static void mapObserverToControlState() {
  const double pHat = observerState[0];
  const double xiHat = observerState[1];
  const double thetaHat = observerState[2];

  // Since xi = u + d_i * omega, recover longitudinal velocity u = xi - d_i * omega
  state[0] = pHat;
  state[1] = xiHat - kImuHeightM * gyroPitchRadPerSec;
  state[2] = thetaHat;
  state[3] = gyroPitchRadPerSec;
}

static void updateReferenceTrajectory() {
  const double dv = kUserVelocityMps - cmdVelocityMps;
  const double dvClamped = clampValue(dv, -kMaxAccelMps2 * kDt, kMaxAccelMps2 * kDt);
  cmdVelocityMps = clampValue(cmdVelocityMps + dvClamped, -kMaxVelocityMps, kMaxVelocityMps);
  refPositionM += cmdVelocityMps * kDt;
  yawAngleRad += gyroYawRadPerSec * kDt;

  if (kEnableTrajectoryTracking) {
    targetState[0] = refPositionM;
    targetState[1] = cmdVelocityMps;
  } else {
    targetState[0] = 0.0;
    targetState[1] = 0.0;
  }
  targetState[2] = 0.0;
  targetState[3] = 0.0;
}

static double computeStateFeedbackVoltage() {
  double controlSumV = 0.0;
  for (int i = 0; i < 4; ++i) {
    controlSumV += -kStateFeedbackK[i] * (state[i] - targetState[i]);
  }
  return controlSumV;
}

static void sendTelemetry() {
  mecatro::log(0, state[0]);  // p [m]
  mecatro::log(1, state[1]);  // u [m/s]
  mecatro::log(2, state[2]);  // theta [rad]
  mecatro::log(3, state[3]);  // omega [rad/s]
}

void setup() {
  Serial.begin(kSerialBaud);
  Wire1.begin();

  if (!multiplexer.begin(kMuxAddress, Wire1)) {
    Serial.println("Error: I2C multiplexer (0x70) not found.");
    return;
  }

  bool initialized = true;

  multiplexer.setPort(kImuMuxPort);
  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK) {
    Serial.println("Error: failed to initialize BMI270 IMU.");
    initialized = false;
  }

  multiplexer.setPort(kRightEncoderPort);
  rightEncoder.begin();
  if (!rightEncoder.isConnected()) {
    Serial.println("Error: right AS5600 encoder not connected.");
    initialized = false;
  }

  multiplexer.setPort(kLeftEncoderPort);
  leftEncoder.begin();
  if (!leftEncoder.isConnected()) {
    Serial.println("Error: left AS5600 encoder not connected.");
    initialized = false;
  }

  if (!initialized) {
    return;
  }

  Wire1.setClock(kI2cClockHz);

  constexpr unsigned int kNumTelemetryVars = 4;
  String variableNames[kNumTelemetryVars] = { "p", "u", "theta", "omega" };
  mecatro::initTelemetry(
      WIFI_SSID, WIFI_PASSWRD, kNumTelemetryVars, variableNames, kControlLoopPeriodMs);
  mecatro::configureArduino(kControlLoopPeriodMs);

  // Zero encoder baselines at upright startup
  multiplexer.setPort(kImuMuxPort);
  imu.getSensorData();

  multiplexer.setPort(kRightEncoderPort);
  rightAngleZeroRad = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;

  multiplexer.setPort(kLeftEncoderPort);
  leftAngleZeroRad = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop() {
  readSensors();
  updateLuenbergerObserver();
  mapObserverToControlState();
  updateReferenceTrajectory();

  const double uSumV = computeStateFeedbackVoltage();
  const double uDiffV = -kYawKp * yawAngleRad - kYawKd * gyroYawRadPerSec;

  const double uLeftV = 0.5 * (uSumV - uDiffV);
  const double uRightV = 0.5 * (uSumV + uDiffV);

  const float dutyLeft = static_cast<float>(clampValue(-uLeftV / kSupplyVoltageV, -1.0, 1.0));
  const float dutyRight = static_cast<float>(clampValue(-uRightV / kSupplyVoltageV, -1.0, 1.0));

  mecatro::setMotorDutyCycle(dutyLeft, dutyRight);
  sendTelemetry();
}
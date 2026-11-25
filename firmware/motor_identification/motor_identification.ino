// ============================================================================
// Multi-Excitation Motor System Identification Firmware
// Applies 6 sequential 3-second excitation signals (step, square, sine,
// sawtooth, 0.2-3.0 Hz chirp, and 16-bit LFSR PRBS) at 200 Hz while logging
// wheel angles and duty cycle over WiFi telemetry for MATLAB tfest().
// ============================================================================

#include <Wire.h>
#include "AS5600.h"
#include "MecatroUtils.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

#define WIFI_SSID "ArduinoMecatro6"
#define WIFI_PASSWRD "password11"

constexpr unsigned int kControlLoopPeriodMs = 5;  // 200 Hz
constexpr uint8_t kMuxAddress = 0x70;
constexpr uint8_t kImuPort = 0;
constexpr uint8_t kRightEncoderPort = 3;
constexpr uint8_t kLeftEncoderPort = 4;

// Excitation profile parameters (6 segments x 3 s = 18 s total)
constexpr float kMaxDutyAmplitude = 0.5f;
constexpr float kSegmentDurationSec = 3.0f;
constexpr float kBaseFreqHz = 0.5f;
constexpr float kChirpStartHz = 0.2f;
constexpr float kChirpEndHz = 3.0f;
constexpr unsigned long kPrbsStepMs = 150;

BMI270 imu;
QWIICMUX multiplexer;
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);

void setup() {
  Serial.begin(1000000);
  Wire1.begin();

  if (!multiplexer.begin(kMuxAddress, Wire1)) {
    Serial.println("Error: I2C multiplexer (0x70) not found.");
    return;
  }

  bool initialized = true;

  multiplexer.setPort(kRightEncoderPort);
  rightEncoder.begin();
  if (!rightEncoder.isConnected()) {
    Serial.println("Error: right encoder not connected.");
    initialized = false;
  }

  multiplexer.setPort(kLeftEncoderPort);
  leftEncoder.begin();
  if (!leftEncoder.isConnected()) {
    Serial.println("Error: left encoder not connected.");
    initialized = false;
  }

  multiplexer.setPort(kImuPort);
  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK) {
    Serial.println("Error: BMI270 IMU not initialized.");
    initialized = false;
  }

  if (!initialized) {
    return;
  }

  Wire1.setClock(400000);
  mecatro::configureArduino(kControlLoopPeriodMs);

  constexpr unsigned int kNumVars = 3;
  String variableNames[kNumVars] = { "LeftPos", "RightPos", "uDuty" };
  mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, kNumVars, variableNames, kControlLoopPeriodMs);
}

void loop() {
  mecatro::run();
}

static float generateExcitationDuty(unsigned long nowMs) {
  static unsigned long startMs = nowMs;
  static uint16_t lfsr = 0xACE1u;  // 16-bit maximal-length LFSR (taps 16, 14, 13, 11)
  static float uPrbs = 0.0f;
  static unsigned long lastPrbsMs = 0;

  const float tSec = 0.001f * static_cast<float>(nowMs - startMs);
  const int segment = static_cast<int>(tSec / kSegmentDurationSec);
  const float tau = tSec - static_cast<float>(segment) * kSegmentDurationSec;

  float duty = 0.0f;
  switch (segment) {
    case 0:  // 1) Step
      duty = kMaxDutyAmplitude;
      break;

    case 1:  // 2) Square wave
      duty = (sinf(2.0f * PI * kBaseFreqHz * tau) >= 0.0f) ? kMaxDutyAmplitude : -kMaxDutyAmplitude;
      break;

    case 2:  // 3) Sine wave
      duty = kMaxDutyAmplitude * sinf(2.0f * PI * kBaseFreqHz * tau);
      break;

    case 3: {  // 4) Sawtooth in [-A, A]
      const float phase = fmodf(tau * kBaseFreqHz, 1.0f);
      duty = kMaxDutyAmplitude * (2.0f * phase - 1.0f);
      break;
    }

    case 4: {  // 5) Linear frequency chirp (0.2 Hz -> 3.0 Hz)
      const float freq = kChirpStartHz + (kChirpEndHz - kChirpStartHz) * (tau / kSegmentDurationSec);
      duty = kMaxDutyAmplitude * sinf(2.0f * PI * freq * tau);
      break;
    }

    default:  // 6) Pseudo-Random Binary Sequence (PRBS)
      if (nowMs - lastPrbsMs >= kPrbsStepMs) {
        lastPrbsMs = nowMs;
        const unsigned lsb = (lfsr ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
        lfsr = static_cast<uint16_t>((lfsr >> 1) | (lsb << 15));
        uPrbs = (lfsr & 1u) ? kMaxDutyAmplitude : -kMaxDutyAmplitude;
      }
      duty = uPrbs;
      break;
  }

  return constrain(duty, -1.0f, 1.0f);
}

void mecatro::controlLoop() {
  multiplexer.setPort(kLeftEncoderPort);
  const float leftPosDeg = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

  multiplexer.setPort(kRightEncoderPort);
  const float rightPosDeg = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

  const float uDuty = generateExcitationDuty(millis());
  mecatro::setMotorDutyCycle(uDuty, uDuty);

  mecatro::log(0, leftPosDeg);
  mecatro::log(1, rightPosDeg);
  mecatro::log(2, uDuty);
}
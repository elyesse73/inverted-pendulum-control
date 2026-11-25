// ============================================================================
// BMI270 IMU Bring-Up & Static Gyroscope Bias Calibration (Qwiic I2C Mux)
// Averages 200 stationary samples (~2 s) to estimate gyroscope bias, then
// streams bias-compensated 6-axis IMU measurements at ~50 Hz over Serial.
// ============================================================================

#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

constexpr uint8_t kMuxAddress = 0x70;
constexpr uint8_t kImuMuxPort = 0;
constexpr uint32_t kSerialBaud = 1000000;
constexpr uint16_t kCalibrationSamples = 200;

BMI270 imu;
QWIICMUX mux;

float gxBiasDps = 0.0f;
float gyBiasDps = 0.0f;
float gzBiasDps = 0.0f;

static void printHeader() {
  Serial.println(F("ax[g]\tay[g]\taz[g]\tgx[dps]\tgy[dps]\tgz[dps]"));
}

void setup() {
  Serial.begin(kSerialBaud);
  while (!Serial) {
    ;
  }

  Wire1.begin();
  if (!mux.begin(kMuxAddress, Wire1)) {
    Serial.println(F("ERROR: I2C multiplexer (0x70) not found. Check wiring."));
    while (true) {
    }
  }

  mux.setPort(kImuMuxPort);
  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK) {
    Serial.print(F("ERROR: BMI270 failed to initialize on MUX port "));
    Serial.println(kImuMuxPort);
    while (true) {
    }
  }

  Serial.print(F("OK: BMI270 initialized on MUX port "));
  Serial.println(kImuMuxPort);
  Serial.println(F("Keep the robot stationary for 2 s to calibrate gyroscope bias..."));

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  for (uint16_t i = 0; i < kCalibrationSamples; ++i) {
    mux.setPort(kImuMuxPort);
    imu.getSensorData();
    sumX += imu.data.gyroX;
    sumY += imu.data.gyroY;
    sumZ += imu.data.gyroZ;
    delay(10);
  }

  gxBiasDps = sumX / static_cast<float>(kCalibrationSamples);
  gyBiasDps = sumY / static_cast<float>(kCalibrationSamples);
  gzBiasDps = sumZ / static_cast<float>(kCalibrationSamples);

  Serial.print(F("Estimated gyro bias [dps]: gx="));
  Serial.print(gxBiasDps, 3);
  Serial.print(F(" gy="));
  Serial.print(gyBiasDps, 3);
  Serial.print(F(" gz="));
  Serial.println(gzBiasDps, 3);
  printHeader();
}

void loop() {
  mux.setPort(kImuMuxPort);
  imu.getSensorData();

  const float ax = imu.data.accelX;
  const float ay = imu.data.accelY;
  const float az = imu.data.accelZ;
  const float gx = imu.data.gyroX - gxBiasDps;
  const float gy = imu.data.gyroY - gyBiasDps;
  const float gz = imu.data.gyroZ - gzBiasDps;

  Serial.print(ax, 3); Serial.print('\t');
  Serial.print(ay, 3); Serial.print('\t');
  Serial.print(az, 3); Serial.print('\t');
  Serial.print(gx, 2); Serial.print('\t');
  Serial.print(gy, 2); Serial.print('\t');
  Serial.println(gz, 2);

  delay(20);  // ~50 Hz
}
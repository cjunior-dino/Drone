#pragma once

namespace imu {

struct Sample {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temperatureC;
  float rollDeg;
  float pitchDeg;
};

bool begin();
bool calibrateGyroscope();
bool update(float deltaSeconds, Sample &sample);

}  // namespace imu

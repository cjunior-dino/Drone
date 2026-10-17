#pragma once

#include <Arduino.h>

namespace motor_output {

struct Status {
  bool signalActive;
  bool driveEnabled;
  float requestedThrottle;
  uint16_t pulseUs;
  uint16_t limitUs;
};

bool begin();
void update(bool outputEnabled, bool receiverSignalValid,
            float requestedThrottle, float deltaSeconds);
void emergencyStop();
Status getStatus();

}  // namespace motor_output

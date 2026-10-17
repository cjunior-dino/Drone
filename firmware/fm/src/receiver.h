#pragma once

#include <Arduino.h>

namespace receiver {

struct Frame {
  uint32_t channel1Us;
  uint32_t channel2Us;
  uint32_t channel3Us;
  uint32_t channel4Us;
  bool signalValid;
};

struct Diagnostics {
  uint32_t rejectedChannel1;
  uint32_t rejectedChannel2;
  uint32_t rejectedChannel3;
  uint32_t rejectedChannel4;
  uint32_t validChannel1;
  uint32_t validChannel2;
  uint32_t validChannel3;
  uint32_t validChannel4;
  uint32_t shortChannel1;
  uint32_t shortChannel2;
  uint32_t shortChannel3;
  uint32_t shortChannel4;
  uint32_t longChannel1;
  uint32_t longChannel2;
  uint32_t longChannel3;
  uint32_t longChannel4;
  uint32_t orphanFallChannel1;
  uint32_t orphanFallChannel2;
  uint32_t orphanFallChannel3;
  uint32_t orphanFallChannel4;
  uint32_t failsafeEvents;
  uint32_t recoveryEvents;
};

void begin();
void update();
Frame getFrame();
Diagnostics getDiagnostics();

}  // namespace receiver

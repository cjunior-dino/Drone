#pragma once

#include <Arduino.h>

#include "control_commands.h"

namespace safety {

enum class FlightState {
  Disarmed,
  Arming,
  BenchArmed,
  Disarming,
  FailsafeLockout,
};

void update(bool receiverSignalValid, const control::Commands &commands,
            uint32_t nowMs);
FlightState flightState();
const char *flightStateName();
uint32_t stateElapsedMs(uint32_t nowMs);
bool outputsEnabled();
[[noreturn]] void stopWithError(const char *message);

}  // namespace safety

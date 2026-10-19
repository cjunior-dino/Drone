#include "safety.h"

#include <Arduino.h>

#include "motor_output.h"

namespace safety {
namespace {

constexpr float kMaximumSafeThrottle = 0.02f;
constexpr float kMaximumCenteredCommand = 0.15f;
constexpr float kArmYawThreshold = 0.80f;
constexpr float kDisarmYawThreshold = -0.80f;
constexpr uint32_t kArmHoldMs = 1500;
constexpr uint32_t kDisarmHoldMs = 1000;
constexpr uint32_t kLockoutRecoveryHoldMs = 1000;

FlightState currentState = FlightState::Disarmed;
uint32_t stateStartedMs = 0;
bool receiverWasValid = false;
bool armReady = false;

bool elapsed(uint32_t nowMs, uint32_t durationMs) {
  return static_cast<uint32_t>(nowMs - stateStartedMs) >= durationMs;
}

void changeState(FlightState nextState, uint32_t nowMs) {
  if (currentState == nextState) {
    return;
  }
  currentState = nextState;
  stateStartedMs = nowMs;
  armReady = false;
}

bool throttleIsLow(const control::Commands &commands) {
  return commands.throttle <= kMaximumSafeThrottle;
}

bool centered(float command) {
  return command >= -kMaximumCenteredCommand &&
         command <= kMaximumCenteredCommand;
}

bool armGestureActive(const control::Commands &commands) {
  return throttleIsLow(commands) && centered(commands.roll) &&
         centered(commands.pitch) && commands.yaw >= kArmYawThreshold;
}

bool disarmGestureActive(const control::Commands &commands) {
  return throttleIsLow(commands) && commands.yaw <= kDisarmYawThreshold;
}

bool lockoutRecoveryPosition(const control::Commands &commands) {
  return throttleIsLow(commands) && centered(commands.roll) &&
         centered(commands.pitch) && centered(commands.yaw);
}

}  // namespace

void update(bool receiverSignalValid, const control::Commands &commands,
            uint32_t nowMs) {
  if (!receiverSignalValid) {
    if (receiverWasValid) {
      changeState(FlightState::FailsafeLockout, nowMs);
    }
    receiverWasValid = false;
    return;
  }

  const bool signalRecovered = !receiverWasValid;
  receiverWasValid = true;
  if (currentState == FlightState::FailsafeLockout && signalRecovered) {
    // A contagem de recuperacao comeca quando o sinal realmente retorna, nao
    // no instante anterior em que ele foi perdido.
    stateStartedMs = nowMs;
  }

  switch (currentState) {
    case FlightState::Disarmed:
      if (lockoutRecoveryPosition(commands)) {
        // Exige que os controles passem pela posicao segura antes de aceitar
        // qualquer gesto de armamento.
        armReady = true;
      } else if (armReady && armGestureActive(commands)) {
        changeState(FlightState::Arming, nowMs);
      }
      break;

    case FlightState::Arming:
      if (!armGestureActive(commands)) {
        changeState(FlightState::Disarmed, nowMs);
      } else if (elapsed(nowMs, kArmHoldMs)) {
        changeState(FlightState::BenchArmed, nowMs);
      }
      break;

    case FlightState::BenchArmed:
      if (disarmGestureActive(commands)) {
        changeState(FlightState::Disarming, nowMs);
      }
      break;

    case FlightState::Disarming:
      if (!disarmGestureActive(commands)) {
        changeState(FlightState::BenchArmed, nowMs);
      } else if (elapsed(nowMs, kDisarmHoldMs)) {
        changeState(FlightState::Disarmed, nowMs);
      }
      break;

    case FlightState::FailsafeLockout:
      if (!lockoutRecoveryPosition(commands)) {
        stateStartedMs = nowMs;
      } else if (elapsed(nowMs, kLockoutRecoveryHoldMs)) {
        changeState(FlightState::Disarmed, nowMs);
      }
      break;
  }
}

FlightState flightState() { return currentState; }

const char *flightStateName() {
  switch (currentState) {
    case FlightState::Disarmed:
      return "DISARMED";
    case FlightState::Arming:
      return "ARMING";
    case FlightState::BenchArmed:
      return "BENCH_ARMED";
    case FlightState::Disarming:
      return "DISARMING";
    case FlightState::FailsafeLockout:
      return "FAILSAFE_LOCKOUT";
  }
  return "UNKNOWN";
}

uint32_t stateElapsedMs(uint32_t nowMs) {
  return static_cast<uint32_t>(nowMs - stateStartedMs);
}

bool outputsEnabled() { return currentState == FlightState::BenchArmed; }

[[noreturn]] void stopWithError(const char *message) {
  currentState = FlightState::FailsafeLockout;
  motor_output::emergencyStop();
  while (true) {
    Serial.println();
    Serial.println(message);
    Serial.println(
        "Teste interrompido. Corrija a ligacao e pressione o botao EN/RESET.");
    Serial.flush();
    delay(1000);
  }
}

}  // namespace safety

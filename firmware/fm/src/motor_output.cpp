#include "motor_output.h"

namespace motor_output {
namespace {

constexpr uint8_t kMotor1Pin = 32;
constexpr uint8_t kPwmChannel = 0;
constexpr uint32_t kPwmFrequencyHz = 50;
constexpr uint8_t kPwmResolutionBits = 16;
constexpr uint32_t kPwmPeriodUs = 1000000UL / kPwmFrequencyHz;
constexpr uint16_t kMinimumPulseUs = 1000;
constexpr uint16_t kBenchLimitPulseUs = 1100;
constexpr float kRampUpUsPerSecond = 20.0f;

bool initialized = false;
bool driveEnabled = false;
float requestedThrottle = 0.0f;
float currentPulseUs = static_cast<float>(kMinimumPulseUs);

float clamp(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

void writePulseUs(uint16_t pulseUs) {
  const uint32_t maximumDuty = (1UL << kPwmResolutionBits) - 1UL;
  const uint32_t duty = static_cast<uint32_t>(
      (static_cast<uint64_t>(pulseUs) * maximumDuty) / kPwmPeriodUs);
  ledcWrite(kPwmChannel, duty);
}

}  // namespace

bool begin() {
  const double actualFrequency =
      ledcSetup(kPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
  if (actualFrequency <= 0.0) {
    return false;
  }

  ledcAttachPin(kMotor1Pin, kPwmChannel);
  initialized = true;
  emergencyStop();
  return true;
}

void update(bool outputEnabled, bool receiverSignalValid,
            float throttleRequest, float deltaSeconds) {
  requestedThrottle = clamp(throttleRequest, 0.0f, 1.0f);
  driveEnabled = initialized && outputEnabled && receiverSignalValid;

  if (!driveEnabled) {
    currentPulseUs = static_cast<float>(kMinimumPulseUs);
    if (initialized) {
      writePulseUs(kMinimumPulseUs);
    }
    return;
  }

  const float targetPulseUs =
      static_cast<float>(kMinimumPulseUs) +
      requestedThrottle *
          static_cast<float>(kBenchLimitPulseUs - kMinimumPulseUs);

  if (targetPulseUs <= currentPulseUs) {
    // Reducoes de potencia sao aplicadas imediatamente.
    currentPulseUs = targetPulseUs;
  } else {
    const float safeDeltaSeconds = clamp(deltaSeconds, 0.0f, 0.05f);
    currentPulseUs += kRampUpUsPerSecond * safeDeltaSeconds;
    if (currentPulseUs > targetPulseUs) {
      currentPulseUs = targetPulseUs;
    }
  }

  writePulseUs(static_cast<uint16_t>(currentPulseUs + 0.5f));
}

void emergencyStop() {
  driveEnabled = false;
  requestedThrottle = 0.0f;
  currentPulseUs = static_cast<float>(kMinimumPulseUs);
  if (initialized) {
    writePulseUs(kMinimumPulseUs);
  }
}

Status getStatus() {
  return {
      initialized,
      driveEnabled,
      requestedThrottle,
      static_cast<uint16_t>(currentPulseUs + 0.5f),
      kBenchLimitPulseUs,
  };
}

}  // namespace motor_output

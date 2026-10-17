#include "control_commands.h"

namespace control {
namespace {

constexpr uint32_t kMinPulseUs = 1000;
constexpr uint32_t kCenterPulseUs = 1500;
constexpr uint32_t kMaxPulseUs = 2000;
constexpr uint32_t kDeadbandUs = 25;

float clamp(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float normalizeCenteredChannel(uint32_t pulseUs) {
  const uint32_t lowerDeadbandUs = kCenterPulseUs - kDeadbandUs;
  const uint32_t upperDeadbandUs = kCenterPulseUs + kDeadbandUs;

  if (pulseUs >= lowerDeadbandUs && pulseUs <= upperDeadbandUs) {
    return 0.0f;
  }

  if (pulseUs < lowerDeadbandUs) {
    const float value =
        -static_cast<float>(lowerDeadbandUs - pulseUs) /
        static_cast<float>(lowerDeadbandUs - kMinPulseUs);
    return clamp(value, -1.0f, 0.0f);
  }

  const float value = static_cast<float>(pulseUs - upperDeadbandUs) /
                      static_cast<float>(kMaxPulseUs - upperDeadbandUs);
  return clamp(value, 0.0f, 1.0f);
}

float normalizeThrottle(uint32_t pulseUs) {
  const float value =
      (static_cast<float>(pulseUs) - static_cast<float>(kMinPulseUs)) /
      static_cast<float>(kMaxPulseUs - kMinPulseUs);
  return clamp(value, 0.0f, 1.0f);
}

}  // namespace

Commands fromReceiver(const receiver::Frame &receiverFrame) {
  if (!receiverFrame.signalValid) {
    return {0.0f, 0.0f, 0.0f, 0.0f};
  }

  // Mapeamento observado no FS-i6 em Mode 1:
  // CH1=roll, CH2=pitch, CH3=throttle e CH4=yaw.
  return {
      normalizeCenteredChannel(receiverFrame.channel1Us),
      normalizeCenteredChannel(receiverFrame.channel2Us),
      normalizeThrottle(receiverFrame.channel3Us),
      normalizeCenteredChannel(receiverFrame.channel4Us),
  };
}

}  // namespace control

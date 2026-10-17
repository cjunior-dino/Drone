#pragma once

#include "receiver.h"

namespace control {

struct Commands {
  float roll;
  float pitch;
  float throttle;
  float yaw;
};

Commands fromReceiver(const receiver::Frame &receiverFrame);

}  // namespace control

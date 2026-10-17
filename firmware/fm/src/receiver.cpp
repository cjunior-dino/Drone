#include "receiver.h"

namespace receiver {
namespace {

constexpr int kChannel1Pin = 27;
constexpr int kChannel2Pin = 26;
constexpr int kChannel3Pin = 25;
constexpr int kChannel4Pin = 33;
constexpr uint32_t kSignalTimeoutUs = 100000;
constexpr uint32_t kValidMinPulseUs = 900;
constexpr uint32_t kValidMaxPulseUs = 2100;
constexpr uint32_t kImmediateStepUs = 120;
constexpr uint32_t kCandidateToleranceUs = 40;
constexpr uint8_t kRequiredConfirmations = 2;

struct Snapshot {
  uint32_t pulseUs;
  uint32_t lastPulseUs;
  uint32_t sequence;
};

struct Filter {
  uint32_t pulseUs = 0;
  uint32_t candidatePulseUs = 0;
  uint32_t processedSequence = 0;
  uint8_t candidateConfirmations = 0;
  bool valid = false;
};

volatile uint32_t channel1RiseUs = 0;
volatile uint32_t channel1PulseUs = 0;
volatile uint32_t channel1LastPulseUs = 0;
volatile uint32_t channel1Sequence = 0;
volatile uint32_t channel1RejectedPulses = 0;
volatile uint32_t channel1ValidPulses = 0;
volatile uint32_t channel1ShortPulses = 0;
volatile uint32_t channel1LongPulses = 0;
volatile uint32_t channel1OrphanFalls = 0;
volatile bool channel1RisePending = false;
volatile uint32_t channel2RiseUs = 0;
volatile uint32_t channel2PulseUs = 0;
volatile uint32_t channel2LastPulseUs = 0;
volatile uint32_t channel2Sequence = 0;
volatile uint32_t channel2RejectedPulses = 0;
volatile uint32_t channel2ValidPulses = 0;
volatile uint32_t channel2ShortPulses = 0;
volatile uint32_t channel2LongPulses = 0;
volatile uint32_t channel2OrphanFalls = 0;
volatile bool channel2RisePending = false;
volatile uint32_t channel3RiseUs = 0;
volatile uint32_t channel3PulseUs = 0;
volatile uint32_t channel3LastPulseUs = 0;
volatile uint32_t channel3Sequence = 0;
volatile uint32_t channel3RejectedPulses = 0;
volatile uint32_t channel3ValidPulses = 0;
volatile uint32_t channel3ShortPulses = 0;
volatile uint32_t channel3LongPulses = 0;
volatile uint32_t channel3OrphanFalls = 0;
volatile bool channel3RisePending = false;
volatile uint32_t channel4RiseUs = 0;
volatile uint32_t channel4PulseUs = 0;
volatile uint32_t channel4LastPulseUs = 0;
volatile uint32_t channel4Sequence = 0;
volatile uint32_t channel4RejectedPulses = 0;
volatile uint32_t channel4ValidPulses = 0;
volatile uint32_t channel4ShortPulses = 0;
volatile uint32_t channel4LongPulses = 0;
volatile uint32_t channel4OrphanFalls = 0;
volatile bool channel4RisePending = false;
Filter filters[4];
uint32_t failsafeEvents = 0;
uint32_t recoveryEvents = 0;
bool hadValidSignal = false;
bool previousSignalValid = false;

void IRAM_ATTR handleChannel1Edge() {
  const uint32_t nowUs = micros();
  if (digitalRead(kChannel1Pin) == HIGH) {
    channel1RiseUs = nowUs;
    channel1RisePending = true;
    return;
  }

  if (!channel1RisePending) {
    ++channel1OrphanFalls;
    return;
  }
  channel1RisePending = false;
  const uint32_t pulseUs = static_cast<uint32_t>(nowUs - channel1RiseUs);
  if (pulseUs >= kValidMinPulseUs && pulseUs <= kValidMaxPulseUs) {
    channel1PulseUs = pulseUs;
    channel1LastPulseUs = nowUs;
    ++channel1Sequence;
    ++channel1ValidPulses;
  } else {
    ++channel1RejectedPulses;
    if (pulseUs < kValidMinPulseUs) {
      ++channel1ShortPulses;
    } else {
      ++channel1LongPulses;
    }
  }
}

void IRAM_ATTR handleChannel2Edge() {
  const uint32_t nowUs = micros();
  if (digitalRead(kChannel2Pin) == HIGH) {
    channel2RiseUs = nowUs;
    channel2RisePending = true;
    return;
  }

  if (!channel2RisePending) {
    ++channel2OrphanFalls;
    return;
  }
  channel2RisePending = false;
  const uint32_t pulseUs = static_cast<uint32_t>(nowUs - channel2RiseUs);
  if (pulseUs >= kValidMinPulseUs && pulseUs <= kValidMaxPulseUs) {
    channel2PulseUs = pulseUs;
    channel2LastPulseUs = nowUs;
    ++channel2Sequence;
    ++channel2ValidPulses;
  } else {
    ++channel2RejectedPulses;
    if (pulseUs < kValidMinPulseUs) {
      ++channel2ShortPulses;
    } else {
      ++channel2LongPulses;
    }
  }
}

void IRAM_ATTR handleChannel3Edge() {
  const uint32_t nowUs = micros();
  if (digitalRead(kChannel3Pin) == HIGH) {
    channel3RiseUs = nowUs;
    channel3RisePending = true;
    return;
  }

  if (!channel3RisePending) {
    ++channel3OrphanFalls;
    return;
  }
  channel3RisePending = false;
  const uint32_t pulseUs = static_cast<uint32_t>(nowUs - channel3RiseUs);
  if (pulseUs >= kValidMinPulseUs && pulseUs <= kValidMaxPulseUs) {
    channel3PulseUs = pulseUs;
    channel3LastPulseUs = nowUs;
    ++channel3Sequence;
    ++channel3ValidPulses;
  } else {
    ++channel3RejectedPulses;
    if (pulseUs < kValidMinPulseUs) {
      ++channel3ShortPulses;
    } else {
      ++channel3LongPulses;
    }
  }
}

void IRAM_ATTR handleChannel4Edge() {
  const uint32_t nowUs = micros();
  if (digitalRead(kChannel4Pin) == HIGH) {
    channel4RiseUs = nowUs;
    channel4RisePending = true;
    return;
  }

  if (!channel4RisePending) {
    ++channel4OrphanFalls;
    return;
  }
  channel4RisePending = false;
  const uint32_t pulseUs = static_cast<uint32_t>(nowUs - channel4RiseUs);
  if (pulseUs >= kValidMinPulseUs && pulseUs <= kValidMaxPulseUs) {
    channel4PulseUs = pulseUs;
    channel4LastPulseUs = nowUs;
    ++channel4Sequence;
    ++channel4ValidPulses;
  } else {
    ++channel4RejectedPulses;
    if (pulseUs < kValidMinPulseUs) {
      ++channel4ShortPulses;
    } else {
      ++channel4LongPulses;
    }
  }
}

uint32_t pulseDifference(uint32_t firstPulseUs, uint32_t secondPulseUs) {
  return firstPulseUs > secondPulseUs ? firstPulseUs - secondPulseUs
                                      : secondPulseUs - firstPulseUs;
}

void resetFilter(Filter &filter, uint32_t sequence) {
  filter.pulseUs = 0;
  filter.candidatePulseUs = 0;
  filter.processedSequence = sequence;
  filter.candidateConfirmations = 0;
  filter.valid = false;
}

void updateFilter(const Snapshot &snapshot, Filter &filter, uint32_t nowUs) {
  if (snapshot.lastPulseUs == 0 ||
      static_cast<uint32_t>(nowUs - snapshot.lastPulseUs) >
          kSignalTimeoutUs) {
    resetFilter(filter, snapshot.sequence);
    return;
  }

  if (snapshot.sequence == filter.processedSequence) {
    return;
  }
  filter.processedSequence = snapshot.sequence;

  // Verificacao defensiva: os pulsos fora da faixa ja sao descartados no ISR.
  if (snapshot.pulseUs < kValidMinPulseUs ||
      snapshot.pulseUs > kValidMaxPulseUs) {
    return;
  }

  if (!filter.valid) {
    if (filter.candidateConfirmations == 0 ||
        pulseDifference(snapshot.pulseUs, filter.candidatePulseUs) >
            kCandidateToleranceUs) {
      filter.candidatePulseUs = snapshot.pulseUs;
      filter.candidateConfirmations = 1;
      return;
    }

    ++filter.candidateConfirmations;
    filter.candidatePulseUs = snapshot.pulseUs;
    if (filter.candidateConfirmations >= kRequiredConfirmations) {
      filter.pulseUs = snapshot.pulseUs;
      filter.candidateConfirmations = 0;
      filter.valid = true;
    }
    return;
  }

  if (pulseDifference(snapshot.pulseUs, filter.pulseUs) <=
      kImmediateStepUs) {
    filter.pulseUs = snapshot.pulseUs;
    filter.candidateConfirmations = 0;
    return;
  }

  if (filter.candidateConfirmations == 0 ||
      pulseDifference(snapshot.pulseUs, filter.candidatePulseUs) >
          kCandidateToleranceUs) {
    filter.candidatePulseUs = snapshot.pulseUs;
    filter.candidateConfirmations = 1;
    return;
  }

  ++filter.candidateConfirmations;
  filter.candidatePulseUs = snapshot.pulseUs;
  if (filter.candidateConfirmations >= kRequiredConfirmations) {
    filter.pulseUs = snapshot.pulseUs;
    filter.candidateConfirmations = 0;
  }
}

bool allChannelsValid() {
  return filters[0].valid && filters[1].valid && filters[2].valid &&
         filters[3].valid;
}

}  // namespace

void begin() {
  pinMode(kChannel1Pin, INPUT);
  pinMode(kChannel2Pin, INPUT);
  pinMode(kChannel3Pin, INPUT);
  pinMode(kChannel4Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kChannel1Pin), handleChannel1Edge,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(kChannel2Pin), handleChannel2Edge,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(kChannel3Pin), handleChannel3Edge,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(kChannel4Pin), handleChannel4Edge,
                  CHANGE);
}

void update() {
  Snapshot snapshots[4];

  noInterrupts();
  snapshots[0] = {channel1PulseUs, channel1LastPulseUs, channel1Sequence};
  snapshots[1] = {channel2PulseUs, channel2LastPulseUs, channel2Sequence};
  snapshots[2] = {channel3PulseUs, channel3LastPulseUs, channel3Sequence};
  snapshots[3] = {channel4PulseUs, channel4LastPulseUs, channel4Sequence};
  interrupts();

  const uint32_t nowUs = micros();
  for (size_t index = 0; index < 4; ++index) {
    updateFilter(snapshots[index], filters[index], nowUs);
  }

  const bool signalValid = allChannelsValid();
  if (signalValid) {
    if (hadValidSignal && !previousSignalValid) {
      ++recoveryEvents;
    }
    hadValidSignal = true;
  } else if (hadValidSignal && previousSignalValid) {
    ++failsafeEvents;
  }
  previousSignalValid = signalValid;
}

Frame getFrame() {
  const bool signalValid = allChannelsValid();
  return {
      filters[0].valid ? filters[0].pulseUs : 0,
      filters[1].valid ? filters[1].pulseUs : 0,
      filters[2].valid ? filters[2].pulseUs : 0,
      filters[3].valid ? filters[3].pulseUs : 0,
      signalValid,
  };
}

Diagnostics getDiagnostics() {
  Diagnostics diagnostics{};

  noInterrupts();
  diagnostics.rejectedChannel1 = channel1RejectedPulses;
  diagnostics.rejectedChannel2 = channel2RejectedPulses;
  diagnostics.rejectedChannel3 = channel3RejectedPulses;
  diagnostics.rejectedChannel4 = channel4RejectedPulses;
  diagnostics.validChannel1 = channel1ValidPulses;
  diagnostics.validChannel2 = channel2ValidPulses;
  diagnostics.validChannel3 = channel3ValidPulses;
  diagnostics.validChannel4 = channel4ValidPulses;
  diagnostics.shortChannel1 = channel1ShortPulses;
  diagnostics.shortChannel2 = channel2ShortPulses;
  diagnostics.shortChannel3 = channel3ShortPulses;
  diagnostics.shortChannel4 = channel4ShortPulses;
  diagnostics.longChannel1 = channel1LongPulses;
  diagnostics.longChannel2 = channel2LongPulses;
  diagnostics.longChannel3 = channel3LongPulses;
  diagnostics.longChannel4 = channel4LongPulses;
  diagnostics.orphanFallChannel1 = channel1OrphanFalls;
  diagnostics.orphanFallChannel2 = channel2OrphanFalls;
  diagnostics.orphanFallChannel3 = channel3OrphanFalls;
  diagnostics.orphanFallChannel4 = channel4OrphanFalls;
  interrupts();

  diagnostics.failsafeEvents = failsafeEvents;
  diagnostics.recoveryEvents = recoveryEvents;
  return diagnostics;
}

}  // namespace receiver

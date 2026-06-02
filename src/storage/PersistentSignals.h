#pragma once

#include <Arduino.h>

static constexpr uint16_t PERSISTENT_RAW_BUFFER_LENGTH = 100;

struct PersistedSignal {
  uint16_t protocol;
  uint16_t address;
  uint16_t command;
  uint8_t bits;
  uint8_t isRaw;
  uint16_t rawCodeLength;
  uint8_t rawCode[PERSISTENT_RAW_BUFFER_LENGTH];
};

bool loadSignalsFromNvs(uint8_t &savedCount, PersistedSignal *signals, size_t maxSignals);
bool saveSignalsToNvs(uint8_t savedCount, const PersistedSignal *signals, size_t maxSignals);
bool clearSignalsInNvs();

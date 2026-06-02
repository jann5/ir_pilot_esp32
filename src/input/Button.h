#pragma once

#include <Arduino.h>

struct Button {
  uint8_t pin;
  bool useInternalPullup;
  bool activeLow;
  bool triggerOnBothEdges;
  bool stableLevel;
  bool lastLevel;
  uint32_t changedMs;
  uint32_t lastPressMs;
  bool pressEvent;
  bool pressedLatch;
};

void initButton(Button &b);
void updateButton(Button &b, uint32_t nowMs, uint32_t debounceMs, uint32_t edgeLockMs);
bool consumePress(Button &b);
bool isButtonActive(const Button &b);

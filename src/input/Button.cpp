#include "Button.h"

void initButton(Button &b) {
  pinMode(b.pin, b.useInternalPullup ? INPUT_PULLUP : INPUT);
  delay(1);
  const bool level = digitalRead(b.pin);
  b.stableLevel = level;
  b.lastLevel = level;
  b.changedMs = millis();
  b.lastPressMs = 0;
  b.pressEvent = false;
  b.pressedLatch = b.activeLow ? (level == LOW) : (level == HIGH);
}

void updateButton(Button &b, uint32_t nowMs, uint32_t debounceMs, uint32_t edgeLockMs) {
  const bool raw = digitalRead(b.pin);

  if (raw != b.lastLevel) {
    b.lastLevel = raw;
    b.changedMs = nowMs;
  }

  if ((nowMs - b.changedMs) < debounceMs || raw == b.stableLevel) {
    return;
  }

  b.stableLevel = raw;

  if (b.triggerOnBothEdges) {
    if ((nowMs - b.lastPressMs) >= edgeLockMs) {
      b.lastPressMs = nowMs;
      b.pressEvent = true;
    }
    return;
  }

  const bool active = b.activeLow ? (b.stableLevel == LOW) : (b.stableLevel == HIGH);
  if (!active) {
    b.pressedLatch = false;
    return;
  }

  if (!b.pressedLatch && (nowMs - b.lastPressMs) >= edgeLockMs) {
    b.lastPressMs = nowMs;
    b.pressEvent = true;
    b.pressedLatch = true;
  }
}

bool consumePress(Button &b) {
  if (b.pressEvent) {
    b.pressEvent = false;
    return true;
  }
  return false;
}

bool isButtonActive(const Button &b) {
  return b.activeLow ? (b.stableLevel == LOW) : (b.stableLevel == HIGH);
}

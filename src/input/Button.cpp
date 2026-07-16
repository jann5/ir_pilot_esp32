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
}

void updateButton(Button &b, uint32_t nowMs, uint32_t debounceMs, uint32_t edgeLockMs) {
  const bool raw = digitalRead(b.pin);
  if (raw != b.lastLevel) {
    b.lastLevel = raw;
    b.changedMs = nowMs;
  }

  if ((nowMs - b.changedMs) >= debounceMs && raw != b.stableLevel) {
    const bool previous = b.stableLevel;
    b.stableLevel = raw;

    bool isPress = false;
    if (b.triggerOnBothEdges) {
      isPress = true;
    } else if (b.activeLow) {
      isPress = (previous == HIGH && b.stableLevel == LOW);
    } else {
      isPress = (previous == LOW && b.stableLevel == HIGH);
    }

    if (isPress && (nowMs - b.lastPressMs) >= edgeLockMs) {
      b.lastPressMs = nowMs;
      b.pressEvent = true;
    }
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

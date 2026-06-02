#pragma once

#include <Arduino.h>

enum class TvBgoneRegion : uint8_t {
  NA = 0,
  EU = 1,
};

class UniversalTvBlaster {
public:
  void begin(TvBgoneRegion region = TvBgoneRegion::NA);

  void setRegion(TvBgoneRegion region);
  TvBgoneRegion getRegion() const;
  const char *regionLabel() const;
  void setCodeLimit(uint16_t limit);
  uint16_t codeLimit() const;
  void setLooping(bool enabled);
  bool looping() const;

  void start(uint16_t startIndex = 0);
  void stop();
  void tick();
  bool sendSingle(uint16_t index);

  bool isRunning() const;
  bool isFinished() const;

  uint16_t totalCodes() const;
  uint16_t sentCodes() const;
  uint16_t nextCodeIndex() const;
  uint16_t lastSentCodeIndex() const;

private:
  bool sendCodeByIndex(uint16_t index);

  TvBgoneRegion region_ = TvBgoneRegion::NA;
  bool running_ = false;
  bool finished_ = false;
  bool looping_ = true;
  uint16_t codeLimit_ = 0; // 0 = pelna baza
  uint16_t codeIndex_ = 0;
  uint16_t sentCodes_ = 0;
  uint16_t lastSentCode_ = 0;
  uint32_t nextSendMs_ = 0;
};

#include "irblast/UniversalTvBlaster.h"

#include "irblast/TvBgoneTypes.h"
#include "irblast/WorldIrCodes.h"

class IRrecv {
public:
  void restartAfterSend();
};

class IRsend {
public:
  void sendRaw(const uint16_t aBufferWithMicroseconds[], uint_fast16_t aLengthOfBuffer,
               uint_fast8_t aIRFrequencyKilohertz);
};

extern IRrecv IrReceiver;
extern IRsend IrSender;

namespace {
constexpr uint32_t TVBG_INTER_CODE_MS = 0;
constexpr uint16_t TVBG_RAW_BUFFER_CAPACITY = 300;

const IrCode *const *getCodeTable(TvBgoneRegion region) {
  return (region == TvBgoneRegion::NA) ? NApowerCodes : EUpowerCodes;
}

uint16_t getCodeCount(TvBgoneRegion region) {
  return (region == TvBgoneRegion::NA) ? num_NAcodes : num_EUcodes;
}

uint16_t getEffectiveCodeCount(TvBgoneRegion region, uint16_t limit) {
  const uint16_t all = getCodeCount(region);
  if (limit == 0 || limit >= all) {
    return all;
  }
  return limit;
}

uint8_t readPackedBits(const uint8_t *&codePtr, uint8_t &bitsLeft, uint8_t &bitsBuf, uint8_t count) {
  uint8_t out = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (bitsLeft == 0) {
      bitsBuf = pgm_read_byte(codePtr++);
      bitsLeft = 8;
    }
    bitsLeft--;
    out |= (((bitsBuf >> bitsLeft) & 0x01U) << (count - 1U - i));
  }
  return out;
}

uint16_t tenUsToMicros(uint16_t tenUs) {
  uint32_t us = static_cast<uint32_t>(tenUs) * 10UL;
  if (us == 0) {
    return 1;
  }
  if (us > 65000UL) {
    return 65000;
  }
  return static_cast<uint16_t>(us);
}

uint8_t timerToCarrierKHz(uint8_t timerVal) {
  if (timerVal == 0) {
    return 38;
  }
  const uint32_t hz = 2000000UL / (static_cast<uint32_t>(timerVal) + 1UL);
  uint32_t khz = (hz + 500UL) / 1000UL;
  if (khz < 30UL) {
    khz = 30UL;
  }
  if (khz > 60UL) {
    khz = 60UL;
  }
  return static_cast<uint8_t>(khz);
}
} // namespace

void UniversalTvBlaster::begin(TvBgoneRegion region) {
  setRegion(region);
}

void UniversalTvBlaster::setRegion(TvBgoneRegion region) {
  region_ = region;
  stop();
}

TvBgoneRegion UniversalTvBlaster::getRegion() const {
  return region_;
}

const char *UniversalTvBlaster::regionLabel() const {
  return (region_ == TvBgoneRegion::NA) ? "NA/ASIA" : "EU";
}

void UniversalTvBlaster::setCodeLimit(uint16_t limit) {
  codeLimit_ = limit;
  const uint16_t count = totalCodes();
  if (count == 0) {
    codeIndex_ = 0;
  } else if (codeIndex_ >= count) {
    codeIndex_ = 0;
  }
}

uint16_t UniversalTvBlaster::codeLimit() const {
  return codeLimit_;
}

void UniversalTvBlaster::setLooping(bool enabled) {
  looping_ = enabled;
}

bool UniversalTvBlaster::looping() const {
  return looping_;
}

void UniversalTvBlaster::start(uint16_t startIndex) {
  const uint16_t count = totalCodes();
  if (count == 0) {
    running_ = false;
    finished_ = true;
    sentCodes_ = 0;
    codeIndex_ = 0;
    lastSentCode_ = 0;
    return;
  }

  if (startIndex >= count) {
    startIndex = 0;
  }

  running_ = true;
  finished_ = false;
  sentCodes_ = 0;
  codeIndex_ = startIndex;
  lastSentCode_ = startIndex;
  nextSendMs_ = millis();
}

void UniversalTvBlaster::stop() {
  running_ = false;
}

bool UniversalTvBlaster::isRunning() const {
  return running_;
}

bool UniversalTvBlaster::isFinished() const {
  return finished_;
}

uint16_t UniversalTvBlaster::totalCodes() const {
  return getEffectiveCodeCount(region_, codeLimit_);
}

uint16_t UniversalTvBlaster::sentCodes() const {
  return sentCodes_;
}

uint16_t UniversalTvBlaster::nextCodeIndex() const {
  return codeIndex_;
}

uint16_t UniversalTvBlaster::lastSentCodeIndex() const {
  return lastSentCode_;
}

void UniversalTvBlaster::tick() {
  if (!running_) {
    return;
  }

  const uint32_t nowMs = millis();
  if (static_cast<int32_t>(nowMs - nextSendMs_) < 0) {
    return;
  }

  const uint16_t count = totalCodes();
  if (codeIndex_ >= count) {
    running_ = false;
    finished_ = true;
    return;
  }

  const bool ok = sendCodeByIndex(codeIndex_);
  if (ok) {
    lastSentCode_ = codeIndex_;
    sentCodes_++;
  }

  codeIndex_++;
  if (codeIndex_ >= count) {
    finished_ = true;
    if (looping_) {
      codeIndex_ = 0;
      sentCodes_ = 0;
    } else {
      running_ = false;
    }
  }

  nextSendMs_ = millis() + TVBG_INTER_CODE_MS;
}

bool UniversalTvBlaster::sendSingle(uint16_t index) {
  return sendCodeByIndex(index);
}

bool UniversalTvBlaster::sendCodeByIndex(uint16_t index) {
  const uint16_t count = totalCodes();
  if (index >= count) {
    return false;
  }

  const IrCode *const *table = getCodeTable(region_);
  const IrCode *code = reinterpret_cast<const IrCode *>(pgm_read_ptr(table + index));
  if (code == nullptr) {
    return false;
  }

  const uint8_t timerVal = pgm_read_byte(&code->timer_val);
  const uint8_t numPairs = pgm_read_byte(&code->numpairs);
  const uint8_t bitCompression = pgm_read_byte(&code->bitcompression);
  const uint16_t *timePtr = reinterpret_cast<const uint16_t *>(pgm_read_ptr(&code->times));
  const uint8_t *packedCodes = reinterpret_cast<const uint8_t *>(pgm_read_ptr(&code->codes));

  if (numPairs == 0 || bitCompression == 0 || bitCompression > 8 || timePtr == nullptr || packedCodes == nullptr) {
    return false;
  }

  uint16_t raw[TVBG_RAW_BUFFER_CAPACITY];
  uint16_t rawLen = 0;

  uint8_t bitsLeft = 0;
  uint8_t bitsBuf = 0;
  const uint8_t *readPtr = packedCodes;

  for (uint16_t k = 0; k < numPairs; k++) {
    const uint8_t timeIndex = readPackedBits(readPtr, bitsLeft, bitsBuf, bitCompression);
    const uint16_t onTime10us = pgm_read_word(timePtr + static_cast<uint16_t>(timeIndex) * 2U);
    const uint16_t offTime10us = pgm_read_word(timePtr + static_cast<uint16_t>(timeIndex) * 2U + 1U);

    if (rawLen + 2U > TVBG_RAW_BUFFER_CAPACITY) {
      break;
    }

    raw[rawLen++] = tenUsToMicros(onTime10us);
    raw[rawLen++] = tenUsToMicros(offTime10us);
  }

  if (rawLen < 2) {
    return false;
  }

  const uint8_t carrierKHz = timerToCarrierKHz(timerVal);

  IrSender.sendRaw(raw, rawLen, carrierKHz);
  IrReceiver.restartAfterSend();

  return true;

}


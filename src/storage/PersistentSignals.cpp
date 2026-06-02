#include "PersistentSignals.h"

#include <Preferences.h>
#include <cstring>

namespace {
constexpr char NVS_NAMESPACE[] = "irpilot";
constexpr char KEY_META[] = "meta";
constexpr char KEY_BLOB[] = "signals";
constexpr uint32_t META_MAGIC = 0x49525031; // "IRP1"
constexpr uint16_t META_VERSION = 2;

struct PersistMeta {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint8_t savedCount;
  uint8_t maxSignals;
  uint8_t reserved[6];
};
} // namespace

bool loadSignalsFromNvs(uint8_t &savedCount, PersistedSignal *signals, size_t maxSignals) {
  savedCount = 0;
  if (signals == nullptr || maxSignals == 0) {
    return false;
  }
  std::memset(signals, 0, sizeof(PersistedSignal) * maxSignals);

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) {
    return false;
  }

  PersistMeta meta{};
  const size_t metaSize = prefs.getBytes(KEY_META, &meta, sizeof(meta));
  const size_t expectedPayloadSize = sizeof(PersistedSignal) * maxSignals;
  if (metaSize != sizeof(meta) ||
      meta.magic != META_MAGIC ||
      meta.version != META_VERSION ||
      meta.payloadSize != expectedPayloadSize ||
      meta.maxSignals != maxSignals) {
    prefs.end();
    return false;
  }

  const size_t blobSize = prefs.getBytes(KEY_BLOB, signals, expectedPayloadSize);
  prefs.end();
  if (blobSize != expectedPayloadSize) {
    std::memset(signals, 0, expectedPayloadSize);
    return false;
  }

  savedCount = meta.savedCount;
  if (savedCount > maxSignals) {
    savedCount = static_cast<uint8_t>(maxSignals);
  }
  return true;
}

bool saveSignalsToNvs(uint8_t savedCount, const PersistedSignal *signals, size_t maxSignals) {
  if (signals == nullptr || maxSignals == 0) {
    return false;
  }

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return false;
  }

  PersistMeta meta{};
  meta.magic = META_MAGIC;
  meta.version = META_VERSION;
  meta.payloadSize = static_cast<uint16_t>(sizeof(PersistedSignal) * maxSignals);
  meta.savedCount = savedCount;
  meta.maxSignals = static_cast<uint8_t>(maxSignals);

  const size_t metaWritten = prefs.putBytes(KEY_META, &meta, sizeof(meta));
  const size_t blobWritten = prefs.putBytes(KEY_BLOB, signals, sizeof(PersistedSignal) * maxSignals);
  prefs.end();

  return (metaWritten == sizeof(meta) && blobWritten == (sizeof(PersistedSignal) * maxSignals));
}

bool clearSignalsInNvs() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return false;
  }
  const bool okMeta = prefs.remove(KEY_META);
  const bool okBlob = prefs.remove(KEY_BLOB);
  prefs.end();
  return okMeta || okBlob;
}

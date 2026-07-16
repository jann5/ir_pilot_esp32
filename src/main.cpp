#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <cstdio>
#include <cstring>
#include <WiFi.h>
#include <WebServer.h>
#include <BleKeyboard.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "input/Button.h"
#include "irblast/UniversalTvBlaster.h"
#include "storage/PersistentSignals.h"

#define EXCLUDE_EXOTIC_PROTOCOLS
#define EXCLUDE_UNIVERSAL_PROTOCOLS
#include <IRremote.hpp>

// Bad Keyboard
static BleKeyboard bleKeyboard("Keyboard", "ESP32", 100);
static uint8_t badkbPresetIndex = 0;
static char badkbStatus[40] = "";
static bool badkbStaticDrawn = false;
static uint8_t badkbPrevIndex = 0xFF;
static char badkbPrevStatus[40] = "";

// TFT pins:
// SCL=22, SDA=23, RES=4, CS=5, D/C=2
static constexpr uint8_t TFT_SCLK = 22;
static constexpr uint8_t TFT_MOSI = 23;
static constexpr uint8_t TFT_RST  = 4;
static constexpr uint8_t TFT_CS   = 5;
static constexpr uint8_t TFT_DC   = 2;
static constexpr uint8_t TFT_INIT = INITR_MINI160x80_PLUGIN;
static constexpr uint32_t TFT_SPI_HZ = 40000000;

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// IR modules
static constexpr uint8_t IR_RX_PIN = 21;
static constexpr uint8_t IR_TX_PIN = 15;
static constexpr uint8_t TX_FEEDBACK_LED_PIN = 16;
static constexpr uint8_t LASER_PIN = 13;

// Buttons
// K1->GPIO17 = BACK
// K2->GPIO19 = OK/ENTER
// K3->GPIO32 = RIGHT
// K4->GPIO25 = LEFT
// BOOT->GPIO0 = HOME
static constexpr uint8_t BTN_HOME_PIN = 0;
static constexpr uint8_t BTN_BACK_PIN = 17;
static constexpr uint8_t BTN_OK_PIN = 19;
static constexpr uint8_t BTN_RIGHT_PIN = 32;
static constexpr uint8_t BTN_LEFT_PIN = 25;

// Single-source buttons for stable behavior.
static Button btnHome = {BTN_HOME_PIN, true, true, false, HIGH, HIGH, 0, 0, false, false};
static Button btnLeft = {BTN_LEFT_PIN, true, true, false, HIGH, HIGH, 0, 0, false, false};
static Button btnRight = {BTN_RIGHT_PIN, true, true, false, HIGH, HIGH, 0, 0, false, false};
static Button btnOk = {BTN_OK_PIN, true, true, false, HIGH, HIGH, 0, 0, false, false};
static Button btnBack = {BTN_BACK_PIN, true, true, false, HIGH, HIGH, 0, 0, false, false};

static constexpr uint32_t DEBOUNCE_MS = 18;
static constexpr uint32_t EDGE_LOCK_MS = 90;
static constexpr uint32_t CAPTURE_ARM_DELAY_MS = 0;
static constexpr uint32_t CAPTURE_MIN_INTERVAL_MS = 250;
static constexpr uint32_t OK_HOLD_DELETE_ONE_MS = 5000;
static constexpr uint32_t OK_HOLD_DELETE_ALL_MS = 10000;
static constexpr uint32_t MAIN_STATUS_MS = 1600;
static constexpr uint32_t UNIVERSAL_UI_REFRESH_MS = 125;
static constexpr uint32_t PROJECTOR_FAST_STEP_MS = 90;
static constexpr uint32_t UI_SLEEP_MS = 100000;
static constexpr uint8_t TV_BASIC_COUNT = 20;
static constexpr uint8_t PROJECTOR_BASIC_COUNT = 20;
static constexpr uint16_t WEB_PORT = 80;
static constexpr char WEB_AP_PASSWORD[] = "irpanel123";

static const char *MAIN_MENU_ITEMS[] = {"Zapisz IR", "Wyslij IR", "Uniwersal", "TV 20", "Proj 20", "Proj FAST", "Web", "Laser", "BadKB"};
static constexpr uint8_t MAIN_MENU_COUNT = sizeof(MAIN_MENU_ITEMS) / sizeof(MAIN_MENU_ITEMS[0]);

enum class UiScreen : uint8_t { Splash, MainMenu, Capture, SendList, UniversalTv, TvBasic20, Projector20, ProjectorFast, WebPanel, LaserControl, BadKeyboard };

struct StoredSignal {
  IRData data;
  uint8_t rawCode[RAW_BUFFER_LENGTH];
  uint16_t rawCodeLength;
  bool isRaw;
};

struct DevicePreset {
  const char *name;
  decode_type_t protocol;
  uint16_t address;
  uint16_t command;
  uint8_t bits;
};

static constexpr uint8_t MAX_SIGNALS = 10;
static StoredSignal savedSignals[MAX_SIGNALS];
static PersistedSignal persistedSignals[MAX_SIGNALS];
static uint8_t savedCount = 0;
static uint8_t sendIndex = 0;
static UniversalTvBlaster tvBlaster;
static WebServer webServer(WEB_PORT);

// 20 gotowych, nazwanych presetow TV POWER (EU/globalne najczestsze marki).
static const DevicePreset TV20_PRESETS[TV_BASIC_COUNT] = {
    {"Samsung (07/02)", SAMSUNG, 0x0007, 0x0002, 32},
    {"Samsung (0E/0C)", SAMSUNG, 0x000E, 0x000C, 32},
    {"LG webOS (04/08)", NEC, 0x0004, 0x0008, 32},
    {"LG alt (DF00/1C)", ONKYO, 0xDF00, 0x001C, 32},
    {"Sony Bravia A", SONY, 0x0001, 0x0015, 12},
    {"Sony Bravia B", SONY, 0x0010, 0x0015, 12},
    {"Philips RC6", RC6, 0x0000, 0x000C, 20},
    {"Philips RC5", RC5, 0x0000, 0x000C, 12},
    {"Toshiba (40/12)", NEC, 0x0040, 0x0012, 32},
    {"TCL/Thomson A", NEC, 0x0050, 0x0017, 32},
    {"Hisense A", NEC, 0x0038, 0x001C, 32},
    {"Sharp Aquos A", NEC, 0x00AA, 0x001C, 32},
    {"JVC TV A", NEC, 0x0053, 0x0017, 32},
    {"Panasonic TV", PANASONIC, 0x1000, 0x003D, 48},
    {"Grundig/Vestel A", ONKYO, 0x7A83, 0x0008, 32},
    {"Vizio/NEC A", NEC, 0x0018, 0x0008, 32},
    {"RCA/NEC A", NEC, 0x0071, 0x0008, 32},
    {"Hitachi/NEC A", NEC, 0x0048, 0x0000, 32},
    {"Xiaomi/NEC A", ONKYO, 0x6F80, 0x000A, 32},
    {"Sanyo/NEC A", NEC, 0x0000, 0x0001, 32},
};

static const DevicePreset PROJECTOR20_PRESETS[PROJECTOR_BASIC_COUNT] = {
    {"Epson 1", NEC, 0x5583, 0x0018, 32},
    {"Epson 2", NEC, 0x5583, 0x001C, 32},
    {"BenQ 1", NEC, 0xA25D, 0x0022, 32},
    {"BenQ 2", NEC, 0xA25D, 0x0023, 32},
    {"Optoma 1", NEC, 0xC03F, 0x0012, 32},
    {"Optoma 2", NEC, 0xC03F, 0x0013, 32},
    {"ViewSonic 1", NEC, 0x20DF, 0x0012, 32},
    {"ViewSonic 2", NEC, 0x20DF, 0x0013, 32},
    {"Acer 1", NEC, 0x86C1, 0x0014, 32},
    {"Acer 2", NEC, 0x86C1, 0x0015, 32},
    {"NEC Proj 1", NEC, 0x10EF, 0x0012, 32},
    {"NEC Proj 2", NEC, 0x10EF, 0x0013, 32},
    {"Sony Proj 1", SONY, 0x0011, 0x0015, 12},
    {"Sony Proj 2", SONY, 0x001A, 0x0015, 12},
    {"Panasonic 1", PANASONIC, 0x4004, 0x003D, 48},
    {"Panasonic 2", PANASONIC, 0x4004, 0x003E, 48},
    {"LG Proj 1", NEC, 0x04FB, 0x0008, 32},
    {"LG Proj 2", NEC, 0x04FB, 0x0009, 32},
    {"Hitachi Proj", NEC, 0x08F7, 0x0012, 32},
    {"Sanyo Proj", NEC, 0x00AD, 0x0012, 32},
};

static uint32_t captureEnteredMs = 0;
static uint32_t lastSavedSignalMs = 0;

static bool okSendHoldTracking = false;
static bool okSendHoldDoneSingle = false;
static bool okSendHoldDoneAll = false;
static uint32_t okSendHoldStartMs = 0;

static UiScreen uiScreen = UiScreen::MainMenu;
static uint8_t selectedMain = 0;
static bool uiDirty = true;

static constexpr uint16_t C_BG = 0x0032;
static constexpr uint16_t C_PANEL = 0x0418;
static constexpr uint16_t C_TEXT = 0xE71C;
static constexpr uint16_t C_ACCENT = 0x5DFF;
static constexpr uint16_t C_TEXT_DIM = 0x9D9F;
static constexpr uint16_t C_HILITE = 0x1CFF;

static char captureStatus[40] = "Czeka na sygnal...";
static char sendStatus[40] = "";
static char tvStatus[40] = "";
static char tv20Status[40] = "";
static char mainStatus[32] = "";
static uint32_t mainStatusUntilMs = 0;
static uint32_t tvProgressRefreshMs = 0;
static uint32_t lastInteractionMs = 0;
static bool displaySleeping = false;
static uint8_t tv20Index = 0;
static uint8_t projector20Index = 0;
static bool projectorFastRunning = false;
static uint8_t projectorFastIndex = 0;
static uint32_t projectorFastNextSendMs = 0;
static uint32_t projectorFastProgressRefreshMs = 0;
static bool tvTurboStaticDrawn = false;
static bool tvTurboPrevRunning = false;
static uint16_t tvTurboPrevSent = 0xFFFF;
static uint16_t tvTurboPrevTotal = 0xFFFF;
static char tvTurboPrevStatus[40] = "";
static bool mainMenuStaticDrawn = false;
static uint8_t mainMenuPrevSelected = 0xFF;
static char mainMenuPrevStatus[32] = "";
static bool captureStaticDrawn = false;
static char capturePrevStatus[40] = "";
static uint8_t capturePrevCount = 0xFF;
static bool sendListStaticDrawn = false;
static bool sendListPrevEmpty = false;
static uint8_t sendListPrevIndex = 0xFF;
static char sendListPrevLine1[28] = "";
static char sendListPrevLine2[28] = "";
static char sendListPrevStatus[40] = "";
static bool tv20StaticDrawn = false;
static uint8_t tv20PrevIndex = 0xFF;
static char tv20PrevStatus[40] = "";
static bool projector20StaticDrawn = false;
static uint8_t projector20PrevIndex = 0xFF;
static char projector20Status[40] = "";
static char projector20PrevStatus[40] = "";
static bool projectorFastStaticDrawn = false;
static bool projectorFastPrevRunning = false;
static uint8_t projectorFastPrevIndex = 0xFF;
static char projectorFastStatus[40] = "";
static char projectorFastPrevStatus[40] = "";
static bool webPanelStaticDrawn = false;
static char webPanelPrevLine1[32] = "";
static char webPanelPrevLine2[32] = "";
static char webPanelPrevStatus[40] = "";
static bool splashStaticDrawn = false;
static char webApSsid[24] = "ESP32-IR";
static char webApIpText[20] = "192.168.4.1";
static char webStatus[40] = "AP start...";
static bool laserEnabled = false;
static bool laserStaticDrawn = false;
static bool laserPrevEnabled = false;

static const unsigned char PROGMEM image_paint_3_bits[] = {
0x00,0x00,0x00,0xfe,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x01,0x80,0x00,
0x00,0x00,0x00,0xc0,0x00,0x00,0xe0,0x00,0x60,0x00,0x00,0x1c,0x00,0x30,0x00,0x00,
0x06,0x00,0x10,0x00,0x38,0x03,0x00,0x18,0x00,0x0c,0x01,0x80,0x08,0x0e,0x06,0x00,
0xc0,0x08,0x01,0x83,0x00,0x60,0x04,0x00,0xc1,0x80,0x20,0x04,0xe0,0x60,0x80,0x30,
0x04,0x30,0x20,0x40,0x10,0x04,0x10,0x20,0x40,0x10,0x06,0x18,0x20,0x40,0x10,0x02,
0x08,0x20,0x60,0x10,0x02,0x08,0x20,0x20,0x10,0x02,0x38,0x20,0x20,0x10,0x02,0x60,
0x60,0x60,0x10,0x02,0x00,0xc0,0xc0,0x30,0x02,0x00,0x01,0x80,0x60,0x02,0x00,0x01,
0x00,0xc0,0x02,0x00,0x00,0x00,0x80,0x06,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
0x00,0x0c,0x00,0x00,0x00,0x00,0x18
};

static const unsigned char PROGMEM image_Scanning_short_bits[] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x03,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x0f,0x00,0xe0,0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,0x00,0x35,0xc0,0x18,
0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,0x00,0x6a,0xa0,0x06,0x00,0x00,0x00,0x01,
0x40,0x00,0x00,0x00,0x00,0x81,0x50,0x01,0x00,0x00,0x00,0x01,0x40,0x00,0x00,0x00,
0x01,0x00,0xa8,0x00,0x80,0x00,0x00,0x02,0x40,0x00,0x00,0x00,0x02,0x00,0x1c,0x00,
0x40,0x00,0x00,0x02,0x40,0x00,0x00,0x00,0x04,0x00,0x2e,0x00,0x20,0x00,0x00,0x02,
0x41,0x80,0x00,0x00,0x04,0x00,0x16,0x00,0x20,0x00,0x00,0x04,0x41,0x40,0x60,0x00,
0x08,0x00,0x0b,0x07,0xf0,0x00,0x00,0x04,0x41,0x40,0x50,0x30,0x08,0x00,0x05,0x38,
0x08,0x00,0x00,0x04,0x41,0x40,0x50,0x28,0x10,0x07,0xcb,0xc0,0x08,0x00,0x00,0x08,
0x41,0x40,0x50,0x28,0x10,0x09,0xe5,0x02,0x24,0x00,0x00,0x08,0x41,0x40,0x50,0x28,
0x10,0x13,0xfe,0x21,0x14,0x00,0x00,0x08,0x21,0x40,0x50,0xff,0x11,0xe6,0x7c,0x11,
0x14,0x00,0x00,0x08,0x21,0x5f,0xff,0xff,0x22,0x26,0x74,0x11,0x14,0x00,0x00,0x08,
0x3f,0xf5,0xff,0xa8,0x22,0x27,0xf4,0x11,0x54,0x00,0x00,0x18,0x2b,0xfb,0xf8,0x28,
0x22,0x27,0xf4,0x0a,0xff,0x00,0x7f,0xf8,0x37,0xfc,0x50,0x28,0x22,0x23,0xf4,0x57,
0x00,0x80,0x80,0x58,0x3f,0x40,0x50,0x28,0x21,0x11,0xf4,0xb8,0x00,0x41,0xbe,0x78,
0x21,0x40,0x50,0x18,0x61,0xf8,0x63,0x60,0x00,0x21,0xbe,0x68,0x21,0x40,0x50,0x00,
0x62,0xaf,0xc3,0x80,0x00,0x22,0x80,0x44,0x21,0x40,0x30,0x00,0xe1,0x50,0x3e,0x00,
0x00,0x22,0xc0,0x44,0x11,0x40,0x00,0x00,0xa2,0xa0,0x10,0x00,0x7e,0x25,0xc0,0x42,
0x11,0x40,0x00,0x00,0x61,0x60,0x00,0x03,0x81,0xa5,0xe0,0x42,0x10,0xc0,0x00,0x00,
0xa0,0x00,0x00,0x0c,0x00,0x4b,0xff,0x81,0x10,0x00,0x00,0x00,0x60,0x00,0x00,0x30,
0x00,0x4b,0xf8,0x01,0x10,0x00,0x00,0x00,0xa0,0x01,0x00,0xc0,0x00,0x8b,0xf8,0x00,
0x90,0x00,0x00,0x00,0x60,0x00,0x87,0x00,0x01,0x97,0xf0,0x00,0x48,0x00,0x00,0x00,
0xa0,0x00,0x78,0x00,0x03,0x17,0xf0,0x00,0x28,0x00,0x00,0x00,0x60,0x00,0x00,0x00,
0x0e,0x77,0xf0,0x00,0x18,0x00,0x00,0x00,0xe0,0x00,0x00,0x00,0x3c,0x9f,0xf0,0x00,
0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x55,0xf9,0x0f,0xf0,0x00,0x00,0x00,0x00,0x00,
0x80,0x02,0xaa,0xbf,0xfa,0x0f,0xe8,0x00,0x00,0x00,0x00,0x00,0xc0,0x01,0x57,0xff,
0xfc,0x07,0xe8,0x00,0x00,0x00,0x00,0x00,0x80,0x02,0xab,0xff,0xf8,0x07,0xe8,0x00,
0x00,0x00,0x00,0x00,0x40,0x01,0x55,0xff,0xf0,0x07,0xc8,0x00,0x00,0x00,0x00,0x00,
0x80,0x00,0xaa,0xaa,0xc0,0x0f,0xa8,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x55,0x55,
0x00,0x0d,0x50,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x2a,0xae,0x00,0x1a,0xb0,0x00,
0x00,0x00,0x00,0x00,0x40,0x00,0x15,0xf0,0x00,0x15,0x60,0x00,0x00,0x00,0x00,0x00,
0xa0,0x00,0x3e,0x00,0x00,0x3a,0xc0,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
0x00,0x75,0x80,0x00,0x00,0x00,0x00,0x00,0xa0,0x00,0x00,0x00,0x00,0xeb,0x00,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x01,0xde,0x00,0x00,0x00,0x00,0x00,0x00,
0xa8,0x00,0x00,0x00,0x03,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x54,0x00,0x00,0x00,
0x0f,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0xaa,0x00,0x00,0x00,0x3f,0x80,0x00,0x00,
0x00,0x00,0x00,0x00,0x55,0x00,0x00,0x00,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

static void setText(char *dst, size_t n, const char *src) {
  std::strncpy(dst, src, n - 1);
  dst[n - 1] = '\0';
}

static String htmlEscape(const String &input) {
  String out;
  out.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); i++) {
    const char c = input[i];
    if (c == '&') {
      out += F("&amp;");
    } else if (c == '<') {
      out += F("&lt;");
    } else if (c == '>') {
      out += F("&gt;");
    } else if (c == '"') {
      out += F("&quot;");
    } else {
      out += c;
    }
  }
  return out;
}

static void showMainStatus(const char *text) {
  setText(mainStatus, sizeof(mainStatus), text);
  mainStatusUntilMs = millis() + MAIN_STATUS_MS;
}

static void resetTurboScreenCache() {
  tvTurboStaticDrawn = false;
  tvTurboPrevRunning = false;
  tvTurboPrevSent = 0xFFFF;
  tvTurboPrevTotal = 0xFFFF;
  tvTurboPrevStatus[0] = '\0';

  projectorFastStaticDrawn = false;
  projectorFastPrevRunning = false;
  projectorFastPrevIndex = 0xFF;
  projectorFastPrevStatus[0] = '\0';

  webPanelStaticDrawn = false;
  webPanelPrevLine1[0] = '\0';
  webPanelPrevLine2[0] = '\0';
  webPanelPrevStatus[0] = '\0';

  laserStaticDrawn = false;
  laserPrevEnabled = !laserEnabled;
}

static void invalidateAllUiCaches() {
  splashStaticDrawn = false;

  mainMenuStaticDrawn = false;
  mainMenuPrevSelected = 0xFF;
  mainMenuPrevStatus[0] = '\0';

  captureStaticDrawn = false;
  capturePrevStatus[0] = '\0';
  capturePrevCount = 0xFF;

  sendListStaticDrawn = false;
  sendListPrevEmpty = false;
  sendListPrevIndex = 0xFF;
  sendListPrevLine1[0] = '\0';
  sendListPrevLine2[0] = '\0';
  sendListPrevStatus[0] = '\0';

  tv20StaticDrawn = false;
  tv20PrevIndex = 0xFF;
  tv20PrevStatus[0] = '\0';
  projector20StaticDrawn = false;
  projector20PrevIndex = 0xFF;
  projector20Status[0] = '\0';
  projector20PrevStatus[0] = '\0';
  projectorFastStatus[0] = '\0';

  resetTurboScreenCache();
}

static void noteInteraction() {
  lastInteractionMs = millis();
}

static void stopProjectorFast() {
  projectorFastRunning = false;
}

static void stopAllAutomations() {
  tvBlaster.stop();
  stopProjectorFast();
}

// Bad Keyboard presets - TYLKO 3: YouTube, YT Android, Disconnect
static constexpr uint8_t BADKB_PRESET_COUNT = 3;

struct BadKbPreset {
  const char *name;
  void (*run)();
};

static void runBadKbYouTube() {
  if (!bleKeyboard.isConnected()) return;
  // iOS: Home -> Spotlight -> Safari -> URL
  bleKeyboard.press(KEY_LEFT_GUI);
  bleKeyboard.press('h');
  bleKeyboard.releaseAll();
  delay(300);
  bleKeyboard.press(KEY_LEFT_GUI);
  bleKeyboard.press(' ');
  bleKeyboard.releaseAll();
  delay(400);
  bleKeyboard.println("safari");
  delay(200);
  bleKeyboard.press(KEY_RETURN);
  bleKeyboard.releaseAll();
  delay(1500);
  bleKeyboard.press(KEY_LEFT_GUI);
  bleKeyboard.press('l');
  bleKeyboard.releaseAll();
  delay(200);
  bleKeyboard.println("pornhub.com");
  delay(100);
  bleKeyboard.press(KEY_RETURN);
  bleKeyboard.releaseAll();
}

static void runBadKbYouTubeAndroid() {
  if (!bleKeyboard.isConnected()) return;
  // Android: Home -> Search -> youtube.com
  bleKeyboard.press(KEY_HOME);
  bleKeyboard.releaseAll();
  delay(500);
  bleKeyboard.press(KEY_LEFT_GUI);
  bleKeyboard.press(' ');
  bleKeyboard.releaseAll();
  delay(400);
  bleKeyboard.println("pornhub.com");
  delay(200);
  bleKeyboard.press(KEY_RETURN);
  bleKeyboard.releaseAll();
}

static void runBadKbDisconnect() {
  bleKeyboard.end();
  delay(500);
  bleKeyboard.begin();
}

static const BadKbPreset BADKB_PRESETS[BADKB_PRESET_COUNT] = {
  {"YouTube", runBadKbYouTube},
  {"YT Android", runBadKbYouTubeAndroid},
  {"Disconnect", runBadKbDisconnect}
};

static void drawBadKeyboardScreen();

static void wakeDisplay() {
  if (!displaySleeping) {
    return;
  }
  tft.enableSleep(false);
  delay(5);
  tft.enableDisplay(true);
  displaySleeping = false;
  noteInteraction();
  invalidateAllUiCaches();
  uiDirty = true;
}

static void sleepDisplay() {
  if (displaySleeping) {
    return;
  }
  tvBlaster.stop();
  stopProjectorFast();
  tft.enableDisplay(false);
  tft.enableSleep(true);
  displaySleeping = true;
}

static void copyRuntimeToPersisted() {
  std::memset(persistedSignals, 0, sizeof(persistedSignals));
  for (uint8_t i = 0; i < savedCount && i < MAX_SIGNALS; i++) {
    PersistedSignal &dst = persistedSignals[i];
    const StoredSignal &src = savedSignals[i];

    dst.protocol = static_cast<uint16_t>(src.data.protocol);
    dst.address = src.data.address;
    dst.command = src.data.command;
    dst.bits = src.data.numberOfBits;
    dst.isRaw = src.isRaw ? 1U : 0U;
    dst.rawCodeLength = src.rawCodeLength;

    const uint16_t copyLen = (src.rawCodeLength > PERSISTENT_RAW_BUFFER_LENGTH) ? PERSISTENT_RAW_BUFFER_LENGTH : src.rawCodeLength;
    if (copyLen > 0) {
      std::memcpy(dst.rawCode, src.rawCode, copyLen);
      dst.rawCodeLength = copyLen;
    }
  }
}

static void copyPersistedToRuntime() {
  std::memset(savedSignals, 0, sizeof(savedSignals));
  for (uint8_t i = 0; i < savedCount && i < MAX_SIGNALS; i++) {
    StoredSignal &dst = savedSignals[i];
    const PersistedSignal &src = persistedSignals[i];

    dst.data.protocol = static_cast<decode_type_t>(src.protocol);
    dst.data.address = src.address;
    dst.data.command = src.command;
    dst.data.numberOfBits = src.bits;
    dst.data.flags = IRDATA_FLAGS_EMPTY;
    dst.isRaw = (src.isRaw != 0);
    dst.rawCodeLength = src.rawCodeLength;

    const uint16_t copyLen = (src.rawCodeLength > RAW_BUFFER_LENGTH) ? RAW_BUFFER_LENGTH : src.rawCodeLength;
    if (copyLen > 0) {
      std::memcpy(dst.rawCode, src.rawCode, copyLen);
      dst.rawCodeLength = copyLen;
    }
  }
}

static void persistSignals() {
  copyRuntimeToPersisted();
  const bool ok = saveSignalsToNvs(savedCount, persistedSignals, MAX_SIGNALS);
  Serial.printf("NVS save: count=%u ok=%d\n", savedCount, ok ? 1 : 0);
}

static void clearSavedSignals(bool clearNvs) {
  savedCount = 0;
  sendIndex = 0;
  std::memset(savedSignals, 0, sizeof(savedSignals));
  setText(sendStatus, sizeof(sendStatus), "");
  if (clearNvs) {
    clearSignalsInNvs();
    persistSignals();
  }
}

static void drawCentered(const char *text, int16_t y, uint8_t size, uint16_t color) {
  tft.setTextSize(size);
  tft.setTextColor(color);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int16_t x = (tft.width() - static_cast<int16_t>(w)) / 2;
  if (x < 0) {
    x = 0;
  }
  tft.setCursor(x, y);
  tft.print(text);
}

static void drawPanelChrome(const char *title) {
  const int16_t w = tft.width();
  const int16_t h = tft.height();
  tft.fillScreen(C_BG);
  tft.fillRoundRect(4, 8, w - 8, h - 16, 6, C_PANEL);
  tft.drawRoundRect(4, 8, w - 8, h - 16, 6, C_ACCENT);
  tft.fillRoundRect(12, 12, w - 24, 12, 4, C_BG);
  drawCentered(title, 15, 1, C_HILITE);
}

static void clearBand(int16_t top, int16_t height) {
  tft.fillRect(8, top, tft.width() - 16, height, C_PANEL);
}

static void drawCenteredBand(const char *text, int16_t top, int16_t height, uint8_t size, uint16_t color) {
  clearBand(top, height);
  tft.setTextSize(size);
  tft.setTextColor(color);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int16_t x = (tft.width() - static_cast<int16_t>(w)) / 2;
  if (x < 0) {
    x = 0;
  }
  int16_t y = top + (height - static_cast<int16_t>(h)) / 2;
  if (y < top) {
    y = top;
  }
  tft.setCursor(x, y);
  tft.print(text);
}

static void drawNavArrows() {
  const int16_t h = tft.height();
  tft.setTextColor(C_HILITE, C_PANEL);
  tft.setTextSize(2);
  tft.setCursor(12, h / 2 - 12);
  tft.print("<");
  tft.setCursor(tft.width() - 22, h / 2 - 12);
  tft.print(">");
}

static void drawSplashScreen() {
  if (splashStaticDrawn) {
    return;
  }

  tft.fillScreen(0x041F);
  tft.drawBitmap(0, 28, image_Scanning_short_bits, 96, 52, ST77XX_BLACK);
  tft.drawBitmap(98, 29, image_paint_3_bits, 39, 27, ST77XX_BLACK);

  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(0xFBAA);
  tft.setCursor(3, 2);
  tft.print("jan.n5");

  tft.setTextColor(0xFCA0);
  tft.setCursor(45, 2);
  tft.print("project");

  splashStaticDrawn = true;
}

static void drawMainMenu() {
  const bool statusVisible = (mainStatus[0] != '\0' && millis() <= mainStatusUntilMs);
  if (mainStatus[0] != '\0' && millis() > mainStatusUntilMs) {
    mainStatus[0] = '\0';
  }

  if (!mainMenuStaticDrawn) {
    drawPanelChrome("IR MENU");
    drawNavArrows();
    mainMenuStaticDrawn = true;
    mainMenuPrevSelected = 0xFF;
    mainMenuPrevStatus[0] = '\0';
  }

  if (mainMenuPrevSelected != selectedMain) {
    tft.fillRoundRect(22, 30, tft.width() - 44, 22, 5, C_BG);
    drawCenteredBand(MAIN_MENU_ITEMS[selectedMain], 30, 22, 2, C_TEXT);
    mainMenuPrevSelected = selectedMain;
  }

  const char *statusText = statusVisible ? mainStatus : "";
  if (std::strncmp(mainMenuPrevStatus, statusText, sizeof(mainMenuPrevStatus)) != 0) {
    drawCenteredBand(statusText, 66, 10, 1, C_TEXT_DIM);
    setText(mainMenuPrevStatus, sizeof(mainMenuPrevStatus), statusText);
  }
}

static void drawCaptureScreen() {
  if (!captureStaticDrawn) {
    drawPanelChrome("Zapisz IR");
    captureStaticDrawn = true;
    capturePrevStatus[0] = '\0';
    capturePrevCount = 0xFF;
  }

  if (std::strncmp(capturePrevStatus, captureStatus, sizeof(capturePrevStatus)) != 0) {
    drawCenteredBand(captureStatus, 30, 12, 1, C_TEXT);
    setText(capturePrevStatus, sizeof(capturePrevStatus), captureStatus);
  }

  if (capturePrevCount != savedCount) {
    char countLine[24];
    std::snprintf(countLine, sizeof(countLine), "Zapisane: %u", savedCount);
    drawCenteredBand(countLine, 48, 12, 1, C_ACCENT);
    capturePrevCount = savedCount;
  }
}

static void drawSendListScreen() {
  if (!sendListStaticDrawn) {
    drawPanelChrome("Wyslij IR");
    drawNavArrows();
    sendListStaticDrawn = true;
    sendListPrevEmpty = !savedCount;
    sendListPrevIndex = 0xFF;
    sendListPrevLine1[0] = '\0';
    sendListPrevLine2[0] = '\0';
    sendListPrevStatus[0] = '\0';
  }

  if (savedCount == 0) {
    if (!sendListPrevEmpty) {
      clearBand(26, 50);
      sendListPrevLine1[0] = '\0';
      sendListPrevLine2[0] = '\0';
      sendListPrevStatus[0] = '\0';
    }
    drawCenteredBand("Brak zapisanych", 30, 12, 1, C_TEXT);
    sendListPrevEmpty = true;
    return;
  }
  sendListPrevEmpty = false;

  char indexLine[20];
  std::snprintf(indexLine, sizeof(indexLine), "[%u/%u]", sendIndex + 1, savedCount);
  if (sendListPrevIndex != sendIndex) {
    drawCenteredBand(indexLine, 24, 10, 1, C_ACCENT);
    sendListPrevIndex = sendIndex;
  }

  char line1[28];
  char line2[28];
  const StoredSignal &s = savedSignals[sendIndex];
  if (s.isRaw) {
    std::snprintf(line1, sizeof(line1), "RAW 38kHz");
    std::snprintf(line2, sizeof(line2), "Len: %u", s.rawCodeLength);
  } else {
    std::snprintf(line1, sizeof(line1), "%s", getProtocolString(s.data.protocol));
    std::snprintf(line2, sizeof(line2), "A:%04X C:%04X", s.data.address, s.data.command);
  }

  if (std::strncmp(sendListPrevLine1, line1, sizeof(sendListPrevLine1)) != 0) {
    drawCenteredBand(line1, 38, 12, 1, C_TEXT);
    setText(sendListPrevLine1, sizeof(sendListPrevLine1), line1);
  }
  if (std::strncmp(sendListPrevLine2, line2, sizeof(sendListPrevLine2)) != 0) {
    drawCenteredBand(line2, 52, 12, 1, C_TEXT);
    setText(sendListPrevLine2, sizeof(sendListPrevLine2), line2);
  }
  if (std::strncmp(sendListPrevStatus, sendStatus, sizeof(sendListPrevStatus)) != 0) {
    drawCenteredBand(sendStatus, 66, 10, 1, C_TEXT_DIM);
    setText(sendListPrevStatus, sizeof(sendListPrevStatus), sendStatus);
  }
}

static void drawUniversalTvScreen() {
  const int16_t w = tft.width();
  const int16_t h = tft.height();

  if (!tvTurboStaticDrawn) {
    drawPanelChrome("TV BLAST");
    tft.fillRoundRect(18, 28, w - 36, 32, 6, C_BG);
    tvTurboStaticDrawn = true;
    tvTurboPrevSent = 0xFFFF;
    tvTurboPrevTotal = 0xFFFF;
    tvTurboPrevStatus[0] = '\0';
  }

  const bool running = tvBlaster.isRunning();
  const uint16_t sent = tvBlaster.sentCodes();
  const uint16_t total = tvBlaster.totalCodes();

  if (tvTurboPrevRunning != running || tvTurboPrevSent != sent || tvTurboPrevTotal != total) {
    tft.fillRoundRect(18, 28, w - 36, 32, 6, C_BG);
    if (running) {
      char progressLine[28];
      std::snprintf(progressLine, sizeof(progressLine), "Skan %u/%u", sent, total);
      drawCentered(progressLine, 36, 1, C_TEXT);
      drawCentered("OK stop", 48, 1, C_HILITE);
    } else {
      drawCentered("OK start", 42, 1, C_TEXT);
    }
    tvTurboPrevRunning = running;
    tvTurboPrevSent = sent;
    tvTurboPrevTotal = total;
  }

  if (std::strncmp(tvTurboPrevStatus, tvStatus, sizeof(tvTurboPrevStatus)) != 0) {
    clearBand(66, 10);
    if (tvStatus[0] != '\0') {
      drawCentered(tvStatus, 68, 1, C_TEXT_DIM);
    }
    setText(tvTurboPrevStatus, sizeof(tvTurboPrevStatus), tvStatus);
  }
}

static void drawTvBasic20Screen() {
  const DevicePreset &preset = TV20_PRESETS[tv20Index];

  if (!tv20StaticDrawn) {
    drawPanelChrome("TV 20");
    drawNavArrows();
    tft.fillRoundRect(18, 28, tft.width() - 36, 28, 6, C_BG);
    drawCenteredBand("OK power", 62, 10, 1, C_HILITE);
    tv20StaticDrawn = true;
    tv20PrevIndex = 0xFF;
    tv20PrevStatus[0] = '\0';
  }

  if (tv20PrevIndex != tv20Index) {
    tft.fillRoundRect(18, 28, tft.width() - 36, 28, 6, C_BG);
    drawCenteredBand(preset.name, 31, 10, 1, C_TEXT);

    char idxLine[22];
    std::snprintf(idxLine, sizeof(idxLine), "[%u/%u]", tv20Index + 1, TV_BASIC_COUNT);
    drawCenteredBand(idxLine, 44, 10, 1, C_TEXT_DIM);
    tv20PrevIndex = tv20Index;
  }

  if (std::strncmp(tv20PrevStatus, tv20Status, sizeof(tv20PrevStatus)) != 0) {
    drawCenteredBand(tv20Status, 72, 8, 1, C_TEXT_DIM);
    setText(tv20PrevStatus, sizeof(tv20PrevStatus), tv20Status);
  }
}

static bool sendTvBasic20Selected() {
  const DevicePreset &preset = TV20_PRESETS[tv20Index];

  IRData txData{};
  txData.protocol = preset.protocol;
  txData.address = preset.address;
  txData.command = preset.command;
  txData.numberOfBits = preset.bits;
  txData.flags = IRDATA_FLAGS_EMPTY;

  digitalWrite(TX_FEEDBACK_LED_PIN, HIGH);
  const bool ok = (IrSender.write(&txData, 0) > 0);
  digitalWrite(TX_FEEDBACK_LED_PIN, LOW);

  IrReceiver.restartAfterSend();
  Serial.printf("TV20 TX: %s proto=%s addr=0x%04X cmd=0x%04X bits=%u ok=%d\n",
                preset.name,
                getProtocolString(preset.protocol),
                preset.address,
                preset.command,
                preset.bits,
                ok ? 1 : 0);

  return ok;
}

static void drawProjector20Screen() {
  const DevicePreset &preset = PROJECTOR20_PRESETS[projector20Index];

  if (!projector20StaticDrawn) {
    drawPanelChrome("PROJ 20");
    drawNavArrows();
    tft.fillRoundRect(18, 28, tft.width() - 36, 28, 6, C_BG);
    drawCenteredBand("OK power", 62, 10, 1, C_HILITE);
    projector20StaticDrawn = true;
    projector20PrevIndex = 0xFF;
    projector20Status[0] = '\0';
    projector20PrevStatus[0] = '\0';
  }

  if (projector20PrevIndex != projector20Index) {
    tft.fillRoundRect(18, 28, tft.width() - 36, 28, 6, C_BG);
    drawCenteredBand(preset.name, 31, 10, 1, C_TEXT);

    char idxLine[22];
    std::snprintf(idxLine, sizeof(idxLine), "[%u/%u]", projector20Index + 1, PROJECTOR_BASIC_COUNT);
    drawCenteredBand(idxLine, 44, 10, 1, C_TEXT_DIM);
    projector20PrevIndex = projector20Index;
  }

  if (std::strncmp(projector20Status, projector20PrevStatus, sizeof(projector20Status)) != 0) {
    drawCenteredBand(projector20Status, 72, 8, 1, C_TEXT_DIM);
    setText(projector20PrevStatus, sizeof(projector20PrevStatus), projector20Status);
  }
}

static bool sendProjector20Selected() {
  const DevicePreset &preset = PROJECTOR20_PRESETS[projector20Index];

  IRData txData{};
  txData.protocol = preset.protocol;
  txData.address = preset.address;
  txData.command = preset.command;
  txData.numberOfBits = preset.bits;
  txData.flags = IRDATA_FLAGS_EMPTY;

  digitalWrite(TX_FEEDBACK_LED_PIN, HIGH);
  const bool ok = (IrSender.write(&txData, 0) > 0);
  digitalWrite(TX_FEEDBACK_LED_PIN, LOW);

  IrReceiver.restartAfterSend();
  Serial.printf("PROJ20 TX: %s proto=%s addr=0x%04X cmd=0x%04X bits=%u ok=%d\n",
                preset.name,
                getProtocolString(preset.protocol),
                preset.address,
                preset.command,
                preset.bits,
                ok ? 1 : 0);

  return ok;
}

static bool sendProjectorPresetByIndex(uint8_t index) {
  if (index >= PROJECTOR_BASIC_COUNT) {
    return false;
  }
  const DevicePreset &preset = PROJECTOR20_PRESETS[index];

  IRData txData{};
  txData.protocol = preset.protocol;
  txData.address = preset.address;
  txData.command = preset.command;
  txData.numberOfBits = preset.bits;
  txData.flags = IRDATA_FLAGS_EMPTY;

  digitalWrite(TX_FEEDBACK_LED_PIN, HIGH);
  const bool ok = (IrSender.write(&txData, 0) > 0);
  digitalWrite(TX_FEEDBACK_LED_PIN, LOW);

  IrReceiver.restartAfterSend();
  Serial.printf("PROJ20 TX: %s proto=%s addr=0x%04X cmd=0x%04X bits=%u ok=%d\n",
                preset.name,
                getProtocolString(preset.protocol),
                preset.address,
                preset.command,
                preset.bits,
                ok ? 1 : 0);

  return ok;
}

static bool sendStoredSignalByIndex(uint8_t index) {
  if (index >= savedCount) {
    return false;
  }

  StoredSignal &signal = savedSignals[index];
  bool ok = false;
  digitalWrite(TX_FEEDBACK_LED_PIN, HIGH);
  if (signal.isRaw) {
    if (signal.rawCodeLength > 0) {
      IrSender.sendRaw(signal.rawCode, signal.rawCodeLength, 38);
      ok = true;
      Serial.printf("IR TX raw len=%u\n", signal.rawCodeLength);
    }
  } else {
    ok = (IrSender.write(&signal.data, 1) > 0);
    Serial.printf("IR TX proto=%s addr=0x%04X cmd=0x%04X bits=%u ok=%d\n",
                  getProtocolString(signal.data.protocol),
                  signal.data.address,
                  signal.data.command,
                  signal.data.numberOfBits,
                  ok ? 1 : 0);
  }
  digitalWrite(TX_FEEDBACK_LED_PIN, LOW);

  IrReceiver.restartAfterSend();
  return ok;
}

static void startProjectorFast() {
  projectorFastIndex = 0;
  projectorFastRunning = true;
  projectorFastNextSendMs = millis();
  projectorFastProgressRefreshMs = 0;
  setText(projectorFastStatus, sizeof(projectorFastStatus), "");
}

static void startUniversalBlast() {
  tvBlaster.setRegion(TvBgoneRegion::EU);
  tvBlaster.setCodeLimit(0);
  tvBlaster.setLooping(false);
  tvBlaster.stop();
  tvBlaster.start(0);
  tvProgressRefreshMs = 0;
  setText(tvStatus, sizeof(tvStatus), "");
}

static void drawProjectorFastScreen() {
  const int16_t w = tft.width();

  if (!projectorFastStaticDrawn) {
    drawPanelChrome("PROJ FAST");
    tft.fillRoundRect(18, 28, w - 36, 32, 6, C_BG);
    projectorFastStaticDrawn = true;
    projectorFastPrevRunning = false;
    projectorFastPrevIndex = 0xFF;
    projectorFastPrevStatus[0] = '\0';
  }

  if (projectorFastPrevRunning != projectorFastRunning || projectorFastPrevIndex != projectorFastIndex) {
    tft.fillRoundRect(18, 28, w - 36, 32, 6, C_BG);
    if (projectorFastRunning) {
      char idxLine[24];
      std::snprintf(idxLine, sizeof(idxLine), "%u/%u", projectorFastIndex + 1, PROJECTOR_BASIC_COUNT);
      drawCentered(idxLine, 33, 1, C_ACCENT);
      drawCentered(PROJECTOR20_PRESETS[projectorFastIndex].name, 45, 1, C_TEXT);
    } else if (projectorFastIndex >= PROJECTOR_BASIC_COUNT) {
      drawCentered("Koniec", 36, 1, C_TEXT);
      drawCentered("OK start", 48, 1, C_HILITE);
    } else {
      drawCentered("OK start", 42, 1, C_TEXT);
    }
    projectorFastPrevRunning = projectorFastRunning;
    projectorFastPrevIndex = projectorFastIndex;
  }

  if (std::strncmp(projectorFastStatus, projectorFastPrevStatus, sizeof(projectorFastStatus)) != 0) {
    drawCenteredBand(projectorFastStatus, 68, 10, 1, C_TEXT_DIM);
    setText(projectorFastPrevStatus, sizeof(projectorFastPrevStatus), projectorFastStatus);
  }
}

static void drawWebPanelScreen() {
  if (!webPanelStaticDrawn) {
    drawPanelChrome("WEB PANEL");
    webPanelStaticDrawn = true;
    webPanelPrevLine1[0] = '\0';
    webPanelPrevLine2[0] = '\0';
    webPanelPrevStatus[0] = '\0';
  }

  if (std::strncmp(webPanelPrevLine1, webApSsid, sizeof(webPanelPrevLine1)) != 0) {
    drawCenteredBand(webApSsid, 28, 12, 1, C_TEXT);
    setText(webPanelPrevLine1, sizeof(webPanelPrevLine1), webApSsid);
  }

  if (std::strncmp(webPanelPrevLine2, webApIpText, sizeof(webPanelPrevLine2)) != 0) {
    drawCenteredBand(webApIpText, 42, 12, 1, C_ACCENT);
    setText(webPanelPrevLine2, sizeof(webPanelPrevLine2), webApIpText);
  }

  if (std::strncmp(webPanelPrevStatus, webStatus, sizeof(webPanelPrevStatus)) != 0) {
    drawCenteredBand(webStatus, 60, 14, 1, C_TEXT_DIM);
    setText(webPanelPrevStatus, sizeof(webPanelPrevStatus), webStatus);
  }
}

static void drawLaserControlScreen() {
  if (!laserStaticDrawn) {
    drawPanelChrome("LASER");
    tft.fillRoundRect(18, 28, tft.width() - 36, 30, 6, C_BG);
    drawCenteredBand("OK toggle", 62, 10, 1, C_HILITE);
    laserStaticDrawn = true;
    laserPrevEnabled = !laserEnabled;
  }

  if (laserPrevEnabled != laserEnabled) {
    tft.fillRoundRect(18, 28, tft.width() - 36, 30, 6, C_BG);
    drawCentered(laserEnabled ? "WLACZONY" : "WYLACZONY", 40, 1, laserEnabled ? C_ACCENT : C_TEXT);
    laserPrevEnabled = laserEnabled;
  }
}

static bool isUnsupportedForWrite(decode_type_t protocol) {
  return (protocol == UNKNOWN || protocol == PULSE_WIDTH || protocol == PULSE_DISTANCE);
}

static bool isKnownSignalEmpty(const IRData &d) {
  return (d.numberOfBits == 0) &&
         (d.decodedRawData == 0) &&
         (d.address == 0) &&
         (d.command == 0);
}

static void sanitizeStoredSignals() {
  uint8_t writeIdx = 0;
  for (uint8_t i = 0; i < savedCount; i++) {
    const StoredSignal &src = savedSignals[i];
    bool keep = true;
    if (src.isRaw) {
      keep = (src.rawCodeLength >= 12);
    } else {
      keep = !isKnownSignalEmpty(src.data) && (src.data.numberOfBits >= 8);
    }

    if (keep) {
      if (writeIdx != i) {
        savedSignals[writeIdx] = src;
      }
      writeIdx++;
    }
  }

  if (writeIdx < savedCount) {
    for (uint8_t i = writeIdx; i < savedCount; i++) {
      std::memset(&savedSignals[i], 0, sizeof(StoredSignal));
    }
    savedCount = writeIdx;
    if (savedCount == 0) {
      sendIndex = 0;
    } else if (sendIndex >= savedCount) {
      sendIndex = savedCount - 1;
    }
    persistSignals();
  }
}

static void pushSignal(const StoredSignal &signal) {
  if (savedCount >= MAX_SIGNALS) {
    for (uint8_t i = 1; i < MAX_SIGNALS; i++) {
      savedSignals[i - 1] = savedSignals[i];
    }
    savedCount = MAX_SIGNALS - 1;
    if (sendIndex > 0) {
      sendIndex--;
    }
  }
  savedSignals[savedCount++] = signal;
  sendIndex = savedCount - 1;
  persistSignals();
}

static bool removeSignalAt(uint8_t index) {
  if (savedCount == 0 || index >= savedCount) {
    return false;
  }

  for (uint8_t i = index + 1; i < savedCount; i++) {
    savedSignals[i - 1] = savedSignals[i];
  }

  savedCount--;
  if (savedCount == 0) {
    sendIndex = 0;
  } else if (sendIndex >= savedCount) {
    sendIndex = savedCount - 1;
  }

  persistSignals();
  return true;
}

static bool removeSelectedSignal() {
  return removeSignalAt(sendIndex);
}

static void handleCaptureFrame() {
  if (!IrReceiver.decode()) {
    return;
  }

  IRData data = IrReceiver.decodedIRData;
  const uint32_t nowMs = millis();

  if ((nowMs - captureEnteredMs) < CAPTURE_ARM_DELAY_MS) {
    IrReceiver.resume();
    return;
  }
  if ((nowMs - lastSavedSignalMs) < CAPTURE_MIN_INTERVAL_MS) {
    IrReceiver.resume();
    return;
  }

  if (data.flags & IRDATA_FLAGS_IS_REPEAT) {
    IrReceiver.resume();
    return;
  }
  if (data.flags & IRDATA_FLAGS_IS_AUTO_REPEAT) {
    IrReceiver.resume();
    return;
  }
  if (data.flags & IRDATA_FLAGS_PARITY_FAILED) {
    IrReceiver.resume();
    return;
  }
  if (data.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
    Serial.println("IR RX overflow");
    IrReceiver.resume();
    return;
  }

  StoredSignal signal{};
  signal.data = data;
  signal.data.flags = IRDATA_FLAGS_EMPTY;
  signal.isRaw = isUnsupportedForWrite(signal.data.protocol);
  signal.rawCodeLength = 0;

  if (signal.isRaw) {
    Serial.printf("IR RX ignored unknown: bits=%u rawlen=%u hash=0x%llX\n",
                  signal.data.numberOfBits,
                  signal.data.rawlen,
                  static_cast<unsigned long long>(signal.data.decodedRawData));
    IrReceiver.resume();
    return;
  } else {
    if (isKnownSignalEmpty(signal.data) || signal.data.numberOfBits < 8) {
      Serial.println("IR RX empty known frame ignored");
      IrReceiver.resume();
      return;
    }
    char line[40];
    std::snprintf(line, sizeof(line), "Dodano %s", getProtocolString(signal.data.protocol));
    setText(captureStatus, sizeof(captureStatus), line);
    Serial.printf("IR RX saved: proto=%s addr=0x%04X cmd=0x%04X bits=%u raw=0x%llX\n",
                  getProtocolString(signal.data.protocol),
                  signal.data.address,
                  signal.data.command,
                  signal.data.numberOfBits,
                  static_cast<unsigned long long>(signal.data.decodedRawData));
  }

  pushSignal(signal);
  lastSavedSignalMs = nowMs;
  IrReceiver.resume();
  showMainStatus("Zapisano 1 sygnal");
  uiScreen = UiScreen::MainMenu;
  uiDirty = true;
}

static bool sendSelectedSignal() {
  return sendStoredSignalByIndex(sendIndex);
}

static String buildSignalLabel(uint8_t index) {
  if (index >= savedCount) {
    return String();
  }
  const StoredSignal &s = savedSignals[index];
  char line[48];
  if (s.isRaw) {
    std::snprintf(line, sizeof(line), "#%u RAW len:%u", index + 1, s.rawCodeLength);
  } else {
    std::snprintf(line,
                  sizeof(line),
                  "#%u %s %04X/%04X",
                  index + 1,
                  getProtocolString(s.data.protocol),
                  s.data.address,
                  s.data.command);
  }
  return String(line);
}

static void sendWebRedirect() {
  webServer.sendHeader("Location", "/", true);
  webServer.send(303, "text/plain", "");
}

static void handleWebRoot() {
  noteInteraction();

  String html;
  html.reserve(18000);
  html += F("<!doctype html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>ESP32 IR</title><meta http-equiv='refresh' content='12'>");
  html += F("<style>");
  html += F(":root{--bg:#03162d;--bg2:#072850;--panel:rgba(9,28,57,.84);--line:rgba(125,206,255,.18);--text:#eef7ff;--muted:#8db5d9;--accent:#5ddcff;--accent2:#87f6c2;--danger:#ff9191;--shadow:0 18px 46px rgba(0,0,0,.28);}*{box-sizing:border-box}");
  html += F("body{margin:0;color:var(--text);font-family:Verdana,sans-serif;background:radial-gradient(circle at top,#0b4f8f 0%,#072850 26%,#03162d 70%);}");
  html += F(".wrap{max-width:900px;margin:0 auto;padding:14px 14px 28px}");
  html += F(".hero,.card{background:var(--panel);backdrop-filter:blur(10px);border:1px solid var(--line);border-radius:18px;box-shadow:var(--shadow)}");
  html += F(".hero{padding:18px;margin-bottom:14px}.hero-top{display:flex;gap:14px;justify-content:space-between;align-items:flex-start;flex-wrap:wrap}");
  html += F("h1,h2,h3{margin:0}h1{font-size:24px;letter-spacing:.02em}h2{font-size:17px;margin-bottom:12px}h3{font-size:13px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}");
  html += F(".sub{color:var(--muted);font-size:13px;margin-top:6px;line-height:1.45}.status{margin-top:14px;padding:12px 14px;border-radius:14px;background:rgba(93,220,255,.1);border:1px solid rgba(93,220,255,.16)}");
  html += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px;margin-bottom:14px}");
  html += F(".card{padding:14px}.stats{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:14px}.stat{padding:12px;border-radius:14px;background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.07)}");
  html += F(".stat strong{display:block;font-size:18px;margin-top:4px}.actions,.toolbar{display:flex;gap:8px;flex-wrap:wrap}.toolbar{margin-top:12px}");
  html += F(".row{display:flex;gap:10px;align-items:center;justify-content:space-between;padding:10px 0;border-top:1px solid rgba(255,255,255,.09)}.row:first-of-type{border-top:0;padding-top:0}");
  html += F(".name{font-size:14px;font-weight:700;line-height:1.35}.muted{color:var(--muted);font-size:12px;line-height:1.35}");
  html += F(".btn{display:inline-flex;align-items:center;justify-content:center;min-height:38px;padding:9px 12px;border-radius:12px;font-weight:700;font-size:13px;text-decoration:none;color:#08213e;background:var(--accent)}");
  html += F(".btn.alt{background:var(--accent2)}.btn.dark{background:#b6d8ff}.btn.danger{background:var(--danger)}.btn.ghost{background:rgba(255,255,255,.07);color:var(--text);border:1px solid rgba(255,255,255,.08)}.btn.small{min-height:34px;padding:7px 10px;font-size:12px}");
  html += F(".section-title{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:12px}.list{max-height:420px;overflow:auto;padding-right:2px}");
  html += F("@media (max-width:640px){.wrap{padding:12px}.hero,.card{border-radius:16px}.stats{grid-template-columns:1fr}.row{align-items:flex-start}.actions{width:100%}.actions .btn{flex:1}}");
  html += F("</style></head><body><div class='wrap'>");
  html += F("<div class='hero'><div class='hero-top'><div><h1>ESP32 IR Panel</h1><div class='sub'>Minimalny pilot Wi-Fi do IR. Najwazniejsze akcje sa na gorze, lista sygnalow nizej.</div></div>");
  html += F("<div class='muted'>SSID: ");
  html += htmlEscape(String(webApSsid));
  html += F("<br>PASS: ");
  html += htmlEscape(String(WEB_AP_PASSWORD));
  html += F("<br>IP: ");
  html += htmlEscape(String(webApIpText));
  html += F("</div></div><div class='status'>");
  html += htmlEscape(String(webStatus));
  html += F("</div><div class='stats'>");
  html += F("<div class='stat'><h3>Zapisane</h3><strong>");
  html += String(savedCount);
  html += F("</strong></div>");
  html += F("<div class='stat'><h3>Uniwersal</h3><strong>");
  html += (tvBlaster.isRunning() ? F("Aktywny") : F("Stop"));
  html += F("</strong></div>");
  html += F("<div class='stat'><h3>Proj Fast</h3><strong>");
  html += (projectorFastRunning ? F("Aktywny") : F("Stop"));
  html += F("</strong></div></div>");
  html += F("<div class='toolbar'>");
  html += F("<a class='btn' href='/universal/start'>Start Uniwersal</a>");
  html += F("<a class='btn alt' href='/projfast/start'>Start Proj FAST</a>");
  html += F("<a class='btn ghost' href='/stop'>Stop skany</a>");
  html += F("<a class='btn danger' href='/clear-all'>Usun wszystko</a>");
  html += F("</div></div>");

  html += F("<div class='grid'>");
  html += F("<div class='card'><div class='section-title'><h2>Szybkie sterowanie</h2></div><div class='actions'>");
  html += F("<a class='btn small' href='/universal/start'>Uniwersal start</a>");
  html += F("<a class='btn ghost small' href='/universal/stop'>Uniwersal stop</a>");
  html += F("<a class='btn alt small' href='/projfast/start'>Proj FAST start</a>");
  html += F("<a class='btn ghost small' href='/projfast/stop'>Proj FAST stop</a>");
  html += F("</div><div class='sub' style='margin-top:12px'>Uniwersal i Proj FAST zatrzymuja sie po zakonczeniu listy. Panel odswieza sie sam co 12 s.</div></div>");
  html += F("<div class='card'><div class='section-title'><h2>Status urzadzenia</h2></div>");
  html += F("<div class='row'><div><div class='name'>Tryb Wi-Fi AP</div><div class='muted'>Wlasna siec lokalna do sterowania z telefonu</div></div><div class='muted'>ON</div></div>");
  html += F("<div class='row'><div><div class='name'>IR lista</div><div class='muted'>Zapisane i stale przechowywane w NVS</div></div><div class='muted'>");
  html += String(savedCount);
  html += F(" / ");
  html += String(MAX_SIGNALS);
  html += F("</div></div>");
  html += F("<div class='row'><div><div class='name'>Adres panelu</div><div class='muted'>Otworz z telefonu w przegladarce</div></div><div class='muted'>");
  html += htmlEscape(String(webApIpText));
  html += F("</div></div></div></div>");

  html += F("<div class='card'><div class='section-title'><h2>Zapisane IR</h2><a class='btn ghost small' href='/'>Odswiez</a></div>");
  if (savedCount == 0) {
    html += F("<div class='muted'>Brak zapisanych sygnalow.</div>");
  } else {
    html += F("<div class='list'>");
    for (uint8_t i = 0; i < savedCount; i++) {
      html += F("<div class='row'><div><div class='name'>");
      html += htmlEscape(buildSignalLabel(i));
      html += F("</div><div class='muted'>Slot ");
      html += String(i + 1);
      html += F("</div></div><div class='actions'>");
      html += F("<a class='btn small' href='/send?slot=");
      html += String(i);
      html += F("'>Wyslij</a> ");
      html += F("<a class='btn danger small' href='/delete?slot=");
      html += String(i);
      html += F("'>Usun</a></div></div>");
    }
    html += F("</div>");
  }
  html += F("</div>");

  html += F("<div class='grid'>");
  html += F("<div class='card'><div class='section-title'><h2>TV 20</h2></div><div class='list'>");
  for (uint8_t i = 0; i < TV_BASIC_COUNT; i++) {
    html += F("<div class='row'><div><div class='name'>");
    html += htmlEscape(String(TV20_PRESETS[i].name));
    html += F("</div><div class='muted'>Preset TV power</div></div><div class='actions'><a class='btn alt small' href='/tv20?slot=");
    html += String(i);
    html += F("'>Power</a></div></div>");
  }
  html += F("</div></div>");

  html += F("<div class='card'><div class='section-title'><h2>Proj 20</h2></div><div class='list'>");
  for (uint8_t i = 0; i < PROJECTOR_BASIC_COUNT; i++) {
    html += F("<div class='row'><div><div class='name'>");
    html += htmlEscape(String(PROJECTOR20_PRESETS[i].name));
    html += F("</div><div class='muted'>Preset projector power</div></div><div class='actions'><a class='btn alt small' href='/proj20?slot=");
    html += String(i);
    html += F("'>Power</a></div></div>");
  }
  html += F("</div></div></div></div></body></html>");

  webServer.send(200, "text/html; charset=utf-8", html);
}

static void handleWebSendStored() {
  noteInteraction();
  if (!webServer.hasArg("slot")) {
    setText(webStatus, sizeof(webStatus), "Brak slot");
    sendWebRedirect();
    return;
  }

  const int slot = webServer.arg("slot").toInt();
  if (slot < 0 || slot >= savedCount) {
    setText(webStatus, sizeof(webStatus), "Zly slot IR");
    sendWebRedirect();
    return;
  }

  if (sendStoredSignalByIndex(static_cast<uint8_t>(slot))) {
    setText(webStatus, sizeof(webStatus), "WWW: wyslano IR");
  } else {
    setText(webStatus, sizeof(webStatus), "WWW: blad IR");
  }
  uiDirty = (uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebDeleteStored() {
  noteInteraction();
  if (!webServer.hasArg("slot")) {
    setText(webStatus, sizeof(webStatus), "Brak slot");
    sendWebRedirect();
    return;
  }

  const int slot = webServer.arg("slot").toInt();
  if (slot < 0 || slot >= savedCount) {
    setText(webStatus, sizeof(webStatus), "Zly slot del");
    sendWebRedirect();
    return;
  }

  if (removeSignalAt(static_cast<uint8_t>(slot))) {
    if (savedCount == 0) {
      sendIndex = 0;
    } else if (sendIndex >= savedCount) {
      sendIndex = savedCount - 1;
    }
    setText(webStatus, sizeof(webStatus), "WWW: usunieto");
  } else {
    setText(webStatus, sizeof(webStatus), "WWW: blad del");
  }
  uiDirty = (uiScreen == UiScreen::WebPanel || uiScreen == UiScreen::SendList);
  sendWebRedirect();
}

static void handleWebSendTv20() {
  noteInteraction();
  if (!webServer.hasArg("slot")) {
    setText(webStatus, sizeof(webStatus), "Brak TV slot");
    sendWebRedirect();
    return;
  }

  const int slot = webServer.arg("slot").toInt();
  if (slot < 0 || slot >= TV_BASIC_COUNT) {
    setText(webStatus, sizeof(webStatus), "Zly TV slot");
    sendWebRedirect();
    return;
  }

  tv20Index = static_cast<uint8_t>(slot);
  if (sendTvBasic20Selected()) {
    setText(webStatus, sizeof(webStatus), "WWW: TV20");
  } else {
    setText(webStatus, sizeof(webStatus), "WWW: blad TV");
  }
  uiDirty = (uiScreen == UiScreen::TvBasic20 || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebSendProj20() {
  noteInteraction();
  if (!webServer.hasArg("slot")) {
    setText(webStatus, sizeof(webStatus), "Brak PROJ slot");
    sendWebRedirect();
    return;
  }

  const int slot = webServer.arg("slot").toInt();
  if (slot < 0 || slot >= PROJECTOR_BASIC_COUNT) {
    setText(webStatus, sizeof(webStatus), "Zly PROJ slot");
    sendWebRedirect();
    return;
  }

  projector20Index = static_cast<uint8_t>(slot);
  if (sendProjector20Selected()) {
    setText(webStatus, sizeof(webStatus), "WWW: PROJ20");
  } else {
    setText(webStatus, sizeof(webStatus), "WWW: blad PROJ");
  }
  uiDirty = (uiScreen == UiScreen::Projector20 || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebUniversalStart() {
  noteInteraction();
  stopProjectorFast();
  startUniversalBlast();
  setText(webStatus, sizeof(webStatus), "WWW: Uniwersal start");
  uiDirty = (uiScreen == UiScreen::UniversalTv || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebUniversalStop() {
  noteInteraction();
  tvBlaster.stop();
  setText(webStatus, sizeof(webStatus), "WWW: Uniwersal stop");
  uiDirty = (uiScreen == UiScreen::UniversalTv || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebProjFastStart() {
  noteInteraction();
  tvBlaster.stop();
  startProjectorFast();
  setText(webStatus, sizeof(webStatus), "WWW: Proj FAST");
  uiDirty = (uiScreen == UiScreen::ProjectorFast || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebProjFastStop() {
  noteInteraction();
  stopProjectorFast();
  setText(webStatus, sizeof(webStatus), "WWW: Proj stop");
  uiDirty = (uiScreen == UiScreen::ProjectorFast || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebStopAll() {
  noteInteraction();
  stopAllAutomations();
  setText(webStatus, sizeof(webStatus), "WWW: Stop");
  uiDirty = (uiScreen == UiScreen::UniversalTv || uiScreen == UiScreen::ProjectorFast || uiScreen == UiScreen::WebPanel);
  sendWebRedirect();
}

static void handleWebClearAll() {
  noteInteraction();
  clearSavedSignals(true);
  setText(webStatus, sizeof(webStatus), "WWW: wyczyszczono");
  uiDirty = (uiScreen == UiScreen::WebPanel || uiScreen == UiScreen::SendList);
  sendWebRedirect();
}

static void handleWebNotFound() {
  webServer.send(404, "text/plain", "404");
}

static void startWebPanel() {
  const uint64_t chipId = ESP.getEfuseMac();
  std::snprintf(webApSsid, sizeof(webApSsid), "ESP32-IR-%04X", static_cast<uint16_t>(chipId & 0xFFFF));

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const bool apOk = WiFi.softAP(webApSsid, WEB_AP_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  std::snprintf(webApIpText, sizeof(webApIpText), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  setText(webStatus, sizeof(webStatus), apOk ? "AP aktywny" : "AP blad");

  webServer.on("/", HTTP_GET, handleWebRoot);
  webServer.on("/send", HTTP_GET, handleWebSendStored);
  webServer.on("/delete", HTTP_GET, handleWebDeleteStored);
  webServer.on("/tv20", HTTP_GET, handleWebSendTv20);
  webServer.on("/proj20", HTTP_GET, handleWebSendProj20);
  webServer.on("/universal/start", HTTP_GET, handleWebUniversalStart);
  webServer.on("/universal/stop", HTTP_GET, handleWebUniversalStop);
  webServer.on("/projfast/start", HTTP_GET, handleWebProjFastStart);
  webServer.on("/projfast/stop", HTTP_GET, handleWebProjFastStop);
  webServer.on("/stop", HTTP_GET, handleWebStopAll);
  webServer.on("/clear-all", HTTP_GET, handleWebClearAll);
  webServer.onNotFound(handleWebNotFound);
  webServer.begin();

  Serial.printf("WEB AP: ssid=%s ip=%s ok=%d\n", webApSsid, webApIpText, apOk ? 1 : 0);
}

static void openSelectedMainMenuItem() {
  if (selectedMain == 0) {
    setText(captureStatus, sizeof(captureStatus), "Czeka na sygnal...");
    captureEnteredMs = millis();
    invalidateAllUiCaches();
    uiScreen = UiScreen::Capture;
  } else if (selectedMain == 1) {
    setText(sendStatus, sizeof(sendStatus), "");
    invalidateAllUiCaches();
    uiScreen = UiScreen::SendList;
  } else if (selectedMain == 2) {
    stopProjectorFast();
    startUniversalBlast();
    invalidateAllUiCaches();
    uiScreen = UiScreen::UniversalTv;
  } else if (selectedMain == 3) {
    tvBlaster.stop();
    stopProjectorFast();
    setText(tv20Status, sizeof(tv20Status), "");
    invalidateAllUiCaches();
    uiScreen = UiScreen::TvBasic20;
  } else if (selectedMain == 4) {
    tvBlaster.stop();
    stopProjectorFast();
    setText(projector20Status, sizeof(projector20Status), "");
    invalidateAllUiCaches();
    uiScreen = UiScreen::Projector20;
  } else if (selectedMain == 5) {
    tvBlaster.stop();
    startProjectorFast();
    invalidateAllUiCaches();
    uiScreen = UiScreen::ProjectorFast;
  } else if (selectedMain == 6) {
    tvBlaster.stop();
    stopProjectorFast();
    invalidateAllUiCaches();
    uiScreen = UiScreen::WebPanel;
  } else if (selectedMain == 7) {
    tvBlaster.stop();
    stopProjectorFast();
    invalidateAllUiCaches();
    uiScreen = UiScreen::LaserControl;
  } else if (selectedMain == 8) {
    tvBlaster.stop();
    stopProjectorFast();
    setText(badkbStatus, sizeof(badkbStatus), "");
    bleKeyboard.begin();
    invalidateAllUiCaches();
    uiScreen = UiScreen::BadKeyboard;
  }
  uiDirty = true;
}

static void renderUi() {
  if (!uiDirty) {
    return;
  }

  if (uiScreen == UiScreen::Splash) {
    drawSplashScreen();
  } else if (uiScreen == UiScreen::MainMenu) {
    drawMainMenu();
  } else if (uiScreen == UiScreen::Capture) {
    drawCaptureScreen();
  } else if (uiScreen == UiScreen::SendList) {
    drawSendListScreen();
  } else if (uiScreen == UiScreen::UniversalTv) {
    drawUniversalTvScreen();
  } else if (uiScreen == UiScreen::TvBasic20) {
    drawTvBasic20Screen();
  } else if (uiScreen == UiScreen::Projector20) {
    drawProjector20Screen();
  } else if (uiScreen == UiScreen::ProjectorFast) {
    drawProjectorFastScreen();
  } else if (uiScreen == UiScreen::LaserControl) {
    drawLaserControlScreen();
  } else if (uiScreen == UiScreen::BadKeyboard) {
    drawBadKeyboardScreen();
  } else {
    drawWebPanelScreen();
  }

  uiDirty = false;
}

static void drawBadKeyboardScreen() {
  const BadKbPreset &preset = BADKB_PRESETS[badkbPresetIndex];
  if (!badkbStaticDrawn) {
    drawPanelChrome("BAD KB");
    drawNavArrows();
    tft.fillRoundRect(18, 28, tft.width() - 36, 28, 6, C_BG);
    drawCenteredBand("OK run", 62, 10, 1, C_HILITE);
    badkbStaticDrawn = true;
    badkbPrevIndex = 0xFF;
    badkbPrevStatus[0] = '\0';
  }
  if (badkbPrevIndex != badkbPresetIndex) {
    tft.fillRoundRect(18, 28, tft.width() - 36, 28, 6, C_BG);
    drawCenteredBand(preset.name, 31, 10, 1, C_TEXT);
    char idxLine[22];
    std::snprintf(idxLine, sizeof(idxLine), "[%u/%u]", badkbPresetIndex + 1, BADKB_PRESET_COUNT);
    drawCenteredBand(idxLine, 44, 10, 1, C_TEXT_DIM);
    badkbPrevIndex = badkbPresetIndex;
  }
  char status[40];
  if (bleKeyboard.isConnected()) {
    std::snprintf(status, sizeof(status), "Polaczono");
  } else {
    std::snprintf(status, sizeof(status), "Czekam na BT...");
  }
  if (std::strncmp(badkbPrevStatus, status, sizeof(badkbPrevStatus)) != 0) {
    drawCenteredBand(status, 72, 8, 1, C_TEXT_DIM);
    setText(badkbPrevStatus, sizeof(badkbPrevStatus), status);
  }
}

static void handleInput() {
  const uint32_t nowMs = millis();
  updateButton(btnHome, nowMs, DEBOUNCE_MS, EDGE_LOCK_MS);
  updateButton(btnLeft, nowMs, DEBOUNCE_MS, EDGE_LOCK_MS);
  updateButton(btnRight, nowMs, DEBOUNCE_MS, EDGE_LOCK_MS);
  updateButton(btnOk, nowMs, DEBOUNCE_MS, EDGE_LOCK_MS);
  updateButton(btnBack, nowMs, DEBOUNCE_MS, EDGE_LOCK_MS);

  if (displaySleeping) {
    const bool wakePress = consumePress(btnHome) || consumePress(btnLeft) || consumePress(btnRight) ||
                           consumePress(btnOk) || consumePress(btnBack);
    if (wakePress) {
      wakeDisplay();
    }
    return;
  }

  if (consumePress(btnHome)) {
    noteInteraction();
    if (uiScreen == UiScreen::UniversalTv) {
      tvBlaster.stop();
    } else if (uiScreen == UiScreen::ProjectorFast) {
      stopProjectorFast();
    }
    invalidateAllUiCaches();
    uiScreen = UiScreen::Splash;
    uiDirty = true;
    return;
  }

  if (uiScreen == UiScreen::Splash) {
    if (consumePress(btnLeft) || consumePress(btnRight) || consumePress(btnOk) || consumePress(btnBack)) {
      noteInteraction();
      invalidateAllUiCaches();
      uiScreen = UiScreen::MainMenu;
      uiDirty = true;
    }
    return;
  }

  if (consumePress(btnLeft)) {
    noteInteraction();
    if (uiScreen == UiScreen::MainMenu) {
      selectedMain = (selectedMain == 0) ? (MAIN_MENU_COUNT - 1) : (selectedMain - 1);
      uiDirty = true;
    } else if (uiScreen == UiScreen::SendList && savedCount > 0) {
      sendIndex = (sendIndex == 0) ? (savedCount - 1) : (sendIndex - 1);
      uiDirty = true;
    } else if (uiScreen == UiScreen::TvBasic20) {
      tv20Index = (tv20Index == 0) ? (TV_BASIC_COUNT - 1) : (tv20Index - 1);
      uiDirty = true;
    } else if (uiScreen == UiScreen::Projector20) {
      projector20Index = (projector20Index == 0) ? (PROJECTOR_BASIC_COUNT - 1) : (projector20Index - 1);
      uiDirty = true;
    } else if (uiScreen == UiScreen::BadKeyboard) {
      badkbPresetIndex = (badkbPresetIndex == 0) ? (BADKB_PRESET_COUNT - 1) : (badkbPresetIndex - 1);
      uiDirty = true;
    }
  }

  const bool rightPress = consumePress(btnRight);
  if (rightPress) {
    noteInteraction();
    if (uiScreen == UiScreen::MainMenu) {
      selectedMain = (selectedMain + 1) % MAIN_MENU_COUNT;
      uiDirty = true;
    } else if (uiScreen == UiScreen::SendList && savedCount > 0) {
      sendIndex = (sendIndex + 1) % savedCount;
      uiDirty = true;
    } else if (uiScreen == UiScreen::TvBasic20) {
      tv20Index = (tv20Index + 1) % TV_BASIC_COUNT;
      uiDirty = true;
    } else if (uiScreen == UiScreen::Projector20) {
      projector20Index = (projector20Index + 1) % PROJECTOR_BASIC_COUNT;
      uiDirty = true;
    } else if (uiScreen == UiScreen::BadKeyboard) {
      badkbPresetIndex = (badkbPresetIndex + 1) % BADKB_PRESET_COUNT;
      uiDirty = true;
    }
  }

  if (uiScreen == UiScreen::MainMenu) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      openSelectedMainMenuItem();
    }
  } else if (uiScreen == UiScreen::SendList) {
    consumePress(btnOk); // hold logic has priority in this screen
    const bool okActive = isButtonActive(btnOk);

    if (okActive) {
      if (!okSendHoldTracking) {
        okSendHoldTracking = true;
        okSendHoldDoneSingle = false;
        okSendHoldDoneAll = false;
        okSendHoldStartMs = nowMs;
      } else {
        const uint32_t holdMs = nowMs - okSendHoldStartMs;
        if (!okSendHoldDoneAll && holdMs >= OK_HOLD_DELETE_ALL_MS) {
          clearSavedSignals(true);
          setText(sendStatus, sizeof(sendStatus), "Usunieto wszystkie");
          okSendHoldDoneAll = true;
          okSendHoldDoneSingle = true;
          uiDirty = true;
        } else if (!okSendHoldDoneSingle && holdMs >= OK_HOLD_DELETE_ONE_MS) {
          if (removeSelectedSignal()) {
            setText(sendStatus, sizeof(sendStatus), "Usunieto sygnal");
          } else {
            setText(sendStatus, sizeof(sendStatus), "Brak sygnalu");
          }
          okSendHoldDoneSingle = true;
          uiDirty = true;
        }
      }
    } else if (okSendHoldTracking) {
      if (!okSendHoldDoneSingle && !okSendHoldDoneAll) {
        noteInteraction();
        if (sendSelectedSignal()) {
          setText(sendStatus, sizeof(sendStatus), "Wyslano");
        } else {
          setText(sendStatus, sizeof(sendStatus), "Nie wyslano");
        }
        uiDirty = true;
      }
      okSendHoldTracking = false;
      okSendHoldDoneSingle = false;
      okSendHoldDoneAll = false;
    }
  } else if (uiScreen == UiScreen::UniversalTv) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      if (tvBlaster.isRunning()) {
        tvBlaster.stop();
        setText(tvStatus, sizeof(tvStatus), "");
      } else {
        stopProjectorFast();
        startUniversalBlast();
      }
      uiDirty = true;
    }
  } else if (uiScreen == UiScreen::TvBasic20) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      if (sendTvBasic20Selected()) {
        setText(tv20Status, sizeof(tv20Status), "Wyslano");
      } else {
        setText(tv20Status, sizeof(tv20Status), "Blad");
      }
      uiDirty = true;
    }
  } else if (uiScreen == UiScreen::Projector20) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      if (sendProjector20Selected()) {
        setText(projector20Status, sizeof(projector20Status), "Wyslano");
      } else {
        setText(projector20Status, sizeof(projector20Status), "Blad");
      }
      uiDirty = true;
    }
  } else if (uiScreen == UiScreen::ProjectorFast) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      if (projectorFastRunning) {
        stopProjectorFast();
        setText(projectorFastStatus, sizeof(projectorFastStatus), "Stop");
      } else {
        startProjectorFast();
      }
      uiDirty = true;
    }
  } else if (uiScreen == UiScreen::LaserControl) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      laserEnabled = !laserEnabled;
      digitalWrite(LASER_PIN, laserEnabled ? HIGH : LOW);
      uiDirty = true;
    }
  } else if (uiScreen == UiScreen::WebPanel) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      uiDirty = true;
    }
  } else if (uiScreen == UiScreen::BadKeyboard) {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      noteInteraction();
      if (bleKeyboard.isConnected()) {
        BADKB_PRESETS[badkbPresetIndex].run();
        setText(badkbStatus, sizeof(badkbStatus), "Wyslano!");
      } else {
        setText(badkbStatus, sizeof(badkbStatus), "Brak polaczenia");
      }
      uiDirty = true;
    }
  } else {
    okSendHoldTracking = false;
    okSendHoldDoneSingle = false;
    okSendHoldDoneAll = false;
    if (consumePress(btnOk)) {
      // Capture screen uses automatic save on IR receive only.
    }
  }

  if (consumePress(btnBack)) {
    noteInteraction();
    if (uiScreen == UiScreen::Capture || uiScreen == UiScreen::SendList || uiScreen == UiScreen::UniversalTv ||
        uiScreen == UiScreen::TvBasic20 || uiScreen == UiScreen::Projector20 || uiScreen == UiScreen::ProjectorFast ||
        uiScreen == UiScreen::WebPanel || uiScreen == UiScreen::LaserControl || uiScreen == UiScreen::BadKeyboard) {
      if (uiScreen == UiScreen::UniversalTv) {
        tvBlaster.stop();
      } else if (uiScreen == UiScreen::ProjectorFast) {
        stopProjectorFast();
      } else if (uiScreen == UiScreen::BadKeyboard) {
        bleKeyboard.end();
      }
      invalidateAllUiCaches();
      uiScreen = UiScreen::MainMenu;
      uiDirty = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  digitalWrite(TFT_RST, LOW);
  delay(40);
  digitalWrite(TFT_RST, HIGH);
  delay(120);

  tft.initR(TFT_INIT);
  tft.setSPISpeed(TFT_SPI_HZ);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  initButton(btnHome);
  initButton(btnLeft);
  initButton(btnRight);
  initButton(btnOk);
  initButton(btnBack);

  pinMode(TX_FEEDBACK_LED_PIN, OUTPUT);
  digitalWrite(TX_FEEDBACK_LED_PIN, LOW);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  laserEnabled = false;

  IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
  IrSender.begin(IR_TX_PIN, TX_FEEDBACK_LED_PIN);
  tvBlaster.begin(TvBgoneRegion::EU);
  tvBlaster.setCodeLimit(0);
  tvBlaster.setLooping(false);
  startWebPanel();
  lastInteractionMs = millis();

  if (loadSignalsFromNvs(savedCount, persistedSignals, MAX_SIGNALS)) {
    Serial.printf("NVS load: count=%u ok=1\n", savedCount);
    copyPersistedToRuntime();
    if (savedCount > MAX_SIGNALS) {
      savedCount = MAX_SIGNALS;
    }
    sanitizeStoredSignals();
    if (savedCount == 0) {
      sendIndex = 0;
    } else if (sendIndex >= savedCount) {
      sendIndex = savedCount - 1;
    }
    showMainStatus("Wczytano sygnaly");
  } else {
    Serial.println("NVS load: ok=0, init empty store");
    clearSavedSignals(false);
    persistSignals();
    showMainStatus("Brak danych NVS");
  }

  invalidateAllUiCaches();
  uiScreen = UiScreen::Splash;
  uiDirty = true;
  renderUi();
}

void loop() {
  webServer.handleClient();
  handleInput();
  if (!displaySleeping && (millis() - lastInteractionMs) >= UI_SLEEP_MS) {
    sleepDisplay();
    return;
  }
  if (uiScreen == UiScreen::MainMenu && mainStatus[0] != '\0' && millis() > mainStatusUntilMs) {
    uiDirty = true;
  }
  if (uiScreen == UiScreen::UniversalTv) {
    const bool wasRunning = tvBlaster.isRunning();
    const uint16_t sentBefore = tvBlaster.sentCodes();
    const uint16_t totalNow = tvBlaster.totalCodes();
    tvBlaster.tick();
    const bool runningNow = tvBlaster.isRunning();
    const bool sentChanged = (tvBlaster.sentCodes() != sentBefore);
    const bool runningChanged = (runningNow != wasRunning);
    if (sentChanged || runningChanged) {
      if (!tvBlaster.isRunning() && wasRunning) {
        if (tvBlaster.isFinished()) {
          setText(tvStatus, sizeof(tvStatus), "Koniec");
        } else {
          setText(tvStatus, sizeof(tvStatus), "Stop");
        }
        uiDirty = true;
      } else if (runningNow && sentChanged) {
        const uint32_t now = millis();
        if (now >= tvProgressRefreshMs) {
          tvProgressRefreshMs = now + UNIVERSAL_UI_REFRESH_MS;
          uiDirty = true;
        }
      }
    }
  }
  if (uiScreen == UiScreen::ProjectorFast && projectorFastRunning) {
    const uint32_t now = millis();
    if (now >= projectorFastNextSendMs) {
      if (projectorFastIndex < PROJECTOR_BASIC_COUNT) {
        const bool ok = sendProjectorPresetByIndex(projectorFastIndex);
        if (!ok) {
          setText(projectorFastStatus, sizeof(projectorFastStatus), "Blad");
        } else {
          setText(projectorFastStatus, sizeof(projectorFastStatus), "");
        }
        projectorFastIndex++;
        projectorFastNextSendMs = millis() + PROJECTOR_FAST_STEP_MS;
        if (projectorFastIndex >= PROJECTOR_BASIC_COUNT) {
          projectorFastRunning = false;
          setText(projectorFastStatus, sizeof(projectorFastStatus), "Koniec");
          uiDirty = true;
        } else if (now >= projectorFastProgressRefreshMs) {
          projectorFastProgressRefreshMs = now + UNIVERSAL_UI_REFRESH_MS;
          uiDirty = true;
        }
      } else {
        projectorFastRunning = false;
        setText(projectorFastStatus, sizeof(projectorFastStatus), "Koniec");
        uiDirty = true;
      }
    }
  }
  if (uiScreen == UiScreen::Capture) {
    handleCaptureFrame();
  }
  renderUi();
}

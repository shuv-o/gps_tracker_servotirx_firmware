#include "drivers/status_led.h"
#include "board_pins.h"

// neopixelWrite() was renamed rgbLedWrite() in Arduino-ESP32 3.x
static inline void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_STATUS_LED, r, g, b);
#else
  neopixelWrite(PIN_STATUS_LED, r, g, b);
#endif
}

namespace led {

static Pattern s_p = BOOT;
static uint8_t s_phase = 0;

void begin() { ledWrite(0, 0, 8); }
void set(Pattern p) { s_p = p; }

void tick() {
  s_phase = (s_phase + 1) % 10;  // 1 s cycle at 100 ms ticks
  bool on = s_phase < 5;
  bool pulse = s_phase == 0;
  uint8_t r = 0, g = 0, b = 0;
  switch (s_p) {
    case BOOT:            b = 8; break;
    case NET_START:       if (on) b = 16; break;
    case MQTT_CONNECTING: g = 10; b = 10; break;
    case ONLINE_FIX:      g = 12; break;
    case ONLINE_NOFIX:    if (on) g = 12; break;
    case OFFLINE:         r = 12; g = 8; break;
    case FAULT:           if (pulse || s_phase == 2) r = 20; break;
    case OTA:             r = 10; b = 14; break;
  }
  ledWrite(r, g, b);
}

}  // namespace led

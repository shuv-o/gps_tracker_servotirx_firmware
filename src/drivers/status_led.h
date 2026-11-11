#pragma once
// RGB status LED (GPIO21). Pattern is derived from device state by the app task.
//   blue blink   : modem / network starting      cyan       : network up, MQTT connecting
//   green        : online, GNSS fix               green blink: online, no GNSS fix
//   yellow       : offline (storing data)         red blink  : fault / safe mode
//   purple       : OTA in progress
#include <Arduino.h>

namespace led {
enum Pattern : uint8_t { BOOT, NET_START, MQTT_CONNECTING, ONLINE_FIX, ONLINE_NOFIX, OFFLINE, FAULT, OTA };
void begin();
void set(Pattern p);
void tick();   // call every ~100 ms
}  // namespace led

#pragma once
// Modem control lines (all through transistor stages on the board):
//   PWRKEY GPIO7 (HIGH pulse), RESET GPIO15 (HIGH pulse), DTR GPIO16 (HIGH = awake)
//   STATUS GPIO8 (inverted: LOW = on), RI GPIO5 (inverted: rising edge = ring)
#include <Arduino.h>

namespace modemhw {
void begin();
bool isOn();
bool powerOn(uint32_t waitMs = 15000);    // PWRKEY pulse, wait for STATUS
bool powerOffHard(uint32_t waitMs = 15000);  // long PWRKEY press (use AT+CPOF first)
void hardReset();                          // RESET pulse
void setAwake(bool awake);                 // DTR
uint32_t ringCount();
}  // namespace modemhw

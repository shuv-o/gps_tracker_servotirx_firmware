#pragma once
// millis()-wrap-safe deadlines (uptime > 49 days).
// A deadline of 0 always means "due now" - so `x = 0` is a safe "run immediately".
#include <Arduino.h>

inline bool due(uint32_t deadline) { return deadline == 0 || (int32_t)(millis() - deadline) >= 0; }
inline uint32_t after(uint32_t ms) {
  uint32_t t = millis() + ms;
  return t ? t : 1;
}

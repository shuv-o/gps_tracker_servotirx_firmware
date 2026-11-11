#pragma once
// Wall clock with source priority: GNSS > network (NITZ/CCLK) > none.
#include <Arduino.h>

namespace clockx {
enum Source : uint8_t { NONE = 0, NETWORK = 1, GNSS = 2 };
// Set UTC epoch from a source. Lower-priority sources never override a
// fresh higher-priority one (GNSS wins for 1 h).
void set(uint32_t epochUtc, Source src);
bool valid();
uint32_t now();            // UTC epoch seconds, 0 if unknown
Source source();
const char* sourceName();
// Helpers
uint32_t fromYmdHms(int y, int mo, int d, int h, int mi, int s);  // UTC
}  // namespace clockx

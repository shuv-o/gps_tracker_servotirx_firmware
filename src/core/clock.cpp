#include "core/clock.h"
#include <sys/time.h>
#include "core/log.h"

namespace clockx {

static volatile Source s_src = NONE;
static volatile uint32_t s_setAtMs = 0;

void set(uint32_t epoch, Source src) {
  if (epoch < 1700000000UL) return;  // sanity: before Nov 2023 = invalid
  bool higherFresh = s_src > src && (millis() - s_setAtMs) < 3600000UL;
  if (higherFresh) return;
  uint32_t cur = now();
  if (s_src != src || cur == 0 || labs((long)cur - (long)epoch) > 2) {
    struct timeval tv = {(time_t)epoch, 0};
    settimeofday(&tv, nullptr);
    if (s_src != src) LOGI("CLK", "time set from %s: %lu", src == GNSS ? "GNSS" : "network", (unsigned long)epoch);
  }
  s_src = src;
  s_setAtMs = millis();
}

bool valid() { return s_src != NONE; }

uint32_t now() {
  if (s_src == NONE) return 0;
  return (uint32_t)time(nullptr);
}

Source source() { return s_src; }
const char* sourceName() { return s_src == GNSS ? "gnss" : s_src == NETWORK ? "net" : "none"; }

uint32_t fromYmdHms(int y, int mo, int d, int h, int mi, int s) {
  // days from civil (Howard Hinnant), UTC, no timezone involved
  y -= mo <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + (long)doe - 719468L;
  return (uint32_t)(days * 86400L + h * 3600L + mi * 60L + s);
}

}  // namespace clockx

#include "core/log.h"
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace logx {

static SemaphoreHandle_t s_mtx;
static char* s_ring = nullptr;
static size_t s_size = 0, s_head = 0;
static bool s_wrapped = false;
static volatile uint8_t s_level = LOG_INFO;
static volatile bool s_echo = true;
static uint32_t s_errors = 0, s_warns = 0;  // updated atomically

void begin(size_t ringBytes) {
  s_mtx = xSemaphoreCreateMutex();
  s_ring = (char*)(psramFound() ? ps_malloc(ringBytes) : malloc(ringBytes));
  s_size = s_ring ? ringBytes : 0;
}

void setLevel(uint8_t lvl) { s_level = lvl > (uint8_t)LOG_VERBOSE ? (uint8_t)LOG_VERBOSE : lvl; }
uint8_t level() { return s_level; }
void setSerialEcho(bool on) { s_echo = on; }
uint32_t errorCount() { return s_errors; }
uint32_t warnCount() { return s_warns; }

static void ringPut(const char* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    s_ring[s_head++] = s[i];
    if (s_head >= s_size) { s_head = 0; s_wrapped = true; }
  }
}

void write(uint8_t lvl, const char* tag, const char* fmt, ...) {
  static const char LV[] = "-EWIDV";
  char line[320];
  uint32_t ms = millis();
  int n = snprintf(line, sizeof line, "%lu.%03lu %c [%s] ", (unsigned long)(ms / 1000),
                   (unsigned long)(ms % 1000), LV[lvl < 6 ? lvl : 0], tag);
  va_list ap;
  va_start(ap, fmt);
  n += vsnprintf(line + n, sizeof line - n - 2, fmt, ap);
  va_end(ap);
  if (n > (int)sizeof line - 2) n = sizeof line - 2;
  line[n++] = '\n';
  line[n] = 0;

  if (lvl == LOG_ERROR) __atomic_fetch_add(&s_errors, 1, __ATOMIC_RELAXED);
  if (lvl == LOG_WARN) __atomic_fetch_add(&s_warns, 1, __ATOMIC_RELAXED);

  if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);
  if (s_echo) Serial.write((const uint8_t*)line, n);
  if (s_size) ringPut(line, n);
  if (s_mtx) xSemaphoreGive(s_mtx);
}

size_t tail(char* out, size_t maxBytes) {
  if (!s_size || maxBytes < 2) { if (maxBytes) out[0] = 0; return 0; }
  xSemaphoreTake(s_mtx, portMAX_DELAY);
  size_t avail = s_wrapped ? s_size : s_head;
  size_t n = avail < maxBytes - 1 ? avail : maxBytes - 1;
  size_t start = (s_head + s_size - n) % s_size;
  for (size_t i = 0; i < n; i++) out[i] = s_ring[(start + i) % s_size];
  out[n] = 0;
  xSemaphoreGive(s_mtx);
  // start at a line boundary
  char* nl = strchr(out, '\n');
  if (nl && avail > n) {
    size_t skip = nl - out + 1;
    memmove(out, out + skip, n - skip + 1);
    n -= skip;
  }
  return n;
}

}  // namespace logx

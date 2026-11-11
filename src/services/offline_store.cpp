#include "services/offline_store.h"
#include <LittleFS.h>
#include <freertos/semphr.h>
#include "core/counters.h"
#include "core/log.h"

namespace offline {

static const char* TAG = "STORE";
static const size_t SEG_BYTES = 32 * 1024;
static const uint32_t MAX_SEGS = 80;  // ~2.5 MB of the 3.4 MB partition

static SemaphoreHandle_t s_mtx;
static bool s_ok = false;
static uint32_t s_head = 0, s_tail = 0;   // segment numbers: write head, read tail
static uint32_t s_tailOfs = 0;            // read offset in tail segment
static uint32_t s_peekNext = 0;           // offset after the peeked record
static uint32_t s_count = 0;

struct Lock {
  Lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(s_mtx); }
};

static String segPath(uint32_t n) { return "/q/" + String(n) + ".log"; }

static void saveTail() {
  File f = LittleFS.open("/q/tail", "w");
  if (f) { f.printf("%lu %lu", (unsigned long)s_tail, (unsigned long)s_tailOfs); f.close(); }
}

static uint32_t countLines(const String& path, uint32_t from) {
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  f.seek(from);
  uint32_t n = 0;
  uint8_t buf[256];
  while (int k = f.read(buf, sizeof buf))
    for (int i = 0; i < k; i++) n += buf[i] == '\n';
  f.close();
  return n;
}

bool begin() {
  s_mtx = xSemaphoreCreateMutex();
  if (!LittleFS.begin(true, "/littlefs", 10, "spiffs")) {
    LOGE(TAG, "LittleFS mount failed - offline storage disabled");
    return false;
  }
  if (!LittleFS.exists("/q")) LittleFS.mkdir("/q");
  // find segment range
  uint32_t lo = UINT32_MAX, hi = 0;
  File dir = LittleFS.open("/q");
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    String n = f.name();
    if (n.endsWith(".log")) {
      uint32_t v = n.toInt();
      lo = min(lo, v);
      hi = max(hi, v);
    }
  }
  if (lo == UINT32_MAX) { lo = hi = 1; }
  s_tail = lo;
  s_head = hi;
  s_tailOfs = 0;
  File t = LittleFS.open("/q/tail", "r");
  if (t) {
    unsigned long a = 0, b = 0;
    String s = t.readString();
    if (sscanf(s.c_str(), "%lu %lu", &a, &b) == 2 && a == s_tail) s_tailOfs = b;
    t.close();
  }
  s_count = 0;
  for (uint32_t n = s_tail; n <= s_head; n++) s_count += countLines(segPath(n), n == s_tail ? s_tailOfs : 0);
  s_ok = true;
  LOGI(TAG, "offline queue: %lu records in segments %lu..%lu", (unsigned long)s_count, (unsigned long)s_tail,
       (unsigned long)s_head);
  return true;
}

bool store(const OutMsg* m) {
  if (!s_ok || !m) return false;
  Lock l;
  File f = LittleFS.open(segPath(s_head), "a");
  if (f && f.size() >= SEG_BYTES) {
    f.close();
    s_head++;
    f = LittleFS.open(segPath(s_head), "a");
  }
  if (!f) { LOGE(TAG, "open segment failed"); return false; }
  // drop oldest segment if over capacity
  if (s_head - s_tail >= MAX_SEGS) {
    uint32_t lost = countLines(segPath(s_tail), s_tailOfs);
    LittleFS.remove(segPath(s_tail));
    s_tail++;
    s_tailOfs = 0;
    s_count -= min(s_count, lost);
    CNT_ADD(q_dropped, lost);
    saveTail();
    LOGW(TAG, "store full, dropped %lu oldest records", (unsigned long)lost);
  }
  // payloads are compact JSON (no newlines); sanitise just in case
  f.printf("%s\t%u\t%u\t", m->suffix, m->qos, m->retain ? 1 : 0);
  for (uint16_t i = 0; i < m->len; i++) {
    char c = m->payload[i];
    f.write((uint8_t)(c == '\n' || c == '\r' ? ' ' : c));
  }
  f.write('\n');
  f.close();
  s_count++;
  CNT_INC(q_stored);
  return true;
}

OutMsg* peek() {
  if (!s_ok) return nullptr;
  Lock l;
  for (;;) {
    if (s_count == 0) return nullptr;
    String path = segPath(s_tail);
    File f = LittleFS.open(path, "r");
    if (!f || s_tailOfs >= f.size()) {
      if (f) f.close();
      if (s_tail >= s_head) { s_count = 0; return nullptr; }  // everything read: resync counter
      LittleFS.remove(path);
      s_tail++;
      s_tailOfs = 0;
      saveTail();
      continue;
    }
    f.seek(s_tailOfs);
    String line = f.readStringUntil('\n');
    s_peekNext = f.position();
    f.close();
    int t1 = line.indexOf('\t'), t2 = line.indexOf('\t', t1 + 1), t3 = line.indexOf('\t', t2 + 1);
    if (t1 < 1 || t2 < 0 || t3 < 0) {  // corrupt record: skip
      s_tailOfs = s_peekNext;
      if (s_count) s_count--;
      continue;
    }
    String payload = line.substring(t3 + 1);
    return bus::make(line.substring(0, t1).c_str(), payload.c_str(), payload.length(),
                     line.substring(t1 + 1, t2).toInt(), line.substring(t2 + 1, t3).toInt(), true);
  }
}

void ack() {
  if (!s_ok) return;
  Lock l;
  s_tailOfs = s_peekNext;
  if (s_count) s_count--;
  CNT_INC(q_replayed);
  static uint8_t n = 0;
  if (++n % 10 == 0 || s_count == 0) saveTail();  // limit flash writes
}

uint32_t count() { return s_count; }

uint32_t bytes() { return s_ok ? LittleFS.usedBytes() : 0; }

void clear() {
  if (!s_ok) return;
  Lock l;
  for (uint32_t n = s_tail; n <= s_head; n++) LittleFS.remove(segPath(n));
  LittleFS.remove("/q/tail");
  s_tail = s_head = s_head + 1;
  s_tailOfs = 0;
  s_count = 0;
}

}  // namespace offline

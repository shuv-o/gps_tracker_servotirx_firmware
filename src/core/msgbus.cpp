#include "core/msgbus.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "core/clock.h"
#include "core/counters.h"
#include "core/log.h"
#include "services/offline_store.h"

namespace bus {

static QueueHandle_t s_q;

void begin(size_t depth) { s_q = xQueueCreate(depth, sizeof(OutMsg*)); }

OutMsg* make(const char* suffix, const char* payload, size_t len, uint8_t qos, bool retain, bool persist) {
  OutMsg* m = (OutMsg*)calloc(1, sizeof(OutMsg));
  if (!m) return nullptr;
  m->payload = (char*)(psramFound() ? ps_malloc(len + 1) : malloc(len + 1));
  if (!m->payload) { free(m); return nullptr; }
  memcpy(m->payload, payload, len);
  m->payload[len] = 0;
  m->len = len;
  strlcpy(m->suffix, suffix, sizeof m->suffix);
  m->qos = qos;
  m->retain = retain;
  m->persist = persist;
  return m;
}

void release(OutMsg* m) {
  if (!m) return;
  free(m->payload);
  free(m);
}

bool publish(const char* suffix, const char* payload, uint8_t qos, bool retain, bool persist) {
  size_t len = strlen(payload);
  if (len > 8000) { LOGE("BUS", "payload too large (%u) for %s", (unsigned)len, suffix); return false; }
  OutMsg* m = make(suffix, payload, len, qos, retain, persist);
  if (!m) { CNT_INC(msg_dropped); return false; }
  if (s_q && xQueueSend(s_q, &m, 0) == pdTRUE) return true;
  // RAM queue full (network down for a while) -> straight to flash
  bool ok = persist && offline::store(m);
  if (!ok) CNT_INC(msg_dropped);
  release(m);
  return ok;
}

bool event(const char* name, const char* detailJson) {
  char buf[512];
  snprintf(buf, sizeof buf, "{\"ts\":%lu,\"up\":%lu,\"ev\":\"%s\",\"d\":%s}", (unsigned long)clockx::now(),
           (unsigned long)(millis() / 1000), name, detailJson && *detailJson ? detailJson : "{}");
  LOGI("EVT", "%s %s", name, detailJson ? detailJson : "");
  return publish("event", buf, 1, false, true);
}

OutMsg* take(uint32_t waitMs) {
  OutMsg* m = nullptr;
  if (s_q && xQueueReceive(s_q, &m, pdMS_TO_TICKS(waitMs)) == pdTRUE) return m;
  return nullptr;
}

size_t pending() { return s_q ? uxQueueMessagesWaiting(s_q) : 0; }

}  // namespace bus

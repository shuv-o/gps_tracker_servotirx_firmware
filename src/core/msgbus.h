#pragma once
// Outgoing message bus. Any task can publish; the cellular task delivers.
//
//   bus::publish("loc", json, /*qos*/1, /*retain*/false, /*persist*/true);
//   bus::event("ignition", "{\"on\":true}");
//
// Topic = <topic_root>/<device id>/<suffix>. Persistent messages survive
// network loss (stored in flash, replayed oldest-first on reconnect).
#include <Arduino.h>

struct OutMsg {
  char     suffix[24];
  uint8_t  qos;
  bool     retain;
  bool     persist;
  uint16_t len;
  char*    payload;   // heap (PSRAM) - owned by the message
};

namespace bus {
void begin(size_t depth = 48);
// Copies payload. Returns false if dropped.
bool publish(const char* suffix, const char* payload, uint8_t qos = 1, bool retain = false,
             bool persist = true);
// Event helper: {"ts":..,"ev":"<name>","d":<detailJson>}
bool event(const char* name, const char* detailJson = nullptr);
// Consumer side (cellular task)
OutMsg* take(uint32_t waitMs);
void release(OutMsg* m);
size_t pending();
OutMsg* make(const char* suffix, const char* payload, size_t len, uint8_t qos, bool retain, bool persist);
}  // namespace bus

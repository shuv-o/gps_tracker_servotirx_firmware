#pragma once
// SAE J1979 / ISO 15031-5 Mode 01 PIDs that are decoded and reported.
// Only PIDs the vehicle reports as supported (01 00/20/40/...) are polled.
//
// tier = poll rate: 0 every 1 s, 1 every 5 s, 2 every 30 s, 3 every 5 min
// To add a PID: one line in pid_table.cpp.
#include <Arduino.h>

struct PidDef {
  uint8_t     pid;
  const char* key;     // JSON key in the "obd" message
  const char* unit;
  uint8_t     len;     // data bytes
  uint8_t     tier;
  float (*decode)(const uint8_t* d);
};

namespace pids {
const PidDef* table(size_t& count);
const PidDef* find(uint8_t pid);
extern const uint32_t TIER_MS[4];
}  // namespace pids

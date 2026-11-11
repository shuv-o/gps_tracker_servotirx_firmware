#pragma once
// Location service (runs on the cellular task, uses the A7672 GNSS engine).
//
// Fallback chain, best available source is reported in "src":
//   gnss  : live fix with acceptable HDOP
//   agnss : no fix for agnss_after_s -> assistance data downloaded, still "gnss" when it fixes
//   lbs   : no fix for lbs_after_s  -> modem cell-tower location (if firmware supports it)
//   last  : last known good position from flash, flagged stale
// Every location message also carries the serving cell (MCC/MNC/TAC/CID) so the
// server can geolocate by cell tower even when everything else fails.
//
// Health: engine restarted after repeated errors; cold start (ephemeris purge)
// if no fix for a long time.
#include <Arduino.h>
#include <ArduinoJson.h>
#include "drivers/at_channel.h"

enum class LocSrc : uint8_t { None, Gnss, Lbs, Last };

struct LocInfo {
  bool     valid;
  LocSrc   src;
  double   lat, lon;
  float    alt, speedKmh, course, hdop, accuracyM;
  uint8_t  sats, fixMode;     // fixMode 2 = 2D, 3 = 3D
  uint32_t fixEpoch;          // UTC of the fix (0 unknown)
  uint32_t ageMs;             // since last update of this position
  uint32_t noFixSec;          // seconds without a live GNSS fix
  uint32_t ttffMs;            // time to first fix since engine start
  bool     moving;
  double   odometerKm;        // GNSS-integrated distance
  bool     engineOn;
  bool     assisted;          // AGNSS data was injected
};

namespace gnss {
void begin();                       // load last known position
void step(AtChannel& at, bool netOnline);
void powerOff(AtChannel& at);
void modemReset();                  // modem restarted: engine is off, re-init on next step
void snapshot(LocInfo& out);
const char* srcName(LocSrc s);
// commands
bool restart(AtChannel& at, const char* how);   // "cold" | "warm" | "hot" | "power"
bool requestAgnss(AtChannel& at);
void toJson(JsonObject o);
void print(Print& out);
}  // namespace gnss

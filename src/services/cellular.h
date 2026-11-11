#pragma once
// Cellular task: the ONLY owner of the modem UART.
// Runs, in one cooperative loop:
//   network state machine + recovery ladder + 4G->2G failover   (this file)
//   MQTT client (mqtt_client.cpp), GNSS (gnss.cpp), OTA (ota.cpp)
//   delivery of the message bus / offline store, incoming commands
//
// Recovery ladder (escalates on repeated failure, resets after 5 min healthy):
//   L1 PDP re-attach -> L2 radio off/on -> L3 AT+CRESET -> L4 RESET pin -> L5 power cycle (+backoff)
// RAT failover (rat_mode=0): data keeps failing on LTE -> force GSM (2G);
//   retry LTE every lte_retry_s, keep it if MQTT reconnects.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <functional>
#include "drivers/at_channel.h"

enum class NetPhase : uint8_t { Off, PowerOn, Sync, Sim, Register, Attach, Online, Recover };

struct NetInfo {
  NetPhase phase;
  uint8_t  recoveryLevel;
  bool     forced2G;
  bool     dataUp;
  char     rat[12];          // "LTE", "GSM", ...
  int      csq;              // 0..31, 99 unknown
  int      rsrp, rsrq;       // dBm / dB (0 = unknown)
  char     op[24];
  char     mccmnc[8];
  char     tac[12];
  uint32_t cellId;
  char     band[20];
  char     ip[48];
  char     imei[20], iccid[24], imsi[20], model[24], modemFw[40];
  float    vbat;
  int      modemTemp;
  uint32_t onlineSinceMs;
  char     lastFailure[64];
};

namespace cell {
void start();
void snapshot(NetInfo& out);
const char* phaseName(NetPhase p);
const char* deviceId();             // IMEI once known

// Run a job on the cellular task (with the AT channel). Blocks the caller
// until done or timeout. Called from the cellular task itself it runs inline.
// Capture by value / shared_ptr only: on timeout the job may still run later.
typedef std::function<void(AtChannel&)> Job;
bool exec(Job job, uint32_t timeoutMs);

// Feedback from the data layer (MQTT) used to escalate recovery.
void dataFailure(const char* why);
void dataSuccess();

// Debug / commands
void forceRecovery(uint8_t level);  // 1..5
void setRatMode(int mode);          // 0 auto, 1 LTE, 2 GSM (persists in config)
bool online();
void toJson(JsonObject o);
void print(Print& out);
}  // namespace cell

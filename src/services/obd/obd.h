#pragma once
// OBD-II service (own task, core 1). Works with every vehicle that speaks
// OBD-II over CAN (ISO 15765-4): all cars sold in the US since 2008 and in the
// EU since ~2008-2010 (Toyota, Honda, BMW, Mercedes, Hyundai, ...).
// NOT supported by this hardware: pre-CAN K-line / KWP2000 / J1850 vehicles.
//
// States: NO_CHIP -> DETECT -> RUNNING <-> IGN_OFF
//  DETECT : passive listen first (never transmit at a wrong bitrate), then
//           probe 01 00 on 500/250 kbit/s, 11/29-bit
//  RUNNING: tiered PID polling, VIN, DTCs every 5 min, trip + driving events
//  IGN_OFF: listen-only; probe every obd_off_probe_s (battery friendly)
#include <Arduino.h>
#include <ArduinoJson.h>

namespace obd {
enum class State : uint8_t { Disabled, NoChip, Detect, Running, IgnOff };

void begin();                         // always (creates locks/queues)
void start();                         // starts the task (skipped in safe mode)
State state();
const char* stateName();
bool ignitionOn();
bool engineRunning();
float speedKmh(bool* valid = nullptr);

// Report builders (thread safe)
void toJson(JsonObject o);            // live data for the "obd" message
void dtcJson(JsonObject o);           // stored / pending / permanent codes
void infoJson(JsonObject o);          // VIN, protocol, supported PIDs
void healthJson(JsonObject o);        // CAN/OBD diagnostics for telemetry
bool dtcChanged();                    // new codes since last dtc report (clears flag)

// Commands (executed asynchronously by the OBD task)
void requestDtcRead();
void requestDtcClear();
void requestRescan();
void setSniff(bool on);
bool sendRaw(uint32_t id, bool ext, const uint8_t* data, uint8_t len);
void print(Print& out);
void printPids(Print& out);
}  // namespace obd

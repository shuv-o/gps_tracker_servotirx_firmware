#pragma once
// MQTT over the A76xx built-in client (AT+CMQTT*). Runs on the cellular task.
//
// Topics: <root>/<id>/<suffix>
//   status (retained, LWT "offline")  info (retained)  loc  obd  dtc  health  event  log
//   cmd  <- commands in        cmd/resp -> command results out
//
// Failover: exponential backoff with jitter (5 s .. 5 min); after 3 failed
// connects switch to the fallback broker (mqtt_host2); repeated failures are
// reported to the network layer which escalates its recovery ladder.
#include <Arduino.h>
#include <ArduinoJson.h>
#include "drivers/at_channel.h"

namespace mqtt {
void init();
void onUrc(const char* line);                       // fed by the cellular URC handler
void onRx(const char* topic, const char* payload, size_t len);
void step(AtChannel& at);                           // call when the network is online
void networkLost();                                 // network went down
bool connected();
bool publish(AtChannel& at, const char* suffix, const char* payload, size_t len, uint8_t qos, bool retain);
void disconnect(AtChannel& at);
void reconnectNow();
bool popCommand(char* buf, size_t n);               // incoming command payloads
void topic(char* out, size_t n, const char* suffix);
void toJson(JsonObject o);
void print(Print& out);
typedef void (*ConnectedHook)(AtChannel& at);
void setConnectedHook(ConnectedHook h);             // publishes status/info on connect
}  // namespace mqtt

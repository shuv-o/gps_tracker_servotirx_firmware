// =============================================================================
//  Fleet Tracker firmware - entry point
//  ESP32-S3 (N16R8) + SIMCom A7672SA-FASE (LTE Cat-1 / 2G / GNSS) + TCAN4550 (CAN)
//
//  Task layout
//    cell    core0 prio5  modem owner: network, MQTT, GNSS, OTA, delivery, remote cmds
//    obd     core1 prio6  TCAN4550 + OBD-II
//    app     core1 prio3  reporting policy, LED
//    console core0 prio2  USB serial command line
//    loop    core1 prio1  supervisor (watchdogs, safe mode, OTA confirm deadline)
//
//  See docs/ARCHITECTURE.md and docs/MQTT_PROTOCOL.md
// =============================================================================
#include <Arduino.h>
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "drivers/status_led.h"
#include "fw_version.h"
#include "services/cellular.h"
#include "services/commands.h"
#include "services/gnss.h"
#include "services/obd/obd.h"
#include "services/offline_store.h"
#include "services/reporter.h"

Counters g_cnt;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);  // don't block if no USB host

  logx::begin(32 * 1024);
  config::begin();
  logx::setLevel(cfg().log_level);
  LOGI("MAIN", "%s %s (%s %s) hw %s", FW_NAME, FW_VERSION, __DATE__, __TIME__, HW_REVISION);

  sup::begin();
  led::begin();
  offline::begin();
  bus::begin(48);
  gnss::begin();
  obd::begin();

  cell::start();
  if (!sup::safeMode()) obd::start();
  else LOGW("MAIN", "SAFE MODE: OBD and GNSS disabled; connectivity, commands and OTA only");
  report::start();
  cmd::startConsole();

  char d[320];
  snprintf(d, sizeof d, "{\"fw\":\"%s\",\"rst\":\"%s\",\"boot\":%lu,\"safe\":%s,\"ota_pending\":%s,\"crash\":\"%s\",\"stall\":\"%s\"}",
           FW_VERSION, sup::resetReason(), (unsigned long)sup::bootCount(), sup::safeMode() ? "true" : "false",
           sup::otaPendingVerify() ? "true" : "false", sup::lastCrash(), sup::lastStall());
  bus::event("boot", d);
}

void loop() {
  sup::tick();
  delay(1000);
}

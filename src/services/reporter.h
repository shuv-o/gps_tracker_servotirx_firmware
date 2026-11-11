#pragma once
// Reporting policy + JSON message builders (app task, core 1).
//
//   loc    : every rpt_move_s while moving / rpt_park_s parked, plus after
//            rpt_dist_m metres or a heading change of rpt_head_deg (corners)
//   obd    : every rpt_obd_s while the ignition is on
//   dtc    : whenever the code list changes
//   health : every rpt_health_s (device telemetry for improvement work)
//   info   : retained, on every MQTT connect
//   event  : boot, ignition, moving/stopped, net/mqtt/gnss changes, trips, ota ...
// Message formats: docs/MQTT_PROTOCOL.md
#include <Arduino.h>

namespace report {
void start();
const char* fwVersion();
void buildInfo(String& out);
bool send(const char* what);    // "loc" | "obd" | "dtc" | "health" | "info"
}  // namespace report

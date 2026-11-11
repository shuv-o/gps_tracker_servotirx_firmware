#include "services/reporter.h"
#include <ArduinoJson.h>
#include <math.h>
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "drivers/status_led.h"
#include "fw_version.h"
#include "services/cellular.h"
#include "services/gnss.h"
#include "services/mqtt_client.h"
#include "services/obd/obd.h"
#include "services/offline_store.h"
#include "services/ota.h"

namespace report {

static int s_hb = -1;

const char* fwVersion() { return FW_VERSION; }

static bool publishDoc(const char* suffix, JsonDocument& doc, bool persist = true, bool retain = false) {
  String s;
  serializeJson(doc, s);
  return bus::publish(suffix, s.c_str(), 1, retain, persist);
}

static void stamp(JsonDocument& doc) {
  doc["ts"] = clockx::now();
  doc["up"] = millis() / 1000;
}

// ------------------------------------------------------------------ builders
static void addCell(JsonObject c) {
  NetInfo n;
  cell::snapshot(n);
  char mcc[4] = "", mnc[4] = "";
  sscanf(n.mccmnc, "%3[0-9]-%3[0-9]", mcc, mnc);  // "470-02"
  if (mcc[0]) c["mcc"] = mcc;
  if (mnc[0]) c["mnc"] = mnc;
  c["tac"] = n.tac;
  c["cid"] = n.cellId;
  c["rat"] = n.rat;
  c["csq"] = n.csq;
  if (n.rsrp) c["rsrp"] = n.rsrp;
}

static bool sendLoc() {
  LocInfo l;
  gnss::snapshot(l);
  JsonDocument d;
  stamp(d);
  d["src"] = gnss::srcName(l.src);
  if (l.valid) {
    d["lat"] = serialized(String(l.lat, 6));
    d["lon"] = serialized(String(l.lon, 6));
    d["alt"] = (int)l.alt;
    d["fix_ts"] = l.fixEpoch;
    d["age"] = l.ageMs / 1000;
    if (l.src == LocSrc::Gnss) {
      d["spd"] = serialized(String(l.speedKmh, 1));
      d["crs"] = (int)l.course;
      d["hdop"] = serialized(String(l.hdop, 1));
      d["sat"] = l.sats;
    }
    if (l.accuracyM > 0) d["acc"] = (int)l.accuracyM;
  }
  bool obdOk;
  float v = obd::speedKmh(&obdOk);
  if (obdOk) d["obd_spd"] = (int)v;
  d["mv"] = l.moving || (obdOk && v >= cfg().move_kmh);
  d["ign"] = obd::ignitionOn();
  d["odo"] = serialized(String(l.odometerKm, 2));
  addCell(d["cell"].to<JsonObject>());
  return publishDoc("loc", d);
}

static bool sendObd() {
  JsonDocument d;
  stamp(d);
  obd::toJson(d.as<JsonObject>());
  return publishDoc("obd", d);
}

static bool sendDtc() {
  JsonDocument d;
  stamp(d);
  obd::dtcJson(d.as<JsonObject>());
  return publishDoc("dtc", d);
}

static bool sendHealth() {
  JsonDocument d;
  stamp(d);
  d["fw"] = FW_VERSION;
  d["boot"] = sup::bootCount();
  d["rst"] = sup::resetReason();
  d["safe"] = sup::safeMode();
  d["heap"] = ESP.getFreeHeap();
  d["heap_min"] = ESP.getMinFreeHeap();
  d["psram"] = ESP.getFreePsram();
  d["mcu_temp"] = serialized(String(temperatureRead(), 1));
  d["clk"] = clockx::sourceName();
  d["q_flash"] = offline::count();
  d["q_ram"] = bus::pending();
  d["log_e"] = logx::errorCount();
  d["log_w"] = logx::warnCount();
  cell::toJson(d["net"].to<JsonObject>());
  mqtt::toJson(d["mqtt"].to<JsonObject>());
  gnss::toJson(d["gnss"].to<JsonObject>());
  obd::healthJson(d["obd"].to<JsonObject>());
  JsonObject c = d["cnt"].to<JsonObject>();
  forEachCounter([&](const char* n, uint32_t v) { if (v) c[n] = v; });
  return publishDoc("health", d, false);
}

void buildInfo(String& out) {
  JsonDocument d;
  NetInfo n;
  cell::snapshot(n);
  d["ts"] = clockx::now();
  d["fw"] = FW_VERSION;
  d["fw_build"] = __DATE__ " " __TIME__;
  d["hw"] = HW_REVISION;
  d["name"] = cfg().dev_name;
  d["imei"] = n.imei;
  d["iccid"] = n.iccid;
  d["imsi"] = n.imsi;
  d["modem"] = n.model;
  d["modem_fw"] = n.modemFw;
  d["boot"] = sup::bootCount();
  d["rst"] = sup::resetReason();
  if (sup::lastCrash()[0]) d["crash"] = sup::lastCrash();
  if (sup::lastStall()[0]) d["stall"] = sup::lastStall();
  d["safe"] = sup::safeMode();
  d["ota_pending"] = sup::otaPendingVerify();
  obd::infoJson(d["obd"].to<JsonObject>());
  JsonObject r = d["rpt"].to<JsonObject>();
  r["move_s"] = cfg().rpt_move_s;
  r["park_s"] = cfg().rpt_park_s;
  r["obd_s"] = cfg().rpt_obd_s;
  r["health_s"] = cfg().rpt_health_s;
  serializeJson(d, out);
}

bool send(const char* what) {
  if (!strcmp(what, "loc")) return sendLoc();
  if (!strcmp(what, "obd")) return sendObd();
  if (!strcmp(what, "dtc")) return sendDtc();
  if (!strcmp(what, "health")) return sendHealth();
  if (!strcmp(what, "info")) {
    String s;
    buildInfo(s);
    return bus::publish("info", s.c_str(), 1, true, false);
  }
  return false;
}

// ------------------------------------------------------------------ policy
static double s_lastLat = 0, s_lastLon = 0;
static float s_lastCourse = -1;
static uint32_t s_lastLocMs = 0, s_lastObdMs = 0, s_lastHealthMs = 0;
static bool s_moving = false;
static uint32_t s_moveChangeMs = 0;
static bool s_prevIgn = false, s_prevFix = false;

static double distM(double a1, double o1, double a2, double o2) {
  const double R = 6371008.8, r = M_PI / 180.0;
  double x = (o2 - o1) * r * cos((a1 + a2) * r / 2), y = (a2 - a1) * r;
  return sqrt(x * x + y * y) * R;
}

static void schedule() {
  const Config& c = cfg();
  uint32_t now = millis();
  LocInfo l;
  gnss::snapshot(l);
  bool obdOk;
  float obdSpd = obd::speedKmh(&obdOk);
  bool rawMoving = l.moving || (obdOk && obdSpd >= c.move_kmh);

  // debounce moving / stopped (10 s to start, 60 s to stop)
  if (rawMoving != s_moving) {
    if (!s_moveChangeMs) s_moveChangeMs = now;
    if (now - s_moveChangeMs > (rawMoving ? 10000UL : 60000UL)) {
      s_moving = rawMoving;
      s_moveChangeMs = 0;
      bus::event(s_moving ? "moving" : "stopped", nullptr);
      sendLoc();
      s_lastLocMs = now;
    }
  } else {
    s_moveChangeMs = 0;
  }

  bool fix = l.valid && l.src == LocSrc::Gnss;
  if (fix != s_prevFix) {
    s_prevFix = fix;
    char d[48];
    snprintf(d, sizeof d, "{\"src\":\"%s\"}", gnss::srcName(l.src));
    bus::event(fix ? "gnss_fix" : "gnss_lost", d);
  }

  // location
  uint32_t interval = (s_moving ? c.rpt_move_s : c.rpt_park_s) * 1000UL;
  bool due = now - s_lastLocMs >= interval || !s_lastLocMs;
  if (!due && s_moving && fix) {
    if (c.rpt_dist_m && distM(s_lastLat, s_lastLon, l.lat, l.lon) >= c.rpt_dist_m) due = true;
    if (c.rpt_head_deg && s_lastCourse >= 0 && l.speedKmh > 15) {
      float dh = fabsf(l.course - s_lastCourse);
      if (dh > 180) dh = 360 - dh;
      if (dh >= c.rpt_head_deg) due = true;
    }
  }
  if (due && now - s_lastLocMs >= 1000) {
    sendLoc();
    s_lastLocMs = now;
    s_lastLat = l.lat;
    s_lastLon = l.lon;
    s_lastCourse = fix ? l.course : -1;
  }

  // vehicle data
  bool ign = obd::ignitionOn();
  if (ign != s_prevIgn) {
    s_prevIgn = ign;
    s_lastObdMs = now - c.rpt_obd_s * 1000UL + 5000;  // first report 5 s after ignition on
  }
  if (ign && now - s_lastObdMs >= (uint32_t)c.rpt_obd_s * 1000UL) {
    sendObd();
    s_lastObdMs = now;
  }
  if (obd::dtcChanged()) sendDtc();

  // device telemetry (first one 60 s after boot)
  if ((!s_lastHealthMs && now > 60000) || (s_lastHealthMs && now - s_lastHealthMs >= (uint32_t)c.rpt_health_s * 1000UL)) {
    sendHealth();
    s_lastHealthMs = now;
  }
}

static void updateLed() {
  if (ota::inProgress()) { led::set(led::OTA); return; }
  if (sup::safeMode()) { led::set(led::FAULT); return; }
  NetInfo n;
  cell::snapshot(n);
  LocInfo l;
  gnss::snapshot(l);
  if (n.phase != NetPhase::Online) led::set(n.phase == NetPhase::Off && n.recoveryLevel >= 5 ? led::OFFLINE : led::NET_START);
  else if (!mqtt::connected()) led::set(led::MQTT_CONNECTING);
  else led::set(l.valid && l.src == LocSrc::Gnss ? led::ONLINE_FIX : led::ONLINE_NOFIX);
}

static void task(void*) {
  s_hb = sup::registerTask("app", 30000);
  uint32_t tick = 0;
  for (;;) {
    sup::beat(s_hb);
    led::tick();
    if (++tick % 10 == 0) {
      updateLed();
      if (!sup::safeMode()) schedule();
      else if (!s_lastHealthMs || millis() - s_lastHealthMs > 300000UL) { sendHealth(); s_lastHealthMs = millis(); }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void start() { xTaskCreatePinnedToCore(task, "app", 8192, nullptr, 3, nullptr, 1); }

}  // namespace report

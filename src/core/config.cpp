#include "core/config.h"
#include <Preferences.h>
#include <stddef.h>
#include "core/log.h"

static Config s_cfg;
static const char* TAG = "CFG";

#define S(k, def, help)            {#k, CfgType::Str, offsetof(Config, k), sizeof(Config::k), def, 0, 0, 0, false, help}
#define SS(k, def, help)           {#k, CfgType::Str, offsetof(Config, k), sizeof(Config::k), def, 0, 0, 0, true, help}
#define I(k, def, mn, mx, help)    {#k, CfgType::Int, offsetof(Config, k), sizeof(int32_t), nullptr, def, mn, mx, false, help}

// clang-format off
static const CfgItem ITEMS[] = {
  // ---- Cellular ----
  S (apn,            "",            "APN (empty = network default)"),
  S (apn_user,       "",            "APN user"),
  SS(apn_pass,       "",            "APN password"),
  SS(sim_pin,        "",            "SIM PIN if locked"),
  I (rat_mode,       0, 0, 2,       "0 auto(LTE->2G fallback) 1 LTE only 2 GSM only"),
  I (reg_timeout_s,  120, 30, 600,  "network registration timeout"),
  I (lte_retry_s,    900, 120, 86400, "on 2G fallback, retry LTE every N s"),
  I (modem_baud,     115200, 9600, 921600, "modem UART baud"),
  // ---- MQTT ----
  S (mqtt_host,      "broker.hivemq.com", "primary broker host"),
  I (mqtt_port,      1883, 1, 65535, "primary broker port"),
  S (mqtt_user,      "",            "broker user"),
  SS(mqtt_pass,      "",            "broker password"),
  I (mqtt_tls,       0, 0, 1,       "1 = TLS"),
  S (mqtt_host2,     "",            "fallback broker host"),
  I (mqtt_port2,     1883, 1, 65535, "fallback broker port"),
  I (mqtt_keepalive, 60, 20, 1200,  "MQTT keepalive s"),
  S (topic_root,     "fleet",       "topic root"),
  // ---- Reporting ----
  I (rpt_move_s,     10, 1, 3600,   "location interval moving"),
  I (rpt_park_s,     300, 10, 86400, "location interval parked"),
  I (rpt_obd_s,      30, 5, 3600,   "vehicle data interval"),
  I (rpt_health_s,   300, 30, 86400, "device telemetry interval"),
  I (rpt_dist_m,     250, 0, 100000, "report every N metres (0 off)"),
  I (rpt_head_deg,   30, 0, 180,    "report on heading change (0 off)"),
  I (move_kmh,       5, 1, 50,      "moving threshold km/h"),
  // ---- GNSS ----
  I (gnss_poll_s,    2, 1, 60,      "GNSS poll interval"),
  I (gnss_mode,      0, 0, 15,      "AT+CGNSSMODE (0 = default)"),
  I (gnss_max_hdop10, 60, 10, 500,  "reject fixes with HDOP*10 above"),
  I (agnss_after_s,  90, 0, 3600,   "no fix -> AGNSS after N s (0 off)"),
  I (lbs_after_s,    180, 0, 3600,  "no fix -> cell location after N s (0 off)"),
  S (agnss_cmd,      "AT+CAGNSS",   "AGNSS command for this modem FW"),
  I (gnss_ant_active, 1, 0, 1,      "1 = active GNSS antenna (needs bias-T on board!)"),
  I (gnss_ant_mv,    3300, 1800, 3300, "active antenna supply mV"),
  // ---- OBD ----
  I (obd_enable,     1, 0, 1,       "OBD-II polling"),
  I (obd_bitrate,    0, 0, 1000,    "0 auto, 250, 500 kbit/s"),
  I (obd_idmode,     0, 0, 2,       "0 auto 1 11-bit 2 29-bit"),
  I (obd_off_probe_s, 60, 10, 3600, "ignition off probe interval"),
  I (obd_sniff,      0, 0, 1,       "log every CAN frame"),
  S (obd_custom,     "",            "custom requests tx:hexreq:every_s;..."),
  // ---- System ----
  S (dev_name,       "",            "friendly device name"),
  I (log_level,      3, 0, 5,       "0 off 1 err 2 warn 3 info 4 debug 5 verbose"),
  I (ota_confirm_s,  300, 60, 3600, "OTA health-confirm window"),
  I (safe_resets,    4, 2, 20,      "abnormal resets in 10 min -> safe mode"),
};
// clang-format on

namespace config {

const Config& get() { return s_cfg; }
Config& mut() { return s_cfg; }

const CfgItem* items(size_t& count) {
  count = sizeof(ITEMS) / sizeof(ITEMS[0]);
  return ITEMS;
}

const CfgItem* find(const char* key) {
  for (auto& it : ITEMS)
    if (!strcmp(it.key, key)) return &it;
  return nullptr;
}

static void applyDefaults() {
  memset(&s_cfg, 0, sizeof s_cfg);
  for (auto& it : ITEMS) {
    uint8_t* p = (uint8_t*)&s_cfg + it.offset;
    if (it.type == CfgType::Str) strlcpy((char*)p, it.defStr, it.size);
    else *(int32_t*)p = it.defInt;
  }
}

void begin() {
  applyDefaults();
  Preferences nv;
  if (!nv.begin("cfg", true)) { LOGW(TAG, "no stored config, using defaults"); return; }
  int loaded = 0;
  for (auto& it : ITEMS) {
    if (!nv.isKey(it.key)) continue;
    uint8_t* p = (uint8_t*)&s_cfg + it.offset;
    if (it.type == CfgType::Str) nv.getString(it.key, (char*)p, it.size);
    else *(int32_t*)p = constrain(nv.getInt(it.key, it.defInt), it.minInt, it.maxInt);
    loaded++;
  }
  nv.end();
  LOGI(TAG, "loaded %d stored settings", loaded);
}

void save() {
  Preferences nv;
  nv.begin("cfg", false);
  for (auto& it : ITEMS) {
    uint8_t* p = (uint8_t*)&s_cfg + it.offset;
    if (it.type == CfgType::Str) {
      if (strcmp((char*)p, it.defStr)) nv.putString(it.key, (char*)p);
      else if (nv.isKey(it.key)) nv.remove(it.key);
    } else {
      if (*(int32_t*)p != it.defInt) nv.putInt(it.key, *(int32_t*)p);
      else if (nv.isKey(it.key)) nv.remove(it.key);
    }
  }
  nv.end();
}

void factoryReset() {
  Preferences nv;
  nv.begin("cfg", false);
  nv.clear();
  nv.end();
  applyDefaults();
}

bool set(const char* key, const char* value, String& err) {
  const CfgItem* it = find(key);
  if (!it) { err = String("unknown key '") + key + "'"; return false; }
  uint8_t* p = (uint8_t*)&s_cfg + it->offset;
  if (it->type == CfgType::Str) {
    if (strlen(value) >= it->size) { err = "too long (max " + String(it->size - 1) + ")"; return false; }
    strlcpy((char*)p, value, it->size);
  } else {
    char* end;
    long v = strtol(value, &end, 10);
    if (*value == 0 || *end != 0) { err = "not a number"; return false; }
    if (v < it->minInt || v > it->maxInt) {
      err = "out of range " + String(it->minInt) + ".." + String(it->maxInt);
      return false;
    }
    *(int32_t*)p = v;
  }
  save();
  if (!strcmp(key, "log_level")) logx::setLevel(s_cfg.log_level);
  LOGI(TAG, "%s updated", key);
  return true;
}

bool valueStr(const CfgItem& it, char* out, size_t n, bool reveal) {
  const uint8_t* p = (const uint8_t*)&s_cfg + it.offset;
  if (it.type == CfgType::Str) {
    if (it.secret && !reveal && *(const char*)p) strlcpy(out, "***", n);
    else strlcpy(out, (const char*)p, n);
  } else {
    snprintf(out, n, "%ld", (long)*(const int32_t*)p);
  }
  return true;
}

}  // namespace config

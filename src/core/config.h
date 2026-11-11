#pragma once
// Runtime configuration, persisted in NVS (namespace "cfg").
//
// Every setting is described ONCE in the table in config.cpp
// (key, type, default, min, max, help). That single table drives:
//   - defaults, NVS load/save
//   - "config get/set/list" on the serial console and over MQTT
// To add a setting: add a field here + one line in the table.
#include <Arduino.h>

struct Config {
  // ---- Cellular ----
  char    apn[64];
  char    apn_user[32];
  char    apn_pass[32];
  char    sim_pin[9];
  int32_t rat_mode;          // 0 auto (LTE preferred, 2G fallback) | 1 LTE only | 2 GSM only
  int32_t reg_timeout_s;     // registration timeout before escalating recovery
  int32_t lte_retry_s;       // while on 2G fallback, try LTE again every N s
  int32_t modem_baud;        // UART baud to the modem

  // ---- MQTT ----
  char    mqtt_host[96];
  int32_t mqtt_port;
  char    mqtt_user[64];
  char    mqtt_pass[64];
  int32_t mqtt_tls;          // 0 tcp | 1 TLS (no cert check)
  char    mqtt_host2[96];    // fallback broker ("" = none)
  int32_t mqtt_port2;
  int32_t mqtt_keepalive;
  char    topic_root[32];    // topics: <root>/<device id>/...

  // ---- Reporting ----
  int32_t rpt_move_s;        // location interval while moving
  int32_t rpt_park_s;        // location interval while parked
  int32_t rpt_obd_s;         // vehicle data interval (ignition on)
  int32_t rpt_health_s;      // device telemetry interval
  int32_t rpt_dist_m;        // extra location report after N metres
  int32_t rpt_head_deg;      // extra location report on heading change (corners)
  int32_t move_kmh;          // speed above which the vehicle is "moving"

  // ---- GNSS ----
  int32_t gnss_poll_s;
  int32_t gnss_mode;         // AT+CGNSSMODE value, 0 = leave default
  int32_t gnss_max_hdop10;   // fixes with HDOP*10 above this are rejected
  int32_t agnss_after_s;     // no fix for N s -> try AGNSS download
  int32_t lbs_after_s;       // no fix for N s -> cell-tower location
  char    agnss_cmd[24];     // modem AGNSS command (firmware dependent)
  int32_t gnss_ant_active;   // 0 passive antenna (no DC) | 1 active antenna (VAUX bias on)
  int32_t gnss_ant_mv;       // active antenna supply voltage (mV)

  // ---- OBD-II / CAN ----
  int32_t obd_enable;
  int32_t obd_bitrate;       // 0 auto | 250 | 500 (kbit/s)
  int32_t obd_idmode;        // 0 auto | 1 11-bit | 2 29-bit
  int32_t obd_off_probe_s;   // ignition off: probe ECU every N s (keep low to protect battery)
  int32_t obd_sniff;         // 1 = log all CAN frames (debug)
  char    obd_custom[200];   // "7E0:221234:30;7E0:22F190:600" tx:request:every_s (raw hex reported)

  // ---- System ----
  char    dev_name[32];
  int32_t log_level;         // 0..5
  int32_t ota_confirm_s;     // new firmware must reach MQTT within N s or it rolls back
  int32_t safe_resets;       // abnormal resets in 10 min that trigger safe mode
};

enum class CfgType : uint8_t { Str, Int };

struct CfgItem {
  const char* key;           // <= 15 chars (NVS limit)
  CfgType     type;
  size_t      offset;
  size_t      size;          // string buffer size
  const char* defStr;
  int32_t     defInt, minInt, maxInt;
  bool        secret;        // value hidden in listings
  const char* help;
};

namespace config {
void begin();                               // load defaults + NVS
const Config& get();                        // read-only access
Config& mut();                              // direct access (call save() after)
const CfgItem* items(size_t& count);
const CfgItem* find(const char* key);
bool set(const char* key, const char* value, String& err);   // validates + saves
bool valueStr(const CfgItem& it, char* out, size_t n, bool reveal = false);
void save();
void factoryReset();
}  // namespace config

inline const Config& cfg() { return config::get(); }

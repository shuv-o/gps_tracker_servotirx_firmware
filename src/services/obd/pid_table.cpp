#include "services/obd/pid_table.h"

#define A d[0]
#define B d[1]
#define C d[2]
#define D d[3]
#define AB ((A << 8) | B)
#define PID_FN [](const uint8_t* d) -> float

// clang-format off
static const PidDef TABLE[] = {
  // pid   key                 unit    len tier decoder
  {0x04, "load",              "%",     1, 0, PID_FN { return A * 100.0f / 255; }},
  {0x05, "coolant",           "C",     1, 2, PID_FN { return A - 40.0f; }},
  {0x06, "stft1",             "%",     1, 1, PID_FN { return (A - 128) * 100.0f / 128; }},
  {0x07, "ltft1",             "%",     1, 2, PID_FN { return (A - 128) * 100.0f / 128; }},
  {0x08, "stft2",             "%",     1, 1, PID_FN { return (A - 128) * 100.0f / 128; }},
  {0x09, "ltft2",             "%",     1, 2, PID_FN { return (A - 128) * 100.0f / 128; }},
  {0x0A, "fuel_press",        "kPa",   1, 2, PID_FN { return A * 3.0f; }},
  {0x0B, "map",               "kPa",   1, 1, PID_FN { return A; }},
  {0x0C, "rpm",               "rpm",   2, 0, PID_FN { return AB / 4.0f; }},
  {0x0D, "speed",             "km/h",  1, 0, PID_FN { return A; }},
  {0x0E, "timing_adv",        "deg",   1, 1, PID_FN { return A / 2.0f - 64; }},
  {0x0F, "iat",               "C",     1, 2, PID_FN { return A - 40.0f; }},
  {0x10, "maf",               "g/s",   2, 0, PID_FN { return AB / 100.0f; }},
  {0x11, "throttle",          "%",     1, 0, PID_FN { return A * 100.0f / 255; }},
  {0x14, "o2_b1s1",           "V",     2, 2, PID_FN { return A / 200.0f; }},
  {0x15, "o2_b1s2",           "V",     2, 2, PID_FN { return A / 200.0f; }},
  {0x1F, "run_time",          "s",     2, 2, PID_FN { return AB; }},
  {0x21, "dist_mil",          "km",    2, 3, PID_FN { return AB; }},
  {0x22, "rail_press_rel",    "kPa",   2, 2, PID_FN { return AB * 0.079f; }},
  {0x23, "rail_press",        "kPa",   2, 2, PID_FN { return AB * 10.0f; }},
  {0x2C, "egr_cmd",           "%",     1, 2, PID_FN { return A * 100.0f / 255; }},
  {0x2D, "egr_err",           "%",     1, 2, PID_FN { return (A - 128) * 100.0f / 128; }},
  {0x2E, "evap_purge",        "%",     1, 2, PID_FN { return A * 100.0f / 255; }},
  {0x2F, "fuel_level",        "%",     1, 2, PID_FN { return A * 100.0f / 255; }},
  {0x30, "warmups",           "",      1, 3, PID_FN { return A; }},
  {0x31, "dist_clr",          "km",    2, 3, PID_FN { return AB; }},
  {0x33, "baro",              "kPa",   1, 3, PID_FN { return A; }},
  {0x3C, "cat_temp_b1s1",     "C",     2, 2, PID_FN { return AB / 10.0f - 40; }},
  {0x3D, "cat_temp_b2s1",     "C",     2, 2, PID_FN { return AB / 10.0f - 40; }},
  {0x42, "ecu_volt",          "V",     2, 1, PID_FN { return AB / 1000.0f; }},
  {0x43, "abs_load",          "%",     2, 1, PID_FN { return AB * 100.0f / 255; }},
  {0x44, "lambda_cmd",        "",      2, 1, PID_FN { return AB * 2.0f / 65536; }},
  {0x45, "rel_throttle",      "%",     1, 1, PID_FN { return A * 100.0f / 255; }},
  {0x46, "ambient",           "C",     1, 3, PID_FN { return A - 40.0f; }},
  {0x47, "abs_throttle_b",    "%",     1, 1, PID_FN { return A * 100.0f / 255; }},
  {0x49, "pedal_d",           "%",     1, 0, PID_FN { return A * 100.0f / 255; }},
  {0x4A, "pedal_e",           "%",     1, 1, PID_FN { return A * 100.0f / 255; }},
  {0x4C, "throttle_cmd",      "%",     1, 1, PID_FN { return A * 100.0f / 255; }},
  {0x4D, "time_mil",          "min",   2, 3, PID_FN { return AB; }},
  {0x4E, "time_clr",          "min",   2, 3, PID_FN { return AB; }},
  {0x51, "fuel_type",         "",      1, 3, PID_FN { return A; }},
  {0x52, "ethanol",           "%",     1, 3, PID_FN { return A * 100.0f / 255; }},
  {0x59, "rail_press_abs",    "kPa",   2, 2, PID_FN { return AB * 10.0f; }},
  {0x5A, "rel_pedal",         "%",     1, 1, PID_FN { return A * 100.0f / 255; }},
  {0x5B, "hybrid_batt",       "%",     1, 2, PID_FN { return A * 100.0f / 255; }},
  {0x5C, "oil_temp",          "C",     1, 2, PID_FN { return A - 40.0f; }},
  {0x5D, "inj_timing",        "deg",   2, 2, PID_FN { return AB / 128.0f - 210; }},
  {0x5E, "fuel_rate",         "L/h",   2, 0, PID_FN { return AB / 20.0f; }},
  {0x61, "torque_demand",     "%",     1, 1, PID_FN { return A - 125.0f; }},
  {0x62, "torque_actual",     "%",     1, 0, PID_FN { return A - 125.0f; }},
  {0x63, "torque_ref",        "Nm",    2, 3, PID_FN { return AB; }},
  {0xA6, "odometer",          "km",    4, 3, PID_FN { return (((uint32_t)A << 24) | ((uint32_t)B << 16) | (C << 8) | D) / 10.0f; }},
};
// clang-format on

namespace pids {
const uint32_t TIER_MS[4] = {1000, 5000, 30000, 300000};

const PidDef* table(size_t& count) {
  count = sizeof(TABLE) / sizeof(TABLE[0]);
  return TABLE;
}

const PidDef* find(uint8_t pid) {
  for (auto& p : TABLE)
    if (p.pid == pid) return &p;
  return nullptr;
}
}  // namespace pids

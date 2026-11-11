#include "services/obd/obd.h"
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/deadline.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "drivers/tcan4550.h"
#include "services/obd/isotp.h"
#include "services/obd/pid_table.h"

namespace obd {

static const char* TAG = "OBD";
static Tcan4550 s_can;
static SemaphoreHandle_t s_mtx;
static int s_hb = -1;
static volatile State s_state = State::NoChip;

// protocol
static bool s_ext = false;
static uint16_t s_kbps = 0;
static uint32_t s_ecus[8];
static uint8_t s_nEcus = 0;
static uint32_t s_primaryRx = 0;   // usually 0x7E8 (engine)

// data
struct Val { float v; uint32_t ms; };
static Val s_vals[256];
static uint8_t s_supported[32];    // mode 01 bitmap
static uint32_t s_tierNext[4];
static char s_vin[18] = "";
static uint8_t s_timeoutStreak = 0;
static uint32_t s_lastOkMs = 0;

// DTC
static const int MAX_DTC = 16;
static char s_dtc[3][MAX_DTC][6];   // stored, pending, permanent
static uint8_t s_nDtc[3];
static bool s_mil = false;
static uint8_t s_milCount = 0;
static uint32_t s_nextDtcMs = 0;
static volatile bool s_dtcChanged = false;

// trip
struct Trip {
  uint32_t startEpoch, startMs;
  float km, maxKmh, idleS, fuelL;
  uint16_t harshAccel, harshBrake, overRev;
} s_trip{};
static float s_prevSpeed = -1;
static uint32_t s_prevSpeedMs = 0, s_lastHarshMs = 0;

// custom requests
struct Custom { uint32_t tx; uint8_t req[7]; uint8_t len; uint32_t everyMs, next; char key[20]; char last[40]; };
static Custom s_custom[6];
static uint8_t s_nCustom = 0;
static char s_customCfg[200] = "\x01";

// requests from other tasks
static volatile bool r_dtcRead = false, r_dtcClear = false, r_rescan = false, s_sniff = false;
struct RawTx { uint32_t id; bool ext; uint8_t len; uint8_t d[8]; };
static QueueHandle_t s_rawQ;

// diagnostics
static uint32_t s_detectAttempts = 0, s_busOffs = 0;
static char s_lastErr[48] = "";

struct Lock {
  Lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(s_mtx); }
};

static bool supported(uint8_t pid) { return s_supported[pid >> 3] & (0x80 >> (pid & 7)); }
static void setSupported(uint8_t pid, bool on) {
  if (on) s_supported[pid >> 3] |= (0x80 >> (pid & 7));
  else s_supported[pid >> 3] &= ~(0x80 >> (pid & 7));
}

static const char* nameOf(State st) {
  switch (st) {
    case State::Disabled: return "disabled";
    case State::NoChip:   return "no_can_chip";
    case State::Detect:   return "detecting";
    case State::Running:  return "running";
    case State::IgnOff:   return "ignition_off";
  }
  return "?";
}
const char* stateName() { return nameOf(s_state); }
State state() { return s_state; }
bool ignitionOn() { return s_state == State::Running; }

static bool fresh(uint8_t pid, uint32_t maxAgeMs) {
  return s_vals[pid].ms && millis() - s_vals[pid].ms < maxAgeMs;
}
bool engineRunning() { return ignitionOn() && fresh(0x0C, 5000) && s_vals[0x0C].v > 300; }
float speedKmh(bool* valid) {
  bool ok = ignitionOn() && fresh(0x0D, 5000);
  if (valid) *valid = ok;
  return ok ? s_vals[0x0D].v : 0;
}

static void sniffHook(const CanFrame& f) {
  if (!s_sniff) return;
  LOGI("CANRX", "%s %08lX [%u] %02X %02X %02X %02X %02X %02X %02X %02X", f.ext ? "X" : "S", (unsigned long)f.id, f.dlc,
       f.data[0], f.data[1], f.data[2], f.data[3], f.data[4], f.data[5], f.data[6], f.data[7]);
}

static void setState(State s) {
  if (s != s_state) LOGI(TAG, "%s -> %s", nameOf(s_state), nameOf(s));
  s_state = s;
}

// ------------------------------------------------------------------ requests
static int req(uint32_t txId, const uint8_t* r, uint8_t n, uint8_t* resp, size_t cap, uint32_t rxFilter,
               uint32_t timeoutMs = 150, uint32_t* from = nullptr) {
  CNT_INC(obd_requests);
  uint8_t nrc = 0;
  int k = isotp::request(s_can, s_ext, txId, r, n, resp, cap, rxFilter, timeoutMs, from, &nrc);
  if (k == isotp::TIMEOUT) CNT_INC(obd_timeouts);
  else if (k == isotp::NEGATIVE) CNT_INC(obd_neg_resp);
  return k;
}

static uint32_t primaryTx() { return isotp::requestIdFor(s_ext, s_primaryRx); }

static bool readMode01(uint8_t pid, uint8_t* d, uint8_t need) {
  uint8_t r[2] = {0x01, pid}, resp[16];
  int k = req(primaryTx(), r, 2, resp, sizeof resp, s_primaryRx);
  if (k == isotp::NEGATIVE) { setSupported(pid, false); return false; }
  if (k < 2 + need) {
    if (k == isotp::TIMEOUT) s_timeoutStreak++;
    return false;
  }
  s_timeoutStreak = 0;
  s_lastOkMs = millis();
  memcpy(d, resp + 2, need);
  return true;
}

static void discoverPids() {
  memset(s_supported, 0, sizeof s_supported);
  for (uint8_t base = 0x00;; base += 0x20) {
    uint8_t d[4];
    if (!readMode01(base, d, 4)) break;
    for (int i = 0; i < 32; i++)
      if (d[i / 8] & (0x80 >> (i % 8))) setSupported(base + 1 + i, true);
    if (!(d[3] & 0x01) || base == 0xE0) break;  // next range not supported
  }
  int n = 0;
  size_t tn;
  const PidDef* t = pids::table(tn);
  for (size_t i = 0; i < tn; i++) n += supported(t[i].pid);
  LOGI(TAG, "vehicle supports %d of %u decoded PIDs", n, (unsigned)tn);
}

static void readVin() {
  uint8_t r[2] = {0x09, 0x02}, resp[40];
  int k = req(primaryTx(), r, 2, resp, sizeof resp, s_primaryRx, 1500);
  if (k < 19) return;
  // 49 02 01 <17 chars>  (some ECUs pad with 00 before the VIN)
  char vin[18];
  int j = 0;
  for (int i = 3; i < k && j < 17; i++)
    if (resp[i] >= 0x20 && resp[i] < 0x7F) vin[j++] = resp[i];
  vin[j] = 0;
  if (j == 17) {
    Lock l;
    if (strcmp(vin, s_vin)) LOGI(TAG, "VIN %s", vin);
    strlcpy(s_vin, vin, sizeof s_vin);
  }
}

static void dtcStr(uint8_t a, uint8_t b, char* out) {
  static const char T[] = "PCBU";
  static const char H[] = "0123456789ABCDEF";
  out[0] = T[a >> 6];
  out[1] = H[(a >> 4) & 3];
  out[2] = H[a & 0x0F];
  out[3] = H[b >> 4];
  out[4] = H[b & 0x0F];
  out[5] = 0;
}

static void readDtcs() {
  static const uint8_t MODES[3] = {0x03, 0x07, 0x0A};
  char found[3][MAX_DTC][6];
  uint8_t cnt[3] = {0, 0, 0};
  for (int m = 0; m < 3; m++) {
    for (uint8_t e = 0; e < s_nEcus; e++) {          // every ECU (engine, transmission, ...)
      uint8_t r[1] = {MODES[m]}, resp[64];
      int k = req(isotp::requestIdFor(s_ext, s_ecus[e]), r, 1, resp, sizeof resp, s_ecus[e], 1000);
      if (k < 2) continue;
      for (int i = 2; i + 1 < k && cnt[m] < MAX_DTC; i += 2) {  // 4x NN a b a b ...
        if (!resp[i] && !resp[i + 1]) continue;
        char c[6];
        dtcStr(resp[i], resp[i + 1], c);
        bool dup = false;
        for (int x = 0; x < cnt[m]; x++) dup |= !strcmp(found[m][x], c);
        if (!dup) strcpy(found[m][cnt[m]++], c);
      }
    }
  }
  uint8_t d[4];
  bool mil = false;
  uint8_t milCount = 0;
  if (supported(0x01) && readMode01(0x01, d, 4)) { mil = d[0] & 0x80; milCount = d[0] & 0x7F; }
  Lock l;
  bool changed = mil != s_mil || memcmp(cnt, s_nDtc, 3) || memcmp(found, s_dtc, sizeof found);
  memcpy(s_dtc, found, sizeof found);
  memcpy(s_nDtc, cnt, 3);
  s_mil = mil;
  s_milCount = milCount;
  if (changed) {
    s_dtcChanged = true;
    LOGI(TAG, "DTC stored %u pending %u permanent %u MIL %s", cnt[0], cnt[1], cnt[2], mil ? "ON" : "off");
  }
}

static void clearDtcs() {
  uint8_t r[1] = {0x04}, resp[8];
  int k = req(isotp::functionalId(s_ext), r, 1, resp, sizeof resp, 0, 2000);
  LOGW(TAG, "clear DTC (mode 04) -> %s", k > 0 ? "ok" : "failed (engine must be off, ignition on)");
  bus::event("dtc_clear", k > 0 ? "{\"ok\":true}" : "{\"ok\":false}");
  s_nextDtcMs = after(2000);
}

static void parseCustom() {
  if (!strcmp(s_customCfg, cfg().obd_custom)) return;
  strlcpy(s_customCfg, cfg().obd_custom, sizeof s_customCfg);
  s_nCustom = 0;
  char buf[200];
  strlcpy(buf, s_customCfg, sizeof buf);
  for (char* item = strtok(buf, ";"); item && s_nCustom < 6; item = strtok(nullptr, ";")) {
    unsigned long tx = 0, every = 0;
    char hex[16] = "";
    if (sscanf(item, "%lx:%15[0-9A-Fa-f]:%lu", &tx, hex, &every) != 3 || strlen(hex) < 2 || strlen(hex) > 14) {
      LOGW(TAG, "bad custom request '%s'", item);
      continue;
    }
    Custom& c = s_custom[s_nCustom];
    c.tx = tx;
    c.len = strlen(hex) / 2;
    for (int i = 0; i < c.len; i++) { char b[3] = {hex[2 * i], hex[2 * i + 1], 0}; c.req[i] = strtoul(b, nullptr, 16); }
    c.everyMs = max<unsigned long>(every, 1) * 1000UL;
    c.next = 0;
    snprintf(c.key, sizeof c.key, "%lX:%s", tx, hex);
    c.last[0] = 0;
    s_nCustom++;
  }
  LOGI(TAG, "%u custom requests configured", s_nCustom);
}

static void runCustom() {
  for (uint8_t i = 0; i < s_nCustom; i++) {
    Custom& c = s_custom[i];
    if (!due(c.next)) continue;
    c.next = after(c.everyMs);
    uint8_t resp[64];
    bool ext = c.tx > 0x7FF;
    uint32_t rx = ext ? (0x18DAF100 | ((c.tx >> 8) & 0xFF)) : c.tx + 8;
    int k = isotp::request(s_can, ext, c.tx, c.req, c.len, resp, sizeof resp, rx, 500);
    Lock l;
    if (k > 0) {
      int j = 0;
      for (int b = 0; b < k && j < (int)sizeof(c.last) - 3; b++) j += snprintf(c.last + j, sizeof(c.last) - j, "%02X", resp[b]);
    } else {
      snprintf(c.last, sizeof c.last, "ERR%d", k);
    }
  }
}

// ------------------------------------------------------------------ trip / events
static void tripStart() {
  memset(&s_trip, 0, sizeof s_trip);
  s_trip.startEpoch = clockx::now();
  s_trip.startMs = millis();
  s_prevSpeed = -1;
}

static void tripEnd() {
  char d[256];
  snprintf(d, sizeof d,
           "{\"start\":%lu,\"dur_s\":%lu,\"km\":%.2f,\"max_kmh\":%.0f,\"idle_s\":%.0f,\"fuel_l\":%.2f,"
           "\"harsh_accel\":%u,\"harsh_brake\":%u,\"over_rev\":%u}",
           (unsigned long)s_trip.startEpoch, (unsigned long)((millis() - s_trip.startMs) / 1000), s_trip.km,
           s_trip.maxKmh, s_trip.idleS, s_trip.fuelL, s_trip.harshAccel, s_trip.harshBrake, s_trip.overRev);
  bus::event("trip_end", d);
}

static void updateTrip() {
  if (!fresh(0x0D, 3000)) return;
  float v = s_vals[0x0D].v;
  uint32_t now = s_vals[0x0D].ms;
  if (s_prevSpeed >= 0 && now != s_prevSpeedMs) {
    float dt = (now - s_prevSpeedMs) / 1000.0f;
    if (dt > 0 && dt < 5) {
      s_trip.km += (v + s_prevSpeed) / 2 * dt / 3600.0f;
      if (engineRunning() && v < 1) s_trip.idleS += dt;
      float fuel = fresh(0x5E, 3000) ? s_vals[0x5E].v
                   : fresh(0x10, 3000) ? s_vals[0x10].v * 3600.0f / (14.7f * 745.0f) : 0;  // L/h
      s_trip.fuelL += fuel * dt / 3600.0f;
      float acc = (v - s_prevSpeed) / dt;  // km/h per s
      if (millis() - s_lastHarshMs > 5000) {
        char d[64];
        if (acc > 12) { s_trip.harshAccel++; s_lastHarshMs = millis(); snprintf(d, sizeof d, "{\"kmh_s\":%.1f,\"kmh\":%.0f}", acc, v); bus::event("harsh_accel", d); }
        if (acc < -14) { s_trip.harshBrake++; s_lastHarshMs = millis(); snprintf(d, sizeof d, "{\"kmh_s\":%.1f,\"kmh\":%.0f}", acc, v); bus::event("harsh_brake", d); }
      }
    }
  }
  if (v > s_trip.maxKmh) s_trip.maxKmh = v;
  if (fresh(0x0C, 3000) && s_vals[0x0C].v > 4500 && millis() - s_lastHarshMs > 5000) {
    s_trip.overRev++;
    s_lastHarshMs = millis();
  }
  s_prevSpeed = v;
  s_prevSpeedMs = now;
}

// ------------------------------------------------------------------ detection
// Listen without transmitting: frames received without errors => bitrate is right.
static int passiveListen(uint16_t kbps, uint32_t ms, bool* errors) {
  s_can.configure(kbps, true, true, false);
  uint32_t t0 = millis();
  int n = 0;
  CanFrame f;
  while (millis() - t0 < ms) {
    while (s_can.receive(f)) { n++; sniffHook(f); }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  CanStatus st = s_can.status();
  *errors = st.rec > 0 || (st.lec != 0 && st.lec != 7);
  return n;
}

// Send 01 00 and collect every ECU that answers.
static bool probe(uint16_t kbps, bool ext) {
  s_can.configure(kbps, false, false, true);  // OBD filters, single-shot transmit
  uint8_t sf[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};
  CanFrame f{};
  f.id = isotp::functionalId(ext);
  f.ext = ext;
  f.dlc = 8;
  memcpy(f.data, sf, 8);
  CanFrame rx;
  while (s_can.receive(rx)) {}
  if (!s_can.send(f)) return false;
  uint8_t n = 0;
  uint32_t ecus[8];
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    if (!s_can.receive(rx)) { vTaskDelay(1); continue; }
    if (rx.ext == ext && isotp::isResponseId(ext, rx.id) && rx.data[1] == 0x41 && rx.data[2] == 0x00) {
      bool dup = false;
      for (int i = 0; i < n; i++) dup |= ecus[i] == rx.id;
      if (!dup && n < 8) ecus[n++] = rx.id;
    }
  }
  CanStatus st = s_can.status();
  if (st.busOff) s_can.recoverBusOff();
  if (!n) return false;
  // sort so the engine (lowest id) is primary
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      if (ecus[j] < ecus[i]) { uint32_t t = ecus[i]; ecus[i] = ecus[j]; ecus[j] = t; }
  Lock l;
  memcpy(s_ecus, ecus, sizeof ecus);
  s_nEcus = n;
  s_primaryRx = ecus[0];
  s_ext = ext;
  s_kbps = kbps;
  return true;
}

static bool detect(bool quick) {
  s_detectAttempts++;
  const Config& c = cfg();
  uint16_t rates[2] = {500, 250};
  int nRates = 2;
  if (c.obd_bitrate) { rates[0] = c.obd_bitrate; nRates = 1; }
  else if (quick && s_kbps) { rates[0] = s_kbps; nRates = 1; }  // known vehicle: try its bitrate only
  bool modes[2] = {false, true};
  int nModes = 2;
  if (c.obd_idmode == 1) { nModes = 1; }
  else if (c.obd_idmode == 2) { modes[0] = true; nModes = 1; }
  else if (quick && s_kbps) { modes[0] = s_ext; nModes = 1; }

  for (int i = 0; i < nRates; i++) {
    bool errors = false;
    int frames = passiveListen(rates[i], 250, &errors);
    if (!frames && errors) {
      LOGD(TAG, "%u kbit/s: bus errors while listening -> wrong bitrate, not transmitting", rates[i]);
      continue;
    }
    for (int m = 0; m < nModes; m++) {
      sup::feed();
      if (probe(rates[i], modes[m])) {
        s_can.configure(rates[i], false, s_sniff, false);
        LOGI(TAG, "vehicle found: CAN %u kbit/s %s-bit, %u ECU(s), primary 0x%lX", rates[i], modes[m] ? "29" : "11",
             s_nEcus, (unsigned long)s_primaryRx);
        return true;
      }
    }
  }
  return false;
}

static void onIgnitionOn() {
  s_timeoutStreak = 0;
  s_lastOkMs = millis();
  for (auto& t : s_tierNext) t = 0;
  discoverPids();
  readVin();
  s_nextDtcMs = after(3000);
  tripStart();
  char d[96];
  snprintf(d, sizeof d, "{\"proto\":\"CAN%s/%u\",\"ecus\":%u,\"vin\":\"%s\"}", s_ext ? "29" : "11", s_kbps, s_nEcus, s_vin);
  bus::event("ignition_on", d);
  setState(State::Running);
}

static void onIgnitionOff() {
  tripEnd();
  bus::event("ignition_off", nullptr);
  {
    Lock l;
    for (auto& v : s_vals) v.ms = 0;  // stale
  }
  setState(State::IgnOff);
  s_can.configure(s_kbps ? s_kbps : 500, true, true, false);  // listen only
}

// ------------------------------------------------------------------ task
static void pollTiers() {
  size_t tn;
  const PidDef* t = pids::table(tn);
  for (int tier = 0; tier < 4; tier++) {
    if (!due(s_tierNext[tier])) continue;
    s_tierNext[tier] = after(pids::TIER_MS[tier]);
    for (size_t i = 0; i < tn; i++) {
      if (t[i].tier != tier || !supported(t[i].pid)) continue;
      uint8_t d[4];
      if (readMode01(t[i].pid, d, t[i].len)) {
        float v = t[i].decode(d);
        Lock l;
        s_vals[t[i].pid] = {v, millis()};
      }
      sup::feed();
      if (s_timeoutStreak >= 6) return;
    }
  }
}

static void checkBus() {
  static uint32_t next = 0, busOffWindow = 0;
  static uint8_t busOffCount = 0;
  if (!due(next)) return;
  next = after(1000);
  CanStatus st = s_can.status();
  if (st.busOff) {
    s_busOffs++;
    s_can.recoverBusOff();
    if (millis() - busOffWindow > 60000) { busOffWindow = millis(); busOffCount = 0; }
    if (++busOffCount >= 3) {
      LOGE(TAG, "repeated bus-off -> re-detect");
      strlcpy(s_lastErr, "repeated bus-off", sizeof s_lastErr);
      CNT_INC(obd_reinit);
      setState(State::Detect);
    }
  }
  static uint8_t spiCheck = 0;
  if (++spiCheck >= 10) {
    spiCheck = 0;
    if (!s_can.alive()) {
      LOGE(TAG, "TCAN4550 stopped answering SPI -> reinit");
      strlcpy(s_lastErr, "SPI lost", sizeof s_lastErr);
      CNT_INC(obd_reinit);
      setState(State::NoChip);
    }
  }
}

static void task(void*) {
  s_hb = sup::registerTask("obd", 60000);
  isotp::setHook(sniffHook);
  uint32_t nextProbe = 0, nextChipTry = 0;
  for (;;) {
    sup::beat(s_hb);
    if (!cfg().obd_enable) {
      s_state = State::Disabled;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    RawTx rt;
    while (xQueueReceive(s_rawQ, &rt, 0) == pdTRUE) {
      CanFrame f{};
      f.id = rt.id; f.ext = rt.ext; f.dlc = rt.len;
      memcpy(f.data, rt.d, rt.len);
      LOGI(TAG, "raw tx %lX -> %s", (unsigned long)rt.id, s_can.send(f) ? "ok" : "fail");
    }

    switch (s_state) {
      case State::Disabled:
        setState(State::NoChip);
        break;

      case State::NoChip:
        if (!due(nextChipTry)) break;
        nextChipTry = after(30000);
        sup::setNote(s_hb, "chip init");
        if (s_can.begin()) setState(State::Detect);
        else strlcpy(s_lastErr, "TCAN4550 not responding", sizeof s_lastErr);
        break;

      case State::Detect:
        sup::setNote(s_hb, "detect");
        if (detect(false)) onIgnitionOn();
        else {
          LOGI(TAG, "no ECU answered (ignition off or non-CAN vehicle)");
          setState(State::IgnOff);
          s_can.configure(s_kbps ? s_kbps : 500, true, true, false);
          nextProbe = after(cfg().obd_off_probe_s * 1000UL);
        }
        break;

      case State::IgnOff: {
        sup::setNote(s_hb, "ign off");
        // wake on bus activity (listen-only) or periodic probe
        CanFrame f;
        int seen = 0;
        while (s_can.receive(f)) { seen++; sniffHook(f); }
        static uint32_t lastActivityProbe = 0;
        bool activity = seen > 3 && millis() - lastActivityProbe > 20000;
        if (r_rescan || activity || due(nextProbe)) {
          if (activity) lastActivityProbe = millis();
          r_rescan = false;
          nextProbe = after(cfg().obd_off_probe_s * 1000UL);
          if (detect(true) || (s_kbps == 0 && detect(false))) onIgnitionOn();
          else s_can.configure(s_kbps ? s_kbps : 500, true, true, false);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        break;
      }

      case State::Running:
        sup::setNote(s_hb, "poll");
        parseCustom();
        pollTiers();
        updateTrip();
        if (r_dtcClear) { r_dtcClear = false; clearDtcs(); }
        if (r_dtcRead || due(s_nextDtcMs)) {
          r_dtcRead = false;
          s_nextDtcMs = after(300000UL);
          readDtcs();
        }
        static uint32_t nextVin = 0;
        if (!s_vin[0] && due(nextVin)) { nextVin = after(60000); readVin(); }
        runCustom();
        checkBus();
        if (r_rescan) { r_rescan = false; setState(State::Detect); }
        if (s_timeoutStreak >= 6 || millis() - s_lastOkMs > 15000) {
          LOGI(TAG, "ECU silent -> ignition off");
          onIgnitionOff();
          nextProbe = after(10000);  // quick re-check in case it was a glitch
        }
        if (s_sniff) { CanFrame f; while (s_can.receive(f)) sniffHook(f); }
        vTaskDelay(pdMS_TO_TICKS(20));
        break;
    }
  }
}

void begin() {
  s_mtx = xSemaphoreCreateMutex();
  s_rawQ = xQueueCreate(4, sizeof(RawTx));
}

void start() {
  xTaskCreatePinnedToCore(task, "obd", 8192, nullptr, 6, nullptr, 1);
}

// ------------------------------------------------------------------ reports
void toJson(JsonObject o) {
  Lock l;
  o["ign"] = ignitionOn();
  o["eng"] = engineRunning();
  if (s_vin[0]) o["vin"] = s_vin;
  o["mil"] = s_mil;
  o["dtc_n"] = s_nDtc[0];
  JsonObject v = o["v"].to<JsonObject>();
  size_t tn;
  const PidDef* t = pids::table(tn);
  for (size_t i = 0; i < tn; i++) {
    uint8_t p = t[i].pid;
    if (supported(p) && fresh(p, pids::TIER_MS[t[i].tier] * 3 + 2000)) v[t[i].key] = serialized(String(s_vals[p].v, 2));
  }
  if (!fresh(0x5E, 5000) && fresh(0x10, 5000)) v["fuel_lph_est"] = serialized(String(s_vals[0x10].v * 3600.0f / (14.7f * 745.0f), 2));
  if (ignitionOn()) {
    JsonObject tr = o["trip"].to<JsonObject>();
    tr["km"] = serialized(String(s_trip.km, 2));
    tr["max_kmh"] = (int)s_trip.maxKmh;
    tr["idle_s"] = (int)s_trip.idleS;
    tr["fuel_l"] = serialized(String(s_trip.fuelL, 2));
    tr["dur_s"] = (millis() - s_trip.startMs) / 1000;
  }
  if (s_nCustom) {
    JsonObject cu = o["custom"].to<JsonObject>();
    for (uint8_t i = 0; i < s_nCustom; i++) if (s_custom[i].last[0]) cu[s_custom[i].key] = s_custom[i].last;
  }
}

void dtcJson(JsonObject o) {
  Lock l;
  static const char* K[3] = {"stored", "pending", "permanent"};
  o["mil"] = s_mil;
  for (int m = 0; m < 3; m++) {
    JsonArray a = o[K[m]].to<JsonArray>();
    for (int i = 0; i < s_nDtc[m]; i++) a.add(s_dtc[m][i]);
  }
}

bool dtcChanged() {
  bool c = s_dtcChanged;
  s_dtcChanged = false;
  return c;
}

void infoJson(JsonObject o) {
  Lock l;
  o["state"] = stateName();
  if (s_kbps) {
    char p[16];
    snprintf(p, sizeof p, "CAN%s/%u", s_ext ? "29" : "11", s_kbps);
    o["proto"] = p;
  }
  if (s_vin[0]) o["vin"] = s_vin;
  JsonArray e = o["ecus"].to<JsonArray>();
  for (int i = 0; i < s_nEcus; i++) { char h[12]; snprintf(h, sizeof h, "%lX", (unsigned long)s_ecus[i]); e.add(h); }
  JsonArray a = o["pids"].to<JsonArray>();
  size_t tn;
  const PidDef* t = pids::table(tn);
  for (size_t i = 0; i < tn; i++) if (supported(t[i].pid)) a.add(t[i].key);
}

void healthJson(JsonObject o) {
  o["st"] = stateName();
  o["kbps"] = s_kbps;
  o["detect"] = s_detectAttempts;
  o["busoff"] = s_busOffs;
  if (s_state != State::NoChip && s_state != State::Disabled) {
    CanStatus st = s_can.status();
    o["tec"] = st.tec;
    o["rec"] = st.rec;
  }
  if (s_lastErr[0]) o["err"] = s_lastErr;
}

void requestDtcRead() { r_dtcRead = true; }
void requestDtcClear() { r_dtcClear = true; }
void requestRescan() { r_rescan = true; }
void setSniff(bool on) { s_sniff = on; if (on) LOGW(TAG, "sniff on: OBD response IDs only while polling, all IDs while ignition off"); }

bool sendRaw(uint32_t id, bool ext, const uint8_t* data, uint8_t len) {
  RawTx r{id, ext, (uint8_t)min<uint8_t>(len, 8), {0}};
  memcpy(r.d, data, r.len);
  return xQueueSend(s_rawQ, &r, 0) == pdTRUE;
}

void print(Print& out) {
  out.printf("OBD state=%s protocol=%s%s/%u ECUs=%u primary=0x%lX VIN=%s\n", stateName(), s_kbps ? "CAN" : "-",
             s_ext ? "29" : "11", s_kbps, s_nEcus, (unsigned long)s_primaryRx, s_vin[0] ? s_vin : "-");
  out.printf("  MIL %s  DTC stored %u pending %u permanent %u  timeouts streak %u\n", s_mil ? "ON" : "off", s_nDtc[0],
             s_nDtc[1], s_nDtc[2], s_timeoutStreak);
  out.printf("  trip %.2f km  max %.0f km/h  idle %.0f s  harsh accel %u brake %u\n", s_trip.km, s_trip.maxKmh,
             s_trip.idleS, s_trip.harshAccel, s_trip.harshBrake);
  if (s_state != State::NoChip && s_state != State::Disabled) {
    CanStatus st = s_can.status();
    out.printf("  CAN TEC %u REC %u LEC %u %s%s%s  bus-offs %lu  detect attempts %lu  last error '%s'\n", st.tec, st.rec,
               st.lec, st.busOff ? "BUS-OFF " : "", st.errPassive ? "ERR-PASSIVE " : "", st.errWarn ? "WARN" : "",
               (unsigned long)s_busOffs, (unsigned long)s_detectAttempts, s_lastErr);
  }
}

void printPids(Print& out) {
  size_t tn;
  const PidDef* t = pids::table(tn);
  out.printf("%-4s %-16s %12s %-6s %s\n", "PID", "key", "value", "unit", "age");
  for (size_t i = 0; i < tn; i++) {
    uint8_t p = t[i].pid;
    if (!supported(p)) continue;
    if (s_vals[p].ms)
      out.printf("%02X   %-16s %12.2f %-6s %lus\n", p, t[i].key, s_vals[p].v, t[i].unit,
                 (unsigned long)((millis() - s_vals[p].ms) / 1000));
    else
      out.printf("%02X   %-16s %12s %-6s\n", p, t[i].key, "-", t[i].unit);
  }
}

}  // namespace obd

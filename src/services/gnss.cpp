#include "services/gnss.h"
#include <Preferences.h>
#include <math.h>
#include <freertos/semphr.h>
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/deadline.h"
#include "core/log.h"
#include "core/msgbus.h"

namespace gnss {

static const char* TAG = "GNSS";
static SemaphoreHandle_t s_mtx;
static LocInfo s_loc{};
static uint32_t s_locMs = 0;          // millis of the reported position
static uint32_t s_lastFixMs = 0;      // millis of the last live GNSS fix
static uint32_t s_engineStartMs = 0;
static bool s_engineOn = false;
static bool s_hadFix = false;
static uint8_t s_errStreak = 0;
static uint32_t s_nextPollMs = 0;
static uint32_t s_lastAgnssMs = 0, s_lastLbsMs = 0, s_lastColdMs = 0;
static bool s_lbsUnsupported = false;
static double s_prevLat = 0, s_prevLon = 0;
static bool s_havePrev = false;
static double s_odoSaved = 0;
static uint32_t s_lastPersistMs = 0;

struct Lock {
  Lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(s_mtx); }
};

const char* srcName(LocSrc s) {
  switch (s) {
    case LocSrc::Gnss: return "gnss";
    case LocSrc::Lbs:  return "lbs";
    case LocSrc::Last: return "last";
    default:           return "none";
  }
}

static double haversineKm(double la1, double lo1, double la2, double lo2) {
  const double R = 6371.0088, d2r = M_PI / 180.0;
  double dla = (la2 - la1) * d2r, dlo = (lo2 - lo1) * d2r;
  double a = sin(dla / 2) * sin(dla / 2) + cos(la1 * d2r) * cos(la2 * d2r) * sin(dlo / 2) * sin(dlo / 2);
  return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

static void persist(bool force) {
  if (!force && millis() - s_lastPersistMs < 300000UL) return;  // flash wear: max every 5 min
  if (!s_loc.valid || s_loc.src != LocSrc::Gnss) return;
  s_lastPersistMs = millis();
  Preferences nv;
  nv.begin("loc", false);
  nv.putDouble("lat", s_loc.lat);
  nv.putDouble("lon", s_loc.lon);
  nv.putFloat("alt", s_loc.alt);
  nv.putUInt("ts", s_loc.fixEpoch);
  nv.putDouble("odo", s_loc.odometerKm);
  nv.end();
  s_odoSaved = s_loc.odometerKm;
}

void begin() {
  s_mtx = xSemaphoreCreateMutex();
  Preferences nv;
  nv.begin("loc", true);
  if (nv.isKey("lat")) {
    s_loc.lat = nv.getDouble("lat", 0);
    s_loc.lon = nv.getDouble("lon", 0);
    s_loc.alt = nv.getFloat("alt", 0);
    s_loc.fixEpoch = nv.getUInt("ts", 0);
    s_loc.valid = true;
    s_loc.src = LocSrc::Last;
  }
  s_loc.odometerKm = s_odoSaved = nv.getDouble("odo", 0);
  nv.end();
  s_locMs = millis();
  if (s_loc.valid) LOGI(TAG, "last known position %.6f,%.6f", s_loc.lat, s_loc.lon);
}

static double nmeaToDeg(double v, double limit) {
  if (fabs(v) > limit) {  // ddmm.mmmm format
    int deg = (int)(v / 100);
    return deg + (v - deg * 100.0) / 60.0;
  }
  return v;
}

// +CGNSSINFO: <mode>,<SVs...>,<lat>,<N/S>,<lon>,<E/W>,<date ddmmyy>,<utc hhmmss.s>,<alt>,<speed kn>,<course>,<PDOP>,<HDOP>,<VDOP>
static bool parseInfo(const char* v, LocInfo& f) {
  char tmp[200];
  strlcpy(tmp, v, sizeof tmp);
  char* tok[28];
  int n = 0;
  for (char* p = tmp; n < 28;) {
    tok[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = 0;
    p = c + 1;
  }
  int ns = -1;
  for (int i = 2; i < n; i++)
    if (!strcmp(tok[i], "N") || !strcmp(tok[i], "S")) { ns = i; break; }
  if (ns < 2 || !*tok[ns - 1] || ns + 10 >= n) return false;
  f.fixMode = atoi(tok[0]);
  int sv = 0;
  for (int i = 1; i < ns - 1; i++) sv += atoi(tok[i]);
  f.sats = sv;
  f.lat = nmeaToDeg(atof(tok[ns - 1]), 90.0) * (tok[ns][0] == 'S' ? -1 : 1);
  f.lon = nmeaToDeg(atof(tok[ns + 1]), 180.0) * (tok[ns + 2][0] == 'W' ? -1 : 1);
  const char* date = tok[ns + 3];
  const char* utc = tok[ns + 4];
  f.alt = atof(tok[ns + 5]);
  f.speedKmh = atof(tok[ns + 6]) * 1.852f;
  f.course = atof(tok[ns + 7]);
  f.hdop = atof(tok[ns + 9]);
  f.fixEpoch = 0;
  if (strlen(date) >= 6 && strlen(utc) >= 6) {
    auto two = [](const char* s) { return (s[0] - '0') * 10 + (s[1] - '0'); };
    f.fixEpoch = clockx::fromYmdHms(2000 + two(date + 4), two(date + 2), two(date), two(utc), two(utc + 2), two(utc + 4));
  }
  return f.lat != 0 || f.lon != 0;
}

// Active antennas need DC on the RF line (modem VAUX through a bias-T on the board).
// Passive antennas must NOT get DC -> default off.
static bool s_antBias = false;
static void applyAntenna(AtChannel& at) {
  const Config& c = cfg();
  if (c.gnss_ant_active) {
    at.cmd(3000, nullptr, "AT+CVAUXV=%ld", (long)c.gnss_ant_mv);
    s_antBias = at.cmd(3000, nullptr, "AT+CVAUXS=1") == At::OK;
    LOGI(TAG, "active antenna: VAUX %ld mV %s", (long)c.gnss_ant_mv, s_antBias ? "on" : "FAILED");
  } else {
    at.cmd(3000, nullptr, "AT+CVAUXS=0");
    s_antBias = false;
  }
}

static bool ensureEngine(AtChannel& at) {
  if (s_engineOn) return true;
  AtResp r;
  char v[16];
  if (at.cmd(3000, &r, "AT+CGNSSPWR?") == At::OK && r.get("+CGNSSPWR:", v, sizeof v) && atoi(v) == 1) {
    s_engineOn = true;
  } else {
    char urc[48];
    At res = at.cmdUrc(20000, "+CGNSSPWR: READY", urc, sizeof urc, nullptr, "AT+CGNSSPWR=1");
    s_engineOn = res == At::OK || (res == At::TIMEOUT && at.cmd(3000, &r, "AT+CGNSSPWR?") == At::OK &&
                                   r.get("+CGNSSPWR:", v, sizeof v) && atoi(v) == 1);
  }
  if (s_engineOn) {
    s_engineStartMs = millis();
    s_hadFix = false;
    applyAntenna(at);
    if (cfg().gnss_mode > 0) at.cmd(3000, nullptr, "AT+CGNSSMODE=%ld", (long)cfg().gnss_mode);
    LOGI(TAG, "engine on");
  } else {
    LOGW(TAG, "engine power-on failed: %s", at.lastError());
  }
  return s_engineOn;
}

bool restart(AtChannel& at, const char* how) {
  CNT_INC(gnss_restarts);
  bool ok;
  if (!strcmp(how, "cold")) { ok = at.cmd(5000, nullptr, "AT+CGPSCOLD") == At::OK; s_lastColdMs = millis(); }
  else if (!strcmp(how, "warm")) ok = at.cmd(5000, nullptr, "AT+CGPSWARM") == At::OK;
  else if (!strcmp(how, "hot")) ok = at.cmd(5000, nullptr, "AT+CGPSHOT") == At::OK;
  else {
    at.cmd(5000, nullptr, "AT+CGNSSPWR=0");
    s_engineOn = false;
    ok = ensureEngine(at);
  }
  s_engineStartMs = millis();
  s_hadFix = false;
  LOGI(TAG, "%s start -> %s", how, ok ? "ok" : at.lastError());
  return ok;
}

bool requestAgnss(AtChannel& at) {
  s_lastAgnssMs = millis();
  AtResp r;
  At res = at.cmd(60000, &r, "%s", cfg().agnss_cmd);
  bool ok = res == At::OK && !r.has("ERROR");
  if (ok) { CNT_INC(agnss_ok); Lock l; s_loc.assisted = true; }
  else CNT_INC(agnss_fail);
  LOGI(TAG, "AGNSS %s %s", ok ? "ok" : "failed", ok ? "" : at.lastError());
  return ok;
}

// Cell-tower location from the modem. Response (SIMCom LBS): +CLBS: 0,<lat>,<lon>,<acc>
static bool tryLbs(AtChannel& at) {
  s_lastLbsMs = millis();
  if (s_lbsUnsupported) return false;
  char urc[96];
  At res = at.cmdUrc(30000, "+CLBS:", urc, sizeof urc, nullptr, "AT+CLBS=1");
  int code = -1;
  double lat = 0, lon = 0;
  int acc = 0;
  if (res == At::OK && sscanf(urc, "+CLBS: %d,%lf,%lf,%d", &code, &lat, &lon, &acc) >= 3 && code == 0 &&
      (lat != 0 || lon != 0)) {
    CNT_INC(lbs_ok);
    Lock l;
    if (!s_loc.valid || s_loc.src != LocSrc::Gnss) {
      s_loc.valid = true;
      s_loc.src = LocSrc::Lbs;
      s_loc.lat = lat;
      s_loc.lon = lon;
      s_loc.accuracyM = acc;
      s_loc.speedKmh = 0;
      s_loc.fixEpoch = clockx::now();
      s_locMs = millis();
    }
    LOGI(TAG, "cell location %.5f,%.5f +-%dm", lat, lon, acc);
    return true;
  }
  CNT_INC(lbs_fail);
  if (res == At::ERROR) {
    s_lbsUnsupported = true;  // firmware has no LBS: rely on cell IDs in the report
    LOGW(TAG, "AT+CLBS not supported - server-side cell location only");
  }
  return false;
}

void step(AtChannel& at, bool netOnline) {
  if (!due(s_nextPollMs)) return;
  s_nextPollMs = after(cfg().gnss_poll_s * 1000UL);
  if (!ensureEngine(at)) { s_nextPollMs = after(30000); return; }

  AtResp r;
  char v[200];
  if (at.cmd(3000, &r, "AT+CGNSSINFO") != At::OK || !r.get("+CGNSSINFO:", v, sizeof v)) {
    if (++s_errStreak >= 5) {
      LOGW(TAG, "engine not answering -> power cycle engine");
      restart(at, "power");
      s_errStreak = 0;
    }
    return;
  }
  s_errStreak = 0;

  LocInfo f{};
  bool fix = parseInfo(v, f) && f.hdop * 10 <= cfg().gnss_max_hdop10;
  uint32_t now = millis();

  if (fix) {
    if (f.fixEpoch) clockx::set(f.fixEpoch, clockx::GNSS);
    Lock l;
    if (!s_hadFix) {
      s_hadFix = true;
      s_loc.ttffMs = now - s_engineStartMs;
      CNT_INC(gnss_fixes);
      LOGI(TAG, "fix acquired, TTFF %lu s, %u sats, HDOP %.1f", (unsigned long)(s_loc.ttffMs / 1000), f.sats, f.hdop);
    }
    bool moving = f.speedKmh >= cfg().move_kmh;
    // odometer: integrate only while moving, reject jumps (> 250 km/h implied)
    if (s_havePrev && moving) {
      double d = haversineKm(s_prevLat, s_prevLon, f.lat, f.lon);
      double dtH = (now - s_lastFixMs) / 3600000.0;
      if (dtH > 0 && d / dtH < 250.0) s_loc.odometerKm += d;
    }
    s_prevLat = f.lat;
    s_prevLon = f.lon;
    s_havePrev = true;
    s_loc.valid = true;
    s_loc.src = LocSrc::Gnss;
    s_loc.lat = f.lat;
    s_loc.lon = f.lon;
    s_loc.alt = f.alt;
    s_loc.speedKmh = f.speedKmh;
    s_loc.course = f.course;
    s_loc.hdop = f.hdop;
    s_loc.accuracyM = f.hdop * 5.0f;  // rough CEP estimate
    s_loc.sats = f.sats;
    s_loc.fixMode = f.fixMode;
    s_loc.fixEpoch = f.fixEpoch ? f.fixEpoch : clockx::now();
    s_loc.moving = moving;
    s_lastFixMs = now;
    s_locMs = now;
  } else {
    Lock l;
    if (s_hadFix && now - s_lastFixMs > 10000) {
      s_hadFix = false;
      s_engineStartMs = now;
      CNT_INC(gnss_fix_lost);
      LOGW(TAG, "fix lost");
    }
    if (s_loc.src == LocSrc::Gnss && now - s_lastFixMs > 30000) {
      s_loc.src = LocSrc::Last;  // keep coordinates, mark stale
      s_loc.moving = false;
      s_loc.speedKmh = 0;
    }
  }
  persist(false);

  // ---- fallbacks while there is no fix
  if (!fix) {
    uint32_t noFix = (now - (s_lastFixMs ? s_lastFixMs : s_engineStartMs)) / 1000;
    const Config& c = cfg();
    if (netOnline && c.agnss_after_s && noFix >= (uint32_t)c.agnss_after_s &&
        (!s_lastAgnssMs || now - s_lastAgnssMs > 2 * 3600000UL))
      requestAgnss(at);
    if (netOnline && c.lbs_after_s && noFix >= (uint32_t)c.lbs_after_s &&
        (!s_lastLbsMs || now - s_lastLbsMs > 600000UL))
      tryLbs(at);
    // stuck for 30 min: purge ephemeris once every 6 h
    if (noFix > 1800 && (!s_lastColdMs || now - s_lastColdMs > 6 * 3600000UL)) restart(at, "cold");
  }
}

void powerOff(AtChannel& at) {
  persist(true);
  at.cmd(5000, nullptr, "AT+CGNSSPWR=0");
  s_engineOn = false;
}

void modemReset() {
  s_engineOn = false;
  s_hadFix = false;
  s_nextPollMs = after(5000);
}

void snapshot(LocInfo& out) {
  Lock l;
  out = s_loc;
  uint32_t now = millis();
  out.ageMs = now - s_locMs;
  out.noFixSec = s_hadFix ? 0 : (now - (s_lastFixMs ? s_lastFixMs : s_engineStartMs)) / 1000;
  out.engineOn = s_engineOn;
}

void toJson(JsonObject o) {
  LocInfo l;
  snapshot(l);
  o["on"] = l.engineOn;
  o["src"] = srcName(l.src);
  o["sat"] = l.sats;
  o["hdop"] = serialized(String(l.hdop, 1));
  o["nofix_s"] = l.noFixSec;
  o["ttff_s"] = l.ttffMs / 1000;
  o["agnss"] = l.assisted;
  o["lbs"] = !s_lbsUnsupported;
  o["ant"] = s_antBias ? "active" : "passive";
}

void print(Print& out) {
  LocInfo l;
  snapshot(l);
  out.printf("GNSS engine=%s src=%s valid=%d fix=%dD sats=%u hdop=%.1f\n", l.engineOn ? "on" : "off", srcName(l.src),
             l.valid, l.fixMode, l.sats, l.hdop);
  out.printf("  pos %.6f, %.6f alt %.1f m  speed %.1f km/h course %.0f  age %lu s\n", l.lat, l.lon, l.alt,
             l.speedKmh, l.course, (unsigned long)(l.ageMs / 1000));
  out.printf("  no-fix %lu s  TTFF %lu s  odometer %.2f km  AGNSS %s  LBS %s\n", (unsigned long)l.noFixSec,
             (unsigned long)(l.ttffMs / 1000), l.odometerKm, l.assisted ? "used" : "no",
             s_lbsUnsupported ? "unsupported" : "available");
  out.printf("  antenna %s (VAUX bias %s)\n", cfg().gnss_ant_active ? "active" : "passive", s_antBias ? "ON" : "off");
  if (l.valid) out.printf("  https://maps.google.com/?q=%.6f,%.6f\n", l.lat, l.lon);
}

}  // namespace gnss

// =============================================================================
//  Fleet Tracker - ACTIVE GPS ANTENNA TEST firmware   (pio env: gps_test)
//
//  Powers the A7672SA, keeps the CELLULAR RADIO OFF (AT+CFUN=4), starts the
//  internal GNSS engine and analyses the NMEA stream live:
//    - per-constellation satellites in view / tracked / used
//    - per-satellite C/N0 (signal strength) with elevation + azimuth
//    - milestones: first NMEA, first satellite, first >=30 dB-Hz, time, 2D/3D fix
//    - top-4 average C/N0 (the standard antenna quality number) + trend
//    - bias-tee calculator (enter the measured antenna voltage)
//    - optional interference test (cellular ON vs OFF)
//    - automatic verdict with likely hardware causes
//
//  Serial monitor 115200, line ending CR or CR+LF.  Commands: 'h'
// =============================================================================
#include <Arduino.h>
#include "board_pins.h"
#include "../test_fw/modem_at.h"
#include "gps_test_config.h"

static Modem modem;

// ----------------------------------------------------------------------------
//  Data model
// ----------------------------------------------------------------------------
enum { C_GPS, C_GLO, C_GAL, C_BDS, C_QZS, C_N };
static const char* CNAME[C_N] = {"GPS", "GLONASS", "Galileo", "BeiDou", "QZSS"};
static const char CPFX[C_N] = {'G', 'R', 'E', 'C', 'J'};

struct Sat {
  uint16_t prn;
  int8_t elev;    // degrees, -1 unknown
  int16_t az;     // degrees, -1 unknown
  uint8_t snr;    // C/N0 dB-Hz, 0 = in view but not tracked
  uint32_t seen;  // millis of last report
};
static const int MAXSAT = 40;
struct Constel {
  Sat s[MAXSAT];
  int n;
  int inView;
  uint32_t viewSeen;
};
static Constel g_con[C_N];

struct Fix {
  int quality = 0;   // GGA: 0 none, 1 GPS, 2 DGPS, 6 estimated
  int fixType = 1;   // GSA: 1 none, 2 = 2D, 3 = 3D
  int used = 0;
  bool rmcValid = false;
  double lat = 0, lon = 0;
  float alt = 0, hdop = 0, pdop = 0, vdop = 0, speedKn = 0;
  String utc, date;
} g_fix;

// Milestones (absolute millis, 0 = not reached yet)
struct Miles {
  uint32_t start, nmea, firstView, firstTrack, first30, timeKnown, fix2d, fix3d;
} g_ms;

struct Stats {
  int view[C_N], trk[C_N];
  int view_all = 0, trk_all = 0, n30 = 0, n35 = 0;
  int maxCn0 = 0;
  float top4 = 0;
  int hiN = 0;          // satellites above 45 deg with signal
  float hiAvg = 0;      // their average C/N0
  int hiDark = 0;       // satellites above 45 deg in view but NOT tracked
} g_st;

// Run-wide peaks and counters
static int g_peakMax = 0, g_peakTrk = 0, g_peakUsed = 0;
static float g_peakTop4 = 0;
static uint32_t g_nmeaOk = 0, g_nmeaBad = 0;
static float g_trend[120];
static int g_trendN = 0;
static const char* g_startKind = "cold";
static bool g_raw = false;
static bool g_diagDone = false;

// Bias / interference results (for the verdict)
static float g_antV = -1, g_antmA = -1, g_vbatAtMeas = 0;
static float g_jamOff = -1, g_jamOn = -1;

// ----------------------------------------------------------------------------
//  Small helpers
// ----------------------------------------------------------------------------
static uint32_t since(uint32_t t) { return t ? (t - g_ms.start) / 1000 : 0; }
static uint32_t elapsedS() { return (millis() - g_ms.start) / 1000; }

static int splitCsv(const String& s, String* out, int maxOut) {
  int n = 0, start = 0;
  for (int i = 0; i <= (int)s.length() && n < maxOut; i++) {
    if (i == (int)s.length() || s[i] == ',') {
      out[n++] = s.substring(start, i);
      start = i + 1;
    }
  }
  return n;
}

static double nmeaDeg(const String& v, const String& hemi) {
  if (v.length() < 4) return 0;
  double x = v.toDouble();
  int deg = (int)(x / 100);
  double d = deg + (x - deg * 100) / 60.0;
  return (hemi == "S" || hemi == "W") ? -d : d;
}

static int talker(const String& l) {
  if (l.startsWith("$GP")) return C_GPS;
  if (l.startsWith("$GL")) return C_GLO;
  if (l.startsWith("$GA")) return C_GAL;
  if (l.startsWith("$GB") || l.startsWith("$BD")) return C_BDS;
  if (l.startsWith("$GQ")) return C_QZS;
  return -1;  // $GN = combined
}

static bool checksumOk(const String& l) {
  int star = l.lastIndexOf('*');
  if (star < 1 || star + 3 > (int)l.length()) return false;
  uint8_t x = 0;
  for (int i = 1; i < star; i++) x ^= (uint8_t)l[i];
  return x == (uint8_t)strtol(l.substring(star + 1, star + 3).c_str(), nullptr, 16);
}

static String bar(int snr) {
  String b;
  for (int i = 0; i < snr / 3; i++) b += '#';
  return b;
}

static float readVbat() {
  String r;
  bool v = modem.verbose();
  modem.setVerbose(false);
  float x = 0;
  if (modem.cmd("AT+CBC", 2000, &r) == AtResult::OK) x = Modem::field(r, "+CBC:").toFloat();
  modem.setVerbose(v);
  return x;
}

static int readCfun() {
  String r;
  if (modem.cmd("AT+CFUN?", 3000, &r) != AtResult::OK) return -1;
  return Modem::field(r, "+CFUN:").toInt();
}

// ----------------------------------------------------------------------------
//  Satellite bookkeeping
// ----------------------------------------------------------------------------
static void upsertSat(int c, uint16_t prn, int elev, int az, int snr) {
  Constel& k = g_con[c];
  Sat* s = nullptr;
  for (int i = 0; i < k.n; i++)
    if (k.s[i].prn == prn) { s = &k.s[i]; break; }
  if (!s) {
    if (k.n < MAXSAT) s = &k.s[k.n++];
    else {  // replace the oldest entry
      s = &k.s[0];
      for (int i = 1; i < k.n; i++)
        if (k.s[i].seen < s->seen) s = &k.s[i];
    }
    s->prn = prn;
  }
  s->elev = elev;
  s->az = az;
  s->snr = snr;
  s->seen = millis();
}

static void resetAll(const char* kind) {
  memset(g_con, 0, sizeof g_con);
  g_fix = Fix();
  memset(&g_ms, 0, sizeof g_ms);
  g_ms.start = millis();
  g_st = Stats();
  g_peakMax = g_peakTrk = g_peakUsed = 0;
  g_peakTop4 = 0;
  g_trendN = 0;
  g_nmeaOk = g_nmeaBad = 0;
  g_startKind = kind;
  g_diagDone = false;
}

static void computeStats() {
  uint32_t now = millis();
  Stats st;
  int all[C_N * MAXSAT];
  int na = 0;
  float hiSum = 0;
  for (int c = 0; c < C_N; c++) {
    Constel& k = g_con[c];
    st.view[c] = (k.viewSeen && now - k.viewSeen < CFG_SAT_STALE_MS) ? k.inView : 0;
    st.trk[c] = 0;
    for (int i = 0; i < k.n; i++) {
      Sat& s = k.s[i];
      if (now - s.seen > CFG_SAT_STALE_MS) continue;
      if (s.snr > 0) {
        st.trk[c]++;
        all[na++] = s.snr;
        if (s.snr >= 30) st.n30++;
        if (s.snr >= 35) st.n35++;
        if (s.snr > st.maxCn0) st.maxCn0 = s.snr;
        if (s.elev >= 45) { st.hiN++; hiSum += s.snr; }
      } else if (s.elev >= 45) {
        st.hiDark++;
      }
    }
    st.view_all += st.view[c];
    st.trk_all += st.trk[c];
  }
  // top-4 average
  for (int i = 0; i < na; i++)
    for (int j = i + 1; j < na; j++)
      if (all[j] > all[i]) { int t = all[i]; all[i] = all[j]; all[j] = t; }
  int m = na < 4 ? na : 4;
  float sum = 0;
  for (int i = 0; i < m; i++) sum += all[i];
  st.top4 = m ? sum / m : 0;
  st.hiAvg = st.hiN ? hiSum / st.hiN : 0;
  g_st = st;

  if (st.maxCn0 > g_peakMax) g_peakMax = st.maxCn0;
  if (st.top4 > g_peakTop4) g_peakTop4 = st.top4;
  if (st.trk_all > g_peakTrk) g_peakTrk = st.trk_all;
  if (g_fix.used > g_peakUsed) g_peakUsed = g_fix.used;

  if (!g_ms.firstView && st.view_all > 0) g_ms.firstView = now;
  if (!g_ms.firstTrack && st.trk_all > 0) g_ms.firstTrack = now;
  if (!g_ms.first30 && st.maxCn0 >= 30) g_ms.first30 = now;
}

// ----------------------------------------------------------------------------
//  NMEA processing
// ----------------------------------------------------------------------------
static void milestone(const char* what) {
  Serial.printf("\n  *** [%4lu s] MILESTONE: %s\n", (unsigned long)elapsedS(), what);
}

static void processNmea(const String& line) {
  if (!checksumOk(line)) { g_nmeaBad++; return; }
  g_nmeaOk++;
  uint32_t now = millis();
  if (!g_ms.nmea) { g_ms.nmea = now; milestone("NMEA stream from GNSS engine is running"); }
  if (g_raw) Serial.printf("  %s\n", line.c_str());

  String body = line.substring(0, line.lastIndexOf('*'));
  String f[28];
  int n = splitCsv(body, f, 28);
  if (body.length() < 6) return;
  String type = body.substring(3, 6);

  if (type == "GSV" && n >= 4) {
    int c = talker(body);
    if (c < 0) return;
    g_con[c].inView = f[3].toInt();
    g_con[c].viewSeen = now;
    for (int i = 4; i + 3 < n; i += 4) {
      int prn = f[i].toInt();
      if (prn <= 0) continue;
      int el = f[i + 1].length() ? f[i + 1].toInt() : -1;
      int az = f[i + 2].length() ? f[i + 2].toInt() : -1;
      int snr = f[i + 3].length() ? f[i + 3].toInt() : 0;
      upsertSat(c, prn, el, az, snr);
    }
  } else if (type == "GGA" && n >= 10) {
    g_fix.quality = f[6].toInt();
    g_fix.used = f[7].toInt();
    g_fix.hdop = f[8].toFloat();
    g_fix.alt = f[9].toFloat();
    if (g_fix.quality > 0) {
      g_fix.lat = nmeaDeg(f[2], f[3]);
      g_fix.lon = nmeaDeg(f[4], f[5]);
    }
  } else if (type == "RMC" && n >= 10) {
    g_fix.utc = f[1];
    g_fix.date = f[9];
    g_fix.rmcValid = f[2] == "A";
    g_fix.speedKn = f[7].toFloat();
    if (!g_ms.timeKnown && f[1].length() >= 6 && f[9].length() == 6) {
      g_ms.timeKnown = now;
      milestone(("UTC time decoded from satellites: " + f[9] + " " + f[1].substring(0, 6)).c_str());
    }
  } else if (type == "GSA" && n >= 18) {
    int ft = f[2].toInt();
    if (ft >= 1 && ft <= 3) g_fix.fixType = ft;
    g_fix.pdop = f[15].toFloat();
    g_fix.hdop = f[16].toFloat();
    g_fix.vdop = f[17].toFloat();
  }

  bool fixed = g_fix.quality > 0 || g_fix.rmcValid;
  if (fixed && !g_ms.fix2d) {
    g_ms.fix2d = now;
    char b[160];
    snprintf(b, sizeof b, "POSITION FIX (%s start) TTFF = %lu s  ->  %.6f, %.6f", g_startKind,
             (unsigned long)since(now), g_fix.lat, g_fix.lon);
    milestone(b);
  }
  if (fixed && g_fix.fixType == 3 && !g_ms.fix3d) {
    g_ms.fix3d = now;
    milestone("3D fix (altitude valid)");
  }
}

static void handleModemLine(const String& l) {
  if (l.startsWith("$")) { processNmea(l); return; }
  if (l == "OK") return;
  Serial.printf("  [modem] %s\n", l.c_str());
}

// Read and process everything the modem sends for `ms` milliseconds.
static void pumpFor(uint32_t ms) {
  uint32_t t0 = millis(), lastStat = 0;
  String l;
  while (millis() - t0 < ms) {
    if (modem.readLine(l, 20, false)) handleModemLine(l);
    if (millis() - lastStat >= 1000) { lastStat = millis(); computeStats(); }
  }
}

// ----------------------------------------------------------------------------
//  Output
// ----------------------------------------------------------------------------
static void printDash() {
  computeStats();
  bool fixed = g_fix.quality > 0 || g_fix.rmcValid;
  Serial.printf("\n[%4lu s] %s start | FIX: %s | used %d | view %d | tracked %d | max %d | top4 %.1f dB-Hz | >=30: %d\n",
                (unsigned long)elapsedS(), g_startKind,
                fixed ? (g_fix.fixType == 3 ? "3D" : "2D") : "none", g_fix.used, g_st.view_all, g_st.trk_all,
                g_st.maxCn0, g_st.top4, g_st.n30);
  uint32_t now = millis();
  for (int c = 0; c < C_N; c++) {
    if (!g_st.view[c] && !g_st.trk[c]) continue;
    String s;
    Constel& k = g_con[c];
    for (int i = 0; i < k.n; i++)
      if (k.s[i].snr > 0 && now - k.s[i].seen <= CFG_SAT_STALE_MS)
        s += String(CPFX[c]) + String(k.s[i].prn) + ":" + String(k.s[i].snr) + " ";
    Serial.printf("         %-8s view %2d  trk %2d | %s\n", CNAME[c], g_st.view[c], g_st.trk[c], s.c_str());
  }
  if (fixed)
    Serial.printf("         pos %.6f, %.6f  alt %.1f m  HDOP %.1f  speed %.1f km/h  UTC %s\n", g_fix.lat,
                  g_fix.lon, g_fix.alt, g_fix.hdop, g_fix.speedKn * 1.852f, g_fix.utc.c_str());
  if (g_nmeaOk == 0) Serial.println("         (no NMEA yet)");
  if (g_trendN < (int)(sizeof g_trend / sizeof g_trend[0])) g_trend[g_trendN++] = g_st.top4;
}

static void printTable() {
  computeStats();
  uint32_t now = millis();
  Serial.printf("\n  ---- satellite table at %lu s ------------------------------------------\n",
                (unsigned long)elapsedS());
  Serial.println("  sat    elev  azim  C/N0  signal (each # = 3 dB-Hz)");
  int rows = 0;
  for (int c = 0; c < C_N; c++) {
    Constel& k = g_con[c];
    for (int i = 0; i < k.n; i++) {
      Sat& s = k.s[i];
      if (now - s.seen > CFG_SAT_STALE_MS) continue;
      String el = s.elev >= 0 ? String(s.elev) : "-";
      String az = s.az >= 0 ? String(s.az) : "-";
      String q = s.snr == 0 ? "not tracked" : s.snr >= CFG_CN0_GOOD ? "strong" : s.snr >= CFG_CN0_OK ? "ok"
                 : s.snr >= CFG_CN0_WEAK ? "weak" : "very weak";
      Serial.printf("  %c%-4u %5s %5s  %4s  %-16s %s\n", CPFX[c], s.prn, el.c_str(), az.c_str(),
                    s.snr ? String(s.snr).c_str() : "--", bar(s.snr).c_str(), q.c_str());
      rows++;
    }
  }
  if (!rows) Serial.println("  (no satellites reported)");
  if (g_st.hiN || g_st.hiDark)
    Serial.printf("  high-elevation (>=45 deg): %d tracked, avg %.1f dB-Hz, %d in view but NOT tracked\n",
                  g_st.hiN, g_st.hiAvg, g_st.hiDark);
  Serial.printf("  NMEA sentences ok %lu, checksum errors %lu\n", (unsigned long)g_nmeaOk,
                (unsigned long)g_nmeaBad);
  Serial.println("  ------------------------------------------------------------------------");
}

static void printTrend() {
  Serial.printf("  top-4 C/N0 trend (every %d s): ", CFG_DASH_EVERY_S);
  int from = g_trendN > 24 ? g_trendN - 24 : 0;
  for (int i = from; i < g_trendN; i++) Serial.printf("%.0f ", g_trend[i]);
  Serial.println();
}

static void line(const char* k, const String& v) { Serial.printf("  %-30s %s\n", k, v.c_str()); }
static String msStr(uint32_t t) { return t ? String(since(t)) + " s" : String("not reached"); }

static void diagnose() {
  computeStats();
  uint32_t el = elapsedS();
  Serial.println("\n==================== ACTIVE ANTENNA DIAGNOSIS ====================");
  line("Test time", String(el) + " s (" + g_startKind + " start), cellular CFUN=" + String(CFG_CFUN_MODE));
  line("First NMEA", msStr(g_ms.nmea));
  line("First satellite in view", msStr(g_ms.firstView));
  line("First satellite tracked", msStr(g_ms.firstTrack));
  line("First signal >= 30 dB-Hz", msStr(g_ms.first30));
  line("UTC time from satellites", msStr(g_ms.timeKnown));
  line("Position fix (TTFF)", msStr(g_ms.fix2d));
  line("3D fix", msStr(g_ms.fix3d));
  line("Peak satellites tracked", String(g_peakTrk));
  line("Peak satellites used in fix", String(g_peakUsed));
  line("Peak max C/N0", String(g_peakMax) + " dB-Hz");
  line("Peak top-4 average C/N0", String(g_peakTop4, 1) + " dB-Hz");
  line("Now: tracked / >=30 / >=35", String(g_st.trk_all) + " / " + String(g_st.n30) + " / " + String(g_st.n35));
  if (g_st.hiN || g_st.hiDark)
    line("High-elevation sats (>=45 deg)",
         String(g_st.hiN) + " tracked avg " + String(g_st.hiAvg, 1) + " dB-Hz, " + String(g_st.hiDark) + " dark");
  if (g_antV >= 0)
    line("Bias-tee (measured)", String(g_antV, 2) + " V at antenna, " + String(g_antmA, 1) + " mA (VBAT " +
                                    String(g_vbatAtMeas, 2) + " V)");
  if (g_jamOn >= 0)
    line("Interference test", "top4 cellular OFF " + String(g_jamOff, 1) + " / ON " + String(g_jamOn, 1) + " dB-Hz");
  printTrend();

  Serial.println("\n  VERDICT:");
  bool problem = false;
  if (g_nmeaOk == 0) {
    Serial.println("  [FAIL] No NMEA from the GNSS engine.");
    Serial.println("         -> AT+CGNSSPWR? must be 1; try 'c' (cold start) or AT+CGNSSPORTSWITCH=0,1");
    Serial.println("         -> this is a modem/firmware problem, not the antenna.");
    problem = true;
  } else if (g_peakTrk == 0) {
    if (el < 90) {
      Serial.println("  [WAIT] No satellite tracked yet - give it at least 90-120 s outdoors.");
    } else {
      Serial.println("  [FAIL] NO RF SIGNAL: engine runs, but not one satellite has any signal.");
      Serial.println("         This is a hardware RF path problem. Check in this order:");
      Serial.println("         1. Antenna power: measure DC at coax centre (antenna side of 47 pF), enter 'v <volts>'.");
      Serial.println("            ~VBAT = antenna open / not connected;  ~0 V = short (coax or solder bridge).");
      Serial.println("         2. 47 pF capacitor joints: pin-90 side and coax side (cold joint = open circuit).");
      Serial.println("         3. Coax shield soldered to GND (pin 89/91), centre not touching shield.");
      Serial.println("         4. Antenna outdoors, patch facing the sky, not under metal.");
      Serial.println("         5. Bypass test: 4.8 cm wire on pin 90 (via the 47 pF). Still 0 sats -> pin 90 /");
      Serial.println("            module GNSS input damaged (ESD) or the pad is not really pin 90.");
    }
    problem = true;
  } else if (g_peakMax < CFG_CN0_WEAK) {
    Serial.printf("  [FAIL] Signal far too weak (peak %d dB-Hz, a good active antenna gives 38-48).\n", g_peakMax);
    Serial.println("         -> LNA not powered properly: an unpowered active antenna works as an ATTENUATOR.");
    Serial.println("            Measure antenna voltage ('v <volts>'); needs >= 2.7 V.");
    Serial.println("         -> Antenna indoors / blocked sky, or a bad coax solder joint.");
    problem = true;
  } else if (g_peakTop4 < CFG_CN0_OK) {
    Serial.printf("  [WARN] Marginal signal (top-4 %.1f dB-Hz). Fixes will be slow or unreliable.\n", g_peakTop4);
    Serial.println("         -> check antenna voltage, coax joint, sky view; replace 100 ohm with an inductor");
    Serial.println("            (33-56 nH) or ferrite bead to recover ~1.5 dB.");
    problem = true;
  } else if (g_peakTop4 < CFG_CN0_GOOD) {
    Serial.printf("  [OK]   Usable signal (top-4 %.1f dB-Hz). Good enough to fix; best antennas reach 40+.\n",
                  g_peakTop4);
  } else {
    Serial.printf("  [PASS] Strong signal (top-4 %.1f dB-Hz) - the active antenna and bias-tee work.\n", g_peakTop4);
  }

  if (g_peakTrk > 0) {
    if (g_ms.fix2d) {
      uint32_t t = since(g_ms.fix2d);
      const char* r = strcmp(g_startKind, "hot") == 0 ? (t <= 5 ? "excellent" : t <= 15 ? "ok" : "slow")
                                                      : (t <= 45 ? "excellent" : t <= 120 ? "ok" : "slow");
      Serial.printf("  [FIX]  TTFF %lu s for a %s start -> %s\n", (unsigned long)t, g_startKind, r);
    } else if (g_peakTop4 >= CFG_CN0_OK) {
      Serial.println("  [INFO] Signal is fine but no fix yet: a cold start needs ~30-60 s of uninterrupted");
      Serial.println("         reception from >= 4 satellites to download ephemeris. Keep waiting.");
    } else {
      Serial.println("  [INFO] No fix: the signal is too weak to decode ephemeris (needs ~30 dB-Hz).");
    }
    if (g_st.hiN >= 2 && g_st.hiAvg < CFG_CN0_OK)
      Serial.println("  [WARN] High satellites are weak -> antenna not facing the sky or sitting under cover.");
    if (g_st.hiDark >= 3)
      Serial.println("  [WARN] Several high satellites are not tracked -> sky obstructed above the antenna.");
  }

  if (g_antV >= 0) {
    if (g_antmA < CFG_ANT_MIN_MA)
      Serial.println("  [BIAS] Almost no antenna current -> antenna/coax open circuit (or a passive antenna).");
    else if (g_antmA > CFG_ANT_MAX_MA || g_antV < 0.5f)
      Serial.println("  [BIAS] Very high current / ~0 V -> SHORT on the coax or antenna. Disconnect and check.");
    else if (g_antV < CFG_ANT_MIN_V)
      Serial.printf("  [BIAS] Antenna gets only %.2f V (%.1f mA) -> too low for most LNAs. Use a smaller resistor\n"
                    "         (47-68 ohm), an inductor, or check the antenna's minimum voltage.\n",
                    g_antV, g_antmA);
    else
      Serial.printf("  [BIAS] Antenna supply OK: %.2f V, %.1f mA.\n", g_antV, g_antmA);
  } else {
    Serial.println("  [TIP]  Measure DC at the coax centre and type 'v <volts>' to check the bias-tee.");
  }

  if (g_jamOn >= 0) {
    float d = g_jamOff - g_jamOn;
    if (d >= 3)
      Serial.printf("  [JAM]  C/N0 drops %.1f dB with cellular ON -> interference. Add 100 nF + 47 pF at the\n"
                    "         VBAT end of the 100 ohm, keep LTE antenna away from the GPS antenna.\n", d);
    else
      Serial.printf("  [JAM]  Cellular ON changes C/N0 by %.1f dB -> no significant interference.\n", -d);
  }

  if (g_nmeaOk > 50 && g_nmeaBad * 100 > g_nmeaOk)
    Serial.println("  [WARN] >1% NMEA checksum errors -> UART noise / level shifter problem.");
  if (!problem) Serial.println("  Nothing wrong found with the antenna path.");
  Serial.println("==================================================================");
}

// ----------------------------------------------------------------------------
//  Modem control
// ----------------------------------------------------------------------------
static bool modemIsOn() { return digitalRead(PIN_MODEM_STATUS) == LOW; }  // inverted via Q3

static void pulse(int pin, uint32_t ms) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

static bool startModem() {
  Serial.printf("[1] Modem power: STATUS(GPIO8) = %s\n", modemIsOn() ? "ON" : "OFF");
  if (!modemIsOn()) {
    Serial.println("    PWRKEY pulse 1000 ms ...");
    pulse(PIN_MODEM_PWRKEY, MODEM_PWRKEY_PULSE_MS);
    uint32_t t = millis();
    while (!modemIsOn() && millis() - t < 15000) delay(50);
    if (!modemIsOn()) {
      Serial.println("    FAIL: modem did not power on (check VBAT, PWRKEY Q4, STATUS Q3)");
      return false;
    }
  }
  Serial.println("    waiting for AT ...");
  modem.setVerbose(false);
  uint32_t t = millis();
  while (millis() - t < 30000) {
    if (modem.cmd("AT", 500) == AtResult::OK) break;
    delay(300);
  }
  modem.setVerbose(true);
  if (modem.cmd("AT", 1000) != AtResult::OK) {
    Serial.println("    FAIL: no AT reply (UART1 / TXS0102 / baud)");
    return false;
  }
  modem.cmd("ATE0");
  modem.cmd("AT+CMEE=2");
  return true;
}

static void setCellular(int mode) {
  Serial.printf("    cellular radio -> AT+CFUN=%d\n", mode);
  modem.cmd((String("AT+CFUN=") + mode).c_str(), 15000);
  delay(500);
  int now = readCfun();
  Serial.printf("    AT+CFUN? = %d %s\n", now,
                now == 4 ? "(flight mode: cellular RF OFF, GNSS on)" : now == 0 ? "(minimum: RF OFF)"
                : now == 1 ? "(full: cellular ON)" : "");
}

static bool enableNmea() {
  modem.cmd("AT+CGNSSTST=1", 2000);
  uint32_t t = millis();
  String l;
  while (millis() - t < 4000) {
    if (modem.readLine(l, 200, false) && l.startsWith("$")) { processNmea(l); return true; }
  }
  Serial.println("    no NMEA on AT port -> AT+CGNSSPORTSWITCH=0,1");
  modem.cmd("AT+CGNSSPORTSWITCH=0,1", 2000);
  modem.cmd("AT+CGNSSTST=1", 2000);
  t = millis();
  while (millis() - t < 4000) {
    if (modem.readLine(l, 200, false) && l.startsWith("$")) { processNmea(l); return true; }
  }
  return false;
}

static void gnssStart(int kind) {
  static const char* CMD[] = {"AT+CGPSCOLD", "AT+CGPSWARM", "AT+CGPSHOT"};
  static const char* NAME[] = {"cold", "warm", "hot"};
  Serial.printf("\n>>> GNSS %s START (%s) - statistics reset\n", NAME[kind], CMD[kind]);
  if (modem.cmd(CMD[kind], 5000) != AtResult::OK)
    Serial.printf("    WARNING: %s rejected: %s\n", CMD[kind], modem.lastError.c_str());
  resetAll(NAME[kind]);
}

static bool startGnss() {
  String r;
  Serial.println("\n[3] GNSS engine");
  modem.cmd("AT+CGNSSPWR?", 2000, &r);
  if (Modem::field(r, "+CGNSSPWR:").toInt() != 1) {
    String urc;
    if (modem.cmdUrc("AT+CGNSSPWR=1", "+CGNSSPWR: READY", 20000, &urc) != AtResult::OK) {
      modem.cmd("AT+CGNSSPWR?", 2000, &r);
      if (Modem::field(r, "+CGNSSPWR:").toInt() != 1) {
        Serial.println("    FAIL: GNSS engine did not power on (check 1V8_GNSS jumper, pin 97)");
        return false;
      }
    }
  }
  Serial.println("    GNSS engine powered");
  if (CFG_GNSS_MODE) modem.cmd((String("AT+CGNSSMODE=") + CFG_GNSS_MODE).c_str(), 2000);
  if (modem.cmd("AT+CGNSSMODE?", 2000, &r) == AtResult::OK)
    Serial.printf("    constellation mode (CGNSSMODE): %s\n", Modem::field(r, "+CGNSSMODE:").c_str());
  Serial.println("    note: VDD_AUX (pin 99) is not connected on this board -> CVAUXS has no effect;");
  Serial.println("          the antenna is powered by the 100 ohm bias resistor from VBAT.");

  gnssStart(CFG_BOOT_START);
  Serial.println("\n[4] NMEA stream");
  if (!enableNmea()) {
    Serial.println("    FAIL: no NMEA output (AT+CGNSSTST / CGNSSPORTSWITCH)");
    return false;
  }
  Serial.println("    NMEA streaming to the AT port");
  modem.setVerbose(false);  // keep NMEA out of the AT log from now on
  return true;
}

// ----------------------------------------------------------------------------
//  Interactive tests
// ----------------------------------------------------------------------------
static float averageTop4(uint32_t seconds, const char* label) {
  float sum = 0;
  int n = 0;
  for (uint32_t i = 0; i < seconds; i++) {
    pumpFor(1000);
    computeStats();
    if (g_st.trk_all > 0) { sum += g_st.top4; n++; }
    if (i % 10 == 9) Serial.printf("    %s %lu/%lu s  top4 now %.1f\n", label, (unsigned long)i + 1,
                                   (unsigned long)seconds, g_st.top4);
  }
  return n ? sum / n : 0;
}

static void interferenceTest() {
  computeStats();
  if (g_st.trk_all == 0) {
    Serial.println("Interference test needs tracked satellites first (wait until you see signal).");
    return;
  }
  Serial.println("\n>>> INTERFERENCE TEST: 30 s cellular OFF, then 45 s cellular ON (LTE TX bursts)");
  g_jamOff = averageTop4(30, "cellular OFF");
  setCellular(1);
  Serial.println("    (letting the modem register; first 15 s not counted)");
  pumpFor(15000);
  g_jamOn = averageTop4(30, "cellular ON ");
  setCellular(CFG_CFUN_MODE);
  Serial.printf(">>> result: top4 OFF %.1f dB-Hz, ON %.1f dB-Hz, change %+.1f dB\n", g_jamOff, g_jamOn,
                g_jamOn - g_jamOff);
}

static void biasCalc(float vAnt) {
  float vb = readVbat();
  if (vb <= 0) { Serial.println("Could not read VBAT (AT+CBC)"); return; }
  g_antV = vAnt;
  g_vbatAtMeas = vb;
  g_antmA = (vb - vAnt) / CFG_BIAS_R_OHMS * 1000.0f;
  if (g_antmA < 0) g_antmA = 0;
  float pR = (vb - vAnt) * (vb - vAnt) / CFG_BIAS_R_OHMS * 1000.0f;
  Serial.printf("\n>>> BIAS-TEE: VBAT %.3f V (AT+CBC), antenna %.2f V -> drop %.2f V over %.0f ohm\n", vb, vAnt,
                vb - vAnt, CFG_BIAS_R_OHMS);
  Serial.printf("    antenna current %.1f mA, resistor dissipation %.0f mW (0402 max ~62 mW)\n", g_antmA, pR);
  if (g_antmA < CFG_ANT_MIN_MA) Serial.println("    -> almost no current: antenna OPEN / not connected / passive");
  else if (g_antmA > CFG_ANT_MAX_MA || vAnt < 0.5f) Serial.println("    -> SHORT CIRCUIT on coax/antenna");
  else if (vAnt < CFG_ANT_MIN_V) Serial.println("    -> voltage too LOW for most active antennas");
  else Serial.println("    -> OK");
  Serial.println("    Tip: VBAT is measured now; measure the antenna voltage at the same time.");
}

static void passthrough() {
  Serial.println("\n--- AT passthrough ('#q' to quit). NMEA keeps printing. ---");
  String in;
  for (;;) {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n' || ch == '\r') {
        in.trim();
        if (in == "#q") { Serial.println("--- passthrough closed ---"); return; }
        if (in.length()) { Serial1.print(in); Serial1.print('\r'); }
        in = "";
      } else in += ch;
    }
    while (Serial1.available()) Serial.write(Serial1.read());
    delay(1);
  }
}

static void help() {
  Serial.println("\n================ ACTIVE GPS ANTENNA TEST - commands ================");
  Serial.println("  d          diagnosis / verdict now        t   satellite table now");
  Serial.println("  c          cold start (resets stats)      w   warm start     o  hot start");
  Serial.println("  v 2.95     enter measured antenna voltage (coax centre to GND) -> bias check");
  Serial.println("  j          interference test: 30 s cellular OFF vs 30 s ON (turns LTE on briefly)");
  Serial.println("  r          raw NMEA on/off                p   AT passthrough (#q to quit)");
  Serial.println("  f          cellular back to full (AT+CFUN=1) before going back to normal firmware");
  Serial.println("  x          cellular off again (AT+CFUN=" + String(CFG_CFUN_MODE) + ")      h   this help");
  Serial.println("====================================================================");
}

static void command(String in) {
  in.trim();
  if (!in.length()) return;
  char c = in[0];
  if (c == 'h') help();
  else if (c == 'd') diagnose();
  else if (c == 't') printTable();
  else if (c == 'c') gnssStart(0);
  else if (c == 'w') gnssStart(1);
  else if (c == 'o') gnssStart(2);
  else if (c == 'r') { g_raw = !g_raw; Serial.printf("raw NMEA %s\n", g_raw ? "ON" : "OFF"); }
  else if (c == 'j') interferenceTest();
  else if (c == 'p') passthrough();
  else if (c == 'f') setCellular(1);
  else if (c == 'x') setCellular(CFG_CFUN_MODE);
  else if (c == 'v') {
    float v = in.substring(1).toFloat();
    if (v <= 0 || v > 6) Serial.println("usage: v <volts>, e.g. v 2.95");
    else biasCalc(v);
  } else Serial.println("unknown command - 'h' for help");
}

// ----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_MODEM_PWRKEY, OUTPUT); digitalWrite(PIN_MODEM_PWRKEY, LOW);
  pinMode(PIN_MODEM_RESET, OUTPUT);  digitalWrite(PIN_MODEM_RESET, LOW);
  pinMode(PIN_MODEM_DTR, OUTPUT);    digitalWrite(PIN_MODEM_DTR, HIGH);  // keep modem awake
  pinMode(PIN_MODEM_STATUS, INPUT);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 8000) delay(10);
  delay(500);
  Serial1.setRxBufferSize(8192);
  Serial1.begin(MODEM_BAUD, SERIAL_8N1, PIN_MODEM_RX, PIN_MODEM_TX);
  modem.begin(Serial1);

  Serial.println("\n####################################################################");
  Serial.println("#  Fleet Tracker - ACTIVE GPS ANTENNA TEST (cellular radio OFF)     #");
  Serial.println("####################################################################");
  resetAll("cold");

  if (!startModem()) { help(); return; }
  String r;
  if (modem.cmd("AT+CGMM", 2000, &r) == AtResult::OK) Serial.printf("    model: %s\n", Modem::firstLine(r).c_str());
  if (modem.cmd("AT+CGMR", 2000, &r) == AtResult::OK) Serial.printf("    firmware: %s\n", Modem::field(r, "+CGMR:").c_str());
  float vb = readVbat();
  Serial.printf("    VBAT %.3f V -> antenna gets about %.2f V at 10 mA through %.0f ohm\n", vb,
                vb - 0.010f * CFG_BIAS_R_OHMS, CFG_BIAS_R_OHMS);
  if (modem.cmd("AT+CPMUTEMP", 2000, &r) == AtResult::OK)
    Serial.printf("    module temperature %s C\n", Modem::field(r, "+CPMUTEMP:").c_str());

  Serial.println("\n[2] Cellular radio OFF (GPS-only test)");
  setCellular(CFG_CFUN_MODE);

  if (!startGnss()) { help(); return; }
  help();
  Serial.printf("\nLive monitoring: status every %d s, satellite table every %d s, verdict at %d s.\n",
                CFG_DASH_EVERY_S, CFG_TABLE_EVERY_S, CFG_DIAG_AFTER_S);
  Serial.println("Put the antenna OUTDOORS with open sky.");
}

void loop() {
  static String in;
  static uint32_t lastStat = 0, lastDash = 0, lastTable = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') { command(in); in = ""; }
    else in += ch;
  }
  String l;
  while (modem.readLine(l, 0, false)) handleModemLine(l);

  uint32_t now = millis();
  if (now - lastStat >= 1000) { lastStat = now; computeStats(); }
  if (now - lastDash >= CFG_DASH_EVERY_S * 1000UL) { lastDash = now; printDash(); }
  if (now - lastTable >= CFG_TABLE_EVERY_S * 1000UL) { lastTable = now; printTable(); }
  if (CFG_DIAG_AFTER_S && !g_diagDone && elapsedS() >= CFG_DIAG_AFTER_S) {
    g_diagDone = true;
    diagnose();
  }
  delay(2);
}

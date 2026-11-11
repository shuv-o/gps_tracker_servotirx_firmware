// =============================================================================
//  Fleet Tracker - SIMCom A7672SA-FASE device test
//  Host: ESP32-S3-WROOM-1U-N16R8, console on native USB (Serial)
//
//  Steps:
//   1  Modem power-on / boot        ("SIMCOM STARTED")
//   2  Modem identity & health      (model, FW, IMEI, VBAT, temperature)
//   3  SIM card
//   4  Radio signal (GSM/LTE RF)
//   5  Network registration
//   6  Network status (operator, band, cell, RSRP/RSRQ, IP, time)
//   7  Packet data (PDP / IP)
//   8  Internet (DNS + HTTP)
//   9  Internet speed - 4G LTE
//  10  Internet speed - 2G GSM/EDGE
//  11  GNSS power-on
//  12  GNSS cold start (TTFF)
//  13  GNSS positioning
//  14  GNSS hot start (TTFF)
//  15  GNSS status (satellites / C/N0)
//  -> Summary + all failure messages
//
//  Serial commands (after the run):
//   a        run all steps        1..15  run one step
//   n        network status now   s      print summary
//   p        AT passthrough (#q to exit)   h  help
// =============================================================================
#include <Arduino.h>
#include <math.h>
#include "board_pins.h"
#include "test_config.h"
#include "modem_at.h"

static Modem modem;
static volatile uint32_t g_riPulses = 0;

// ----------------------------------------------------------------------------
//  Test bookkeeping
// ----------------------------------------------------------------------------
enum Status { ST_NOTRUN, ST_PASS, ST_WARN, ST_FAIL, ST_SKIP };

struct TestCtx {
  Status st = ST_PASS;
  String detail;   // one-line result
  String failMsg;  // what went wrong
  String hint;     // what to check
};

struct TestResult {
  Status st = ST_NOTRUN;
  String detail, failMsg, hint;
  uint32_t ms = 0;
};

typedef void (*TestFn)(TestCtx&);
struct TestDef { const char* name; TestFn fn; };

static void pass(TestCtx& c, const String& d) { c.st = ST_PASS; c.detail = d; }
static void warn(TestCtx& c, const String& d, const String& msg, const String& hint) {
  c.st = ST_WARN; c.detail = d; c.failMsg = msg; c.hint = hint;
}
static void fail(TestCtx& c, const String& msg, const String& hint) {
  c.st = ST_FAIL; c.detail = msg; c.failMsg = msg; c.hint = hint;
}
static void skip(TestCtx& c, const String& why) {
  c.st = ST_SKIP; c.detail = why; c.failMsg = why;
}

static const char* stName(Status s) {
  switch (s) {
    case ST_PASS: return "PASS";
    case ST_WARN: return "WARN";
    case ST_FAIL: return "FAIL";
    case ST_SKIP: return "SKIP";
    default:      return "----";
  }
}

// ----------------------------------------------------------------------------
//  Shared state between steps
// ----------------------------------------------------------------------------
struct GnssFix {
  bool valid = false;
  int mode = 0;          // 2 = 2D, 3 = 3D
  int svUsed = 0;
  double lat = 0, lon = 0;
  float alt = 0, speedKmh = 0, course = 0, pdop = 0, hdop = 0, vdop = 0;
  String date, utc;
};

struct State {
  bool atOk = false;
  bool simReady = false;
  bool signalOk = false;
  bool registered = false;
  bool dataUp = false;
  bool internetOk = false;
  bool useHttps = false;      // carrier intercepts plain HTTP -> use HTTPS
  bool gnssOn = false;
  bool gnssFixed = false;
  GnssFix fix;
} g;

// ----------------------------------------------------------------------------
//  Output helpers
// ----------------------------------------------------------------------------
static void info(const char* key, const String& val) {
  Serial.printf("    - %-24s: %s\n", key, val.c_str());
}
static void note(const String& s) { Serial.printf("    * %s\n", s.c_str()); }

static void led(uint8_t r, uint8_t gg, uint8_t b) { neopixelWrite(PIN_STATUS_LED, r, gg, b); }

static String fmtMs(uint32_t ms) {
  char b[24];
  if (ms < 1000) snprintf(b, sizeof b, "%lu ms", (unsigned long)ms);
  else snprintf(b, sizeof b, "%.1f s", ms / 1000.0);
  return b;
}

// ----------------------------------------------------------------------------
//  Small parsing helpers
// ----------------------------------------------------------------------------
static int splitCsv(const String& s, String* out, int maxOut) {
  int n = 0, start = 0;
  for (int i = 0; i <= (int)s.length() && n < maxOut; i++) {
    if (i == (int)s.length() || s[i] == ',') {
      out[n] = s.substring(start, i);
      out[n].trim();
      n++;
      start = i + 1;
    }
  }
  return n;
}

static String unquote(String s) {
  s.trim();
  if (s.startsWith("\"")) s = s.substring(1);
  if (s.endsWith("\"")) s = s.substring(0, s.length() - 1);
  return s;
}

static bool modemIsOn() { return digitalRead(PIN_MODEM_STATUS) == LOW; }  // inverted by Q3

static void pulse(int pin, uint32_t ms) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

// stat from +CREG/+CGREG/+CEREG: "<n>,<stat>[,...]"  -> stat, or -1
static int regStat(const char* cmd, const char* prefix) {
  String r;
  if (modem.cmd(cmd, 3000, &r) != AtResult::OK) return -1;
  String f[6];
  int n = splitCsv(Modem::field(r, prefix), f, 6);
  return n >= 2 ? f[1].toInt() : -1;
}
static bool isRegistered(int s) { return s == 1 || s == 5; }

static const char* regText(int s) {
  switch (s) {
    case 0: return "not registered, not searching";
    case 1: return "registered (home)";
    case 2: return "searching...";
    case 3: return "registration DENIED";
    case 4: return "unknown";
    case 5: return "registered (roaming)";
    default: return "no answer";
  }
}

// "LTE", "GSM", "NO SERVICE", ...
static String currentRat(String* full = nullptr) {
  String r;
  if (modem.cmd("AT+CPSI?", 3000, &r) != AtResult::OK) return "?";
  String v = Modem::field(r, "+CPSI:");
  if (full) *full = v;
  int c = v.indexOf(',');
  return c > 0 ? v.substring(0, c) : v;
}

static int csqRssi() {
  String r;
  if (modem.cmd("AT+CSQ", 2000, &r) != AtResult::OK) return -1;
  return Modem::field(r, "+CSQ:").toInt();
}

static String httpStatusText(int code) {
  if (code >= 200 && code < 300) return "OK";
  if (code >= 300 && code < 400) return "redirect";
  if (code >= 400 && code < 500) return "client error (URL / not found)";
  if (code >= 500 && code < 600) return "server error";
  switch (code) {   // SIMCom A76xx HTTP(S) result codes
    case 600: return "not HTTP PDU";
    case 601: return "network error";
    case 602: return "no memory (file too large for modem buffer)";
    case 603: return "DNS error";
    case 604: return "stack busy";
    case 701: return "alert state";
    case 702: return "unknown error";
    case 703: return "busy";
    case 704: return "connection closed";
    case 705: return "timeout";
    case 706: return "socket send/receive failed";
    case 707: return "file/memory error";
    case 708: return "invalid parameter";
    case 709: return "network error";
    case 711: return "wrong state";
    case 712: return "failed to create socket";
    case 713: return "DNS lookup failed";
    case 714: return "TCP connect failed";
    case 715: return "TLS handshake failed";
    case 717: return "no network";
    case 718: return "send timeout";
    default:  return "unknown code";
  }
}

static String g_lastRedirect;   // Location header of the last 3xx reply

static bool sslReady = false;
static void setupSsl() {
  if (sslReady) return;
  modem.cmd("AT+CSSLCFG=\"sslversion\",0,4", 2000);   // all TLS versions
  modem.cmd("AT+CSSLCFG=\"authmode\",0,0", 2000);     // no certificate check (test only)
  modem.cmd("AT+CSSLCFG=\"enableSNI\",0,1", 2000);
  sslReady = true;
}

// "http://x" -> "https://x" when the carrier intercepts plain HTTP
static String effUrl(const char* url) {
  String u = url;
  if (g.useHttps && u.startsWith("http://")) u = "https://" + u.substring(7);
  return u;
}

// HTTP GET into the modem's buffer. Time measured from command to +HTTPACTION URC,
// i.e. the over-the-air download time (UART is not involved in the transfer).
static bool httpGet(const char* url, int& status, long& len, uint32_t& ms,
                    uint32_t timeoutMs, String& err) {
  status = -1; len = 0; ms = 0;
  g_lastRedirect = "";
  String u = effUrl(url);
  bool https = u.startsWith("https://");
  if (https) setupSsl();
  modem.cmd("AT+HTTPTERM", 2000);  // ignore result (may not be initialised)
  if (modem.cmd("AT+HTTPINIT", 5000) != AtResult::OK) {
    err = "AT+HTTPINIT failed: " + modem.lastError;
    return false;
  }
  String c = "AT+HTTPPARA=\"URL\",\"" + u + "\"";
  if (modem.cmd(c.c_str(), 3000) != AtResult::OK) {
    err = "AT+HTTPPARA URL failed: " + modem.lastError;
    modem.cmd("AT+HTTPTERM", 2000);
    return false;
  }
  if (https) modem.cmd("AT+HTTPPARA=\"SSLCFG\",0", 2000);  // ignore if unsupported
  String urc;
  uint32_t t0 = millis();
  AtResult r = modem.cmdUrc("AT+HTTPACTION=0", "+HTTPACTION:", timeoutMs, &urc);
  ms = millis() - t0;
  if (r != AtResult::OK) {
    modem.cmd("AT+HTTPTERM", 2000);
    err = "AT+HTTPACTION: " + modem.lastError;
    return false;
  }
  String f[4];
  int n = splitCsv(Modem::field(urc, "+HTTPACTION:"), f, 4);
  if (n >= 3) { status = f[1].toInt(); len = f[2].toInt(); }

  if (status >= 300 && status < 400) {   // read headers to find out where we are sent
    String h;
    if (modem.cmd("AT+HTTPHEAD", 5000, &h) == AtResult::OK) {
      int p = h.indexOf("ocation:");      // Location: / location:
      if (p > 0) {
        int e = h.indexOf('\n', p);
        g_lastRedirect = h.substring(p + 8, e < 0 ? h.length() : e);
        g_lastRedirect.trim();
      }
    }
  }
  modem.cmd("AT+HTTPTERM", 2000);

  if (n < 3) { err = "bad +HTTPACTION reply: " + urc; return false; }
  if (status < 200 || status >= 400) {
    err = "HTTP " + String(status) + " (" + httpStatusText(status) + ")";
    return false;
  }
  if (status >= 300) {
    err = "HTTP " + String(status) + " redirect" +
          (g_lastRedirect.length() ? " -> " + g_lastRedirect : String(""));
  }
  return true;
}

// ----------------------------------------------------------------------------
//  Speed test
// ----------------------------------------------------------------------------
struct SpeedFile { const char* url; long bytes; };   // expected size, used to validate
struct SpeedServer {
  const char* name;
  const char* small;      // latency baseline (nullptr = none)
  SpeedFile big4g;
  SpeedFile big4gAlt;     // used if big4g fails (e.g. modem buffer limit)
  SpeedFile big2g;
};

// Plain-HTTP servers checked 2026-11-05 (speedtest.tele2.net no longer answers).
static const SpeedServer SPEED_SERVERS[] = {
  {"Cloudflare", "http://speed.cloudflare.com/__down?bytes=1000",
     {"http://speed.cloudflare.com/__down?bytes=10000000", 10000000},
     {"http://speed.cloudflare.com/__down?bytes=2000000",   2000000},
     {"http://speed.cloudflare.com/__down?bytes=200000",     200000}},
  {"CacheFly", nullptr,
     {"http://cachefly.cachefly.net/10mb.test", 10485760},
     {"http://cachefly.cachefly.net/1mb.test",   1048576},
     {"http://cachefly.cachefly.net/1mb.test",   1048576}},
  {"httpbin", "http://httpbin.org/bytes/16",
     {"http://httpbin.org/bytes/102400", 102400},
     {nullptr, 0},
     {"http://httpbin.org/bytes/102400", 102400}},
};

// A redirect, captive portal or error page also "downloads" in a few hundred bytes,
// which used to be reported as ~0 Mbit/s. Only accept the real test file.
static bool downloadValid(const SpeedFile& f, int st, long len, String& e) {
  if (st != 200) {
    e = "HTTP " + String(st) + " (" + httpStatusText(st) + ") instead of 200 - not the test file";
    return false;
  }
  if (len < f.bytes * 9 / 10) {
    e = "server sent only " + String(len) + " of " + String(f.bytes) + " bytes (redirect / blocked page?)";
    return false;
  }
  return true;
}

static bool runSpeedTest(bool is2g, double& bestKbps, String& err) {
  bestKbps = 0;
  const uint32_t bigTimeout = is2g ? 240000 : 120000;
  const int runs = is2g ? 1 : CFG_SPEED_RUNS_4G;

  if (g.useHttps) note("Using HTTPS (carrier intercepts plain HTTP); TLS may lower the result a little.");
  for (const SpeedServer& s : SPEED_SERVERS) {
    note(String("Server: ") + s.name);
    int st; long len; uint32_t tSmall = 0; String e;
    if (s.small) {
      bool ok = httpGet(s.small, st, len, tSmall, 60000, e);
      if (!ok || st != 200) {
        note("  baseline request failed: " + (ok ? "HTTP " + String(st) : e) +
             (g_lastRedirect.length() ? " -> " + g_lastRedirect : String("")));
        err = String(s.name) + ": " + (ok ? "HTTP " + String(st) : e);
        continue;
      }
      info("Request latency", fmtMs(tSmall) + " (DNS + connect + server, small file)");
    }

    const SpeedFile* files[2] = { is2g ? &s.big2g : &s.big4g, is2g ? nullptr : &s.big4gAlt };
    for (const SpeedFile* f : files) {
      if (!f || !f->url) continue;
      for (int i = 0; i < runs; i++) {
        uint32_t tBig;
        bool ok = httpGet(f->url, st, len, tBig, bigTimeout, e);
        info((String("Run ") + (i + 1) + " reply").c_str(),
             "HTTP " + String(st) + ", " + String(len) + " bytes in " + fmtMs(tBig));
        if (!ok || !downloadValid(*f, st, len, e)) {
          note("  rejected: " + e + (g_lastRedirect.length() ? " -> " + g_lastRedirect : String("")));
          err = String(s.name) + ": " + e;
          break;
        }
        uint32_t payloadMs = (tBig > tSmall + 100) ? tBig - tSmall : tBig;
        double kbps = (len * 8.0) / payloadMs;  // bits per ms == kbit/s
        char b[120];
        snprintf(b, sizeof b, "%.2f Mbit/s (%.0f kbit/s, transfer %s)",
                 kbps / 1000.0, kbps, fmtMs(payloadMs).c_str());
        info((String("Run ") + (i + 1) + " speed").c_str(), b);
        if (kbps > bestKbps) bestKbps = kbps;
      }
      if (bestKbps > 0) return true;
    }
  }
  return bestKbps > 0;
}

// ----------------------------------------------------------------------------
//  GNSS helpers
// ----------------------------------------------------------------------------
static double nmeaToDeg(double v, double limit) {
  // A76xx firmware outputs either ddmm.mmmm or decimal degrees - detect which.
  if (fabs(v) > limit) {
    int deg = (int)(v / 100);
    double min = v - deg * 100.0;
    return deg + min / 60.0;
  }
  return v;
}

// +CGNSSINFO: <mode>,<SVs...>,<lat>,<N/S>,<lon>,<E/W>,<date>,<UTC>,<alt>,<speed>,<course>,<PDOP>,<HDOP>,<VDOP>
static bool parseCgnssinfo(const String& resp, GnssFix& f) {
  f = GnssFix();
  String v = Modem::field(resp, "+CGNSSINFO:");
  if (v.length() == 0) return false;
  String t[24];
  int n = splitCsv(v, t, 24);
  int ns = -1;
  for (int i = 2; i < n; i++)
    if (t[i] == "N" || t[i] == "S") { ns = i; break; }
  if (ns < 2 || t[ns - 1].length() == 0) return false;

  f.mode = t[0].toInt();
  for (int i = 1; i < ns - 1; i++) f.svUsed += t[i].toInt();
  f.lat = nmeaToDeg(t[ns - 1].toDouble(), 90.0) * (t[ns] == "S" ? -1 : 1);
  if (ns + 2 < n) f.lon = nmeaToDeg(t[ns + 1].toDouble(), 180.0) * (t[ns + 2] == "W" ? -1 : 1);
  if (ns + 3 < n) f.date = t[ns + 3];
  if (ns + 4 < n) f.utc = t[ns + 4];
  if (ns + 5 < n) f.alt = t[ns + 5].toFloat();
  if (ns + 6 < n) f.speedKmh = t[ns + 6].toFloat() * 1.852f;  // knots -> km/h
  if (ns + 7 < n) f.course = t[ns + 7].toFloat();
  if (ns + 8 < n) f.pdop = t[ns + 8].toFloat();
  if (ns + 9 < n) f.hdop = t[ns + 9].toFloat();
  if (ns + 10 < n) f.vdop = t[ns + 10].toFloat();
  f.valid = (f.lat != 0 || f.lon != 0);
  return f.valid;
}

static bool readFix(GnssFix& f) {
  String r;
  if (modem.cmd("AT+CGNSSINFO", 3000, &r) != AtResult::OK) return false;
  return parseCgnssinfo(r, f);
}

// Poll for a fix after a cold/hot start. Returns TTFF in ms, or 0 on timeout.
static uint32_t waitForFix(uint32_t timeoutS, GnssFix& f) {
  bool v = modem.verbose();
  modem.setVerbose(false);  // avoid printing hundreds of polls
  uint32_t t0 = millis(), lastPrint = 0;
  uint32_t ttff = 0;
  while (millis() - t0 < timeoutS * 1000UL) {
    if (readFix(f)) { ttff = millis() - t0; break; }
    uint32_t el = (millis() - t0) / 1000;
    if (el - lastPrint >= 10) {
      lastPrint = el;
      Serial.printf("    ... %lu s / %lu s, no fix yet\n", (unsigned long)el, (unsigned long)timeoutS);
    }
    delay(1000);
  }
  modem.setVerbose(v);
  return ttff;
}

// --- Sky view from NMEA GSV sentences ---
enum { C_GPS, C_GLO, C_GAL, C_BDS, C_QZS, C_N };
static const char* C_NAMES[C_N] = {"GPS", "GLONASS", "Galileo", "BeiDou", "QZSS"};

struct SkyView {
  int nmeaLines = 0;
  int inView[C_N] = {0};
  uint8_t snr[C_N][200];
  String rmcStatus;  // "A" valid / "V" void
};

static int talkerIdx(const String& s) {
  if (s.startsWith("$GP")) return C_GPS;
  if (s.startsWith("$GL")) return C_GLO;
  if (s.startsWith("$GA")) return C_GAL;
  if (s.startsWith("$GB") || s.startsWith("$BD")) return C_BDS;
  if (s.startsWith("$GQ")) return C_QZS;
  return -1;
}

static void captureSky(SkyView& sv, uint32_t seconds) {
  memset(sv.snr, 0, sizeof sv.snr);
  bool v = modem.verbose();
  modem.cmd("AT+CGNSSTST=1", 2000);  // stream NMEA to this AT port
  uint32_t t0 = millis();
  String line;
  while (millis() - t0 < seconds * 1000UL) {
    if (!modem.readLine(line, 200, false)) continue;
    if (!line.startsWith("$")) continue;
    sv.nmeaLines++;
    int star = line.indexOf('*');
    String body = star > 0 ? line.substring(0, star) : line;
    String f[24];
    int n = splitCsv(body, f, 24);
    if (body.substring(3, 6) == "RMC" && n > 2) sv.rmcStatus = f[2];
    if (body.substring(3, 6) != "GSV" || n < 4) continue;
    int c = talkerIdx(body);
    if (c < 0) continue;
    sv.inView[c] = f[3].toInt();
    for (int k = 0; 4 + 4 * k + 3 < n; k++) {
      int prn = f[4 + 4 * k].toInt();
      int snr = f[4 + 4 * k + 3].toInt();
      if (prn > 0 && prn < 200 && snr > sv.snr[c][prn]) sv.snr[c][prn] = snr;
    }
  }
  modem.setVerbose(false);
  modem.cmd("AT+CGNSSTST=0", 2000);
  delay(300);
  modem.drain();
  modem.setVerbose(v);
}

// ----------------------------------------------------------------------------
//  Network status report (step 6 and the 'n' command)
// ----------------------------------------------------------------------------
struct NetInfo {
  bool lte = false;
  int rsrp = 0, rsrq = 0;       // dBm / dB, 0 = unknown
  int csq = 99;
  String op, rat;
};

static const char* qualRsrp(int dbm) {
  return dbm >= -80 ? "excellent" : dbm >= -90 ? "good" : dbm >= -100 ? "fair" : "poor";
}
static const char* qualRsrq(int db) {
  return db >= -10 ? "excellent" : db >= -15 ? "good" : db >= -20 ? "fair" : "poor";
}

static NetInfo printNetworkStatus() {
  NetInfo ni;
  String r;
  Serial.println("    ---------------- NETWORK STATUS ----------------");

  // Operator (long name) + access technology
  modem.cmd("AT+COPS=3,0", 2000);  // report operator as long alphanumeric name
  if (modem.cmd("AT+COPS?", 5000, &r) == AtResult::OK) {
    String f[4];
    int n = splitCsv(Modem::field(r, "+COPS:"), f, 4);
    ni.op = n >= 3 ? unquote(f[2]) : "(none)";
    info("Operator", ni.op);
    if (n >= 4) {
      int act = f[3].toInt();
      info("Access technology", act == 7 ? "LTE (4G)" : act == 0 ? "GSM (2G)" :
                                 act == 3 ? "EDGE (2G)" : act == 2 ? "UTRAN (3G)" : String(act));
    }
  }

  if (modem.cmd("AT+CNMP?", 2000, &r) == AtResult::OK) {
    int m = Modem::field(r, "+CNMP:").toInt();
    info("Mode preference (CNMP)", String(m) + (m == 2 ? " = automatic" : m == 13 ? " = GSM only" :
                                     m == 38 ? " = LTE only" : m == 51 ? " = GSM + LTE" : ""));
  }

  int ce = regStat("AT+CEREG?", "+CEREG:");
  int cr = regStat("AT+CREG?", "+CREG:");
  int cg = regStat("AT+CGREG?", "+CGREG:");
  info("Registration LTE/GSM/GPRS", String(ce) + " / " + cr + " / " + cg + "  (" +
                                    regText(isRegistered(ce) ? ce : cr) + ")");

  // Serving cell
  String cpsi;
  ni.rat = currentRat(&cpsi);
  String t[16];
  int n = splitCsv(cpsi, t, 16);
  if (ni.rat == "LTE" && n >= 10) {
    ni.lte = true;
    info("System / state", t[0] + ", " + t[1]);
    info("MCC-MNC", t[2]);
    info("Tracking area (TAC)", t[3]);
    unsigned long cid = strtoul(t[4].c_str(), nullptr, 10);
    char b[80];
    snprintf(b, sizeof b, "%lu (0x%lX)  eNB %lu, sector %lu", cid, cid, cid >> 8, cid & 0xFF);
    info("Cell ID", b);
    info("Physical cell ID (PCI)", t[5]);
    info("Band", t[6]);
    info("EARFCN (DL channel)", t[7]);
    info("DL / UL bandwidth code", t[8] + " / " + t[9]);
    if (n >= 14)
      info("CPSI RSRQ/RSRP/RSSI/SINR", t[10] + " / " + t[11] + " / " + t[12] + " / " + t[13] + "  (raw)");
  } else {
    info("Serving cell (CPSI)", cpsi.length() ? cpsi : String("no answer"));
  }

  // Standard 3GPP measurements: +CESQ: rxlev,ber,rscp,ecno,rsrq,rsrp
  if (modem.cmd("AT+CESQ", 2000, &r) == AtResult::OK) {
    String f[6];
    if (splitCsv(Modem::field(r, "+CESQ:"), f, 6) == 6) {
      int rxlev = f[0].toInt(), rsrqI = f[4].toInt(), rsrpI = f[5].toInt();
      if (rsrpI != 255) {
        ni.rsrp = -141 + rsrpI;
        info("RSRP (signal power)", String(ni.rsrp) + " dBm  " + qualRsrp(ni.rsrp));
      }
      if (rsrqI != 255) {
        ni.rsrq = (int)lround(-20 + rsrqI * 0.5);
        info("RSRQ (signal quality)", String(-20 + rsrqI * 0.5, 1) + " dB  " + qualRsrq(ni.rsrq));
      }
      if (rxlev != 99) info("GSM RX level", String(-111 + rxlev) + " dBm");
    }
  }

  ni.csq = csqRssi();
  if (ni.csq >= 0 && ni.csq != 99)
    info("CSQ (RSSI)", String(ni.csq) + " = " + String(-113 + 2 * ni.csq) + " dBm");
  else
    info("CSQ (RSSI)", "99 = no signal");

  // Data session
  if (modem.cmd("AT+CGDCONT?", 3000, &r) == AtResult::OK) {
    String f[4];
    splitCsv(Modem::field(r, "+CGDCONT:"), f, 4);
    info("APN / PDP type", unquote(f[2]) + " / " + unquote(f[1]));
  }
  if (modem.cmd("AT+CGATT?", 3000, &r) == AtResult::OK)
    info("Packet attach", Modem::field(r, "+CGATT:") == "1" ? "attached" : "detached");
  if (modem.cmd("AT+CGPADDR=1", 3000, &r) == AtResult::OK) {
    String f[3];
    int k = splitCsv(Modem::field(r, "+CGPADDR:"), f, 3);
    info("IPv4 address", k >= 2 ? unquote(f[1]) : String("none"));
    if (k >= 3) info("IPv6 address", unquote(f[2]));
  }

  // Misc
  if (modem.cmd("AT+CCLK?", 2000, &r) == AtResult::OK)
    info("Network time (CCLK)", unquote(Modem::field(r, "+CCLK:")));
  if (modem.cmd("AT+CNUM", 3000, &r) == AtResult::OK) {
    String f[3];
    int k = splitCsv(Modem::field(r, "+CNUM:"), f, 3);
    info("Own number (CNUM)", k >= 2 && unquote(f[1]).length() ? unquote(f[1]) : String("not stored on SIM"));
  }
  Serial.println("    ------------------------------------------------");
  return ni;
}

// =============================================================================
//  TEST STEPS
// =============================================================================

// 1 ---------------------------------------------------------------------------
static void t_powerOn(TestCtx& c) {
  uint32_t t0 = millis();
  info("STATUS pin (GPIO8)", modemIsOn() ? "LOW -> modem ON" : "HIGH -> modem OFF");

  if (!modemIsOn()) {
    note("Modem is OFF -> PWRKEY pulse 1000 ms on GPIO7");
    pulse(PIN_MODEM_PWRKEY, MODEM_PWRKEY_PULSE_MS);
    uint32_t ts = millis();
    while (!modemIsOn() && millis() - ts < 15000) delay(50);
    if (!modemIsOn()) {
      return fail(c, "Modem did not power on: STATUS stayed OFF 15 s after PWRKEY pulse",
                  "Check Battery_Backed/VBAT at modem (>3.4 V), Q1 + GPIO7 -> PWRKEY pin 1, "
                  "Q3 + STATUS pin 66 -> GPIO8, header H1 seating");
    }
    info("STATUS went ON after", fmtMs(millis() - ts));
  }

  // Wait for the AT interface (UART comes up a few seconds after STATUS)
  note("Waiting for AT interface...");
  bool ok = false;
  bool v = modem.verbose();
  modem.setVerbose(false);
  uint32_t ta = millis();
  while (millis() - ta < 30000) {
    if (modem.cmd("AT", 500) == AtResult::OK) { ok = true; break; }
    delay(300);
  }
  modem.setVerbose(v);

  if (!ok) {
    return fail(c, "Modem STATUS is ON but no reply to AT within 30 s",
                "Check UART1: GPIO17(TX)->SIM_RXD pin 10, GPIO18(RX)<-SIM_TXD pin 9, "
                "TXS0102 VCCA 1.8 V (modem pin 15), baud 115200 (AT+IPR). Try hard reset.");
  }
  modem.cmd("ATE0");
  modem.cmd("AT+CMEE=2");   // verbose error text
  modem.cmd("AT+CNMI=2,1,0,0,0");
  modem.cmd("AT+COPS=3,0");  // operator as name, not number
  g.atOk = true;
  pass(c, "SIMCOM STARTED - AT ready in " + fmtMs(millis() - t0));
}

// 2 ---------------------------------------------------------------------------
static void t_identity(TestCtx& c) {
  if (!g.atOk) return skip(c, "modem not responding (step 1)");
  String r;
  String model = "?", fw = "?", imei = "?";
  if (modem.cmd("AT+CGMM", 2000, &r) == AtResult::OK) model = Modem::firstLine(r);
  if (modem.cmd("AT+CGMR", 2000, &r) == AtResult::OK) fw = Modem::field(r, "+CGMR:");
  if (modem.cmd("AT+SIMEI?", 2000, &r) == AtResult::OK) imei = Modem::field(r, "+SIMEI:");
  else if (modem.cmd("AT+CGSN", 2000, &r) == AtResult::OK) imei = Modem::firstLine(r);
  info("Model", model);
  info("Firmware", fw);
  info("IMEI", imei);

  float vbat = 0;
  if (modem.cmd("AT+CBC", 2000, &r) == AtResult::OK) {
    vbat = Modem::field(r, "+CBC:").toFloat();
    info("Supply voltage (VBAT)", String(vbat, 3) + " V");
  }
  if (modem.cmd("AT+CPMUTEMP", 2000, &r) == AtResult::OK)
    info("Module temperature", Modem::field(r, "+CPMUTEMP:") + " C");

  String d = model + " FW " + fw;
  if (vbat > 0 && vbat < 3.5f)
    return warn(c, d, "Low modem supply " + String(vbat, 2) + " V",
                "TX bursts may brown out the modem; check BQ25616J SYS rail / LiPo charge");
  if (model.indexOf("A767") < 0)
    return warn(c, d, "Unexpected model '" + model + "'", "Expected A7672SA-FASE");
  pass(c, d + ", VBAT " + String(vbat, 2) + " V");
}

// 3 ---------------------------------------------------------------------------
static void t_sim(TestCtx& c) {
  if (!g.atOk) return skip(c, "modem not responding (step 1)");
  String r, cpin;
  for (int i = 0; i < 10; i++) {
    AtResult res = modem.cmd("AT+CPIN?", 3000, &r);
    if (res == AtResult::OK) { cpin = Modem::field(r, "+CPIN:"); break; }
    cpin = modem.lastError;
    if (cpin.indexOf("not inserted") >= 0) break;
    delay(1000);
  }
  info("SIM state", cpin);

  if (cpin == "SIM PIN" && strlen(CFG_SIM_PIN)) {
    String cmd = String("AT+CPIN=\"") + CFG_SIM_PIN + "\"";
    if (modem.cmd(cmd.c_str(), 5000) != AtResult::OK)
      return fail(c, "SIM PIN rejected: " + modem.lastError, "Check CFG_SIM_PIN (3 wrong tries locks the SIM)");
    delay(2000);
    modem.cmd("AT+CPIN?", 3000, &r);
    cpin = Modem::field(r, "+CPIN:");
  }

  if (cpin != "READY") {
    if (cpin.indexOf("not inserted") >= 0)
      return fail(c, "SIM card not detected (" + cpin + ")",
                  "Re-seat nano-SIM, check CARD1 socket solder, SIM_VDD, ESDA6V1W5 for shorts");
    if (cpin == "SIM PIN")
      return fail(c, "SIM is PIN locked", "Set CFG_SIM_PIN in test_config.h or disable PIN on a phone");
    if (cpin == "SIM PUK")
      return fail(c, "SIM is PUK locked", "Unlock SIM with PUK in a phone");
    return fail(c, "SIM not ready: " + cpin,
                "Check SIM_CLK/SIM_DATA/SIM_RST 22R resistors, SIM_VDD, try another SIM");
  }

  String iccid = "?", imsi = "?";
  if (modem.cmd("AT+CICCID", 2000, &r) == AtResult::OK) iccid = Modem::field(r, "+ICCID:");
  if (modem.cmd("AT+CIMI", 2000, &r) == AtResult::OK) imsi = Modem::firstLine(r);
  info("ICCID", iccid);
  info("IMSI", imsi);
  g.simReady = true;
  pass(c, "SIM READY, ICCID " + iccid);
}

// 4 ---------------------------------------------------------------------------
static void t_signal(TestCtx& c) {
  if (!g.atOk) return skip(c, "modem not responding (step 1)");
  String r;
  if (modem.cmd("AT+CFUN?", 2000, &r) == AtResult::OK && Modem::field(r, "+CFUN:").toInt() != 1) {
    note("Radio is off -> AT+CFUN=1");
    modem.cmd("AT+CFUN=1", 10000);
  }

  int rssi = 99;
  uint32_t t0 = millis();
  while (millis() - t0 < CFG_SIGNAL_TIMEOUT_S * 1000UL) {
    rssi = csqRssi();
    if (rssi > 0 && rssi != 99) break;
    delay(2000);
  }
  if (rssi <= 0 || rssi == 99)
    return fail(c, "No RF signal (CSQ 99) after " + String(CFG_SIGNAL_TIMEOUT_S) + " s",
                "Check LTE antenna on U.FL, antenna matching network, CFUN=1, band coverage");

  int dbm = -113 + 2 * rssi;
  const char* q = rssi >= 20 ? "excellent" : rssi >= 15 ? "good" : rssi >= 10 ? "fair" : "poor";
  info("CSQ", String(rssi) + " (" + dbm + " dBm, " + q + ")");
  String full;
  String rat = currentRat(&full);
  info("Radio system (CPSI)", full);
  g.signalOk = true;

  String d = "RF OK: CSQ " + String(rssi) + " / " + dbm + " dBm on " + rat;
  if (rssi < 10)
    return warn(c, d, "Weak signal (CSQ < 10)", "Check antenna placement / cable; data speed will be low");
  pass(c, d);
}

// 5 ---------------------------------------------------------------------------
static void t_register(TestCtx& c) {
  if (!g.simReady) return skip(c, "SIM not ready (step 3)");
  int ce = -1, cr = -1, cg = -1;
  uint32_t t0 = millis();
  bool v = modem.verbose();
  modem.setVerbose(false);
  while (millis() - t0 < CFG_NET_REG_TIMEOUT_S * 1000UL) {
    ce = regStat("AT+CEREG?", "+CEREG:");
    cr = regStat("AT+CREG?", "+CREG:");
    cg = regStat("AT+CGREG?", "+CGREG:");
    if (isRegistered(ce) || isRegistered(cr) || isRegistered(cg) || ce == 3 || cr == 3) break;
    Serial.printf("    ... searching network (%lu s)\n", (unsigned long)((millis() - t0) / 1000));
    delay(3000);
  }
  modem.setVerbose(v);
  info("LTE  (CEREG)", String(ce) + " = " + regText(ce));
  info("GSM  (CREG)", String(cr) + " = " + regText(cr));
  info("GPRS (CGREG)", String(cg) + " = " + regText(cg));

  if (ce == 3 || cr == 3)
    return fail(c, "Network registration DENIED",
                "SIM not provisioned / blocked, IMEI barred, or no roaming agreement");
  if (!(isRegistered(ce) || isRegistered(cr) || isRegistered(cg)))
    return fail(c, "Not registered after " + String(CFG_NET_REG_TIMEOUT_S) + " s",
                "No coverage on supported bands, weak antenna, or operator issue (AT+COPS=?)");

  String r, op = "?";
  if (modem.cmd("AT+COPS?", 5000, &r) == AtResult::OK) {
    String f[4];
    int n = splitCsv(Modem::field(r, "+COPS:"), f, 4);
    if (n >= 3) op = unquote(f[2]);
    if (n >= 4) {
      int act = f[3].toInt();
      info("Access technology", act == 7 ? "LTE (4G)" : act == 0 ? "GSM (2G)" : act == 3 ? "EDGE (2G)" : String(act));
    }
  }
  info("Operator", op);
  String rat = currentRat();
  g.registered = true;
  pass(c, String("NETWORK CONNECTED: ") + op + " on " + rat +
          (isRegistered(ce) && ce == 5 ? " (roaming)" : ""));
}

// 6 ---------------------------------------------------------------------------
static void t_netinfo(TestCtx& c) {
  if (!g.registered) return skip(c, "not registered (step 5)");
  NetInfo ni = printNetworkStatus();
  String d = ni.op + " " + ni.rat;
  if (ni.rsrp) d += ", RSRP " + String(ni.rsrp) + " dBm";
  if (ni.rsrq) d += ", RSRQ " + String(ni.rsrq) + " dB";
  if (ni.lte && ni.rsrp && ni.rsrp < -100)
    return warn(c, d, "Weak LTE signal (RSRP < -100 dBm)",
                "Expect low speed; check LTE antenna, U.FL cable and placement");
  if (ni.lte && ni.rsrq && ni.rsrq < -15)
    return warn(c, d, "Poor LTE quality (RSRQ < -15 dB)",
                "Cell is loaded or interference is high; speed will be limited");
  pass(c, d);
}

// 7 ---------------------------------------------------------------------------
static void t_packet(TestCtx& c) {
  if (!g.registered) return skip(c, "not registered (step 5)");
  String r;
  if (strlen(CFG_APN)) {
    String cmd = String("AT+CGDCONT=1,\"IP\",\"") + CFG_APN + "\"";
    modem.cmd(cmd.c_str(), 3000);
  }
  if (modem.cmd("AT+CGDCONT?", 3000, &r) == AtResult::OK) {
    String f[4];
    splitCsv(Modem::field(r, "+CGDCONT:"), f, 4);
    info("APN (context 1)", unquote(f[2]).length() ? unquote(f[2]) : String("(network default)"));
  }

  int att = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 30000) {
    if (modem.cmd("AT+CGATT?", 3000, &r) == AtResult::OK) att = Modem::field(r, "+CGATT:").toInt();
    if (att == 1) break;
    delay(2000);
  }
  if (att != 1) {
    note("Not attached -> AT+CGATT=1");
    if (modem.cmd("AT+CGATT=1", 75000) != AtResult::OK)
      return fail(c, "Packet attach failed: " + modem.lastError,
                  "Data plan inactive / APN wrong / operator blocks data");
  }
  info("Packet attach (CGATT)", "1 = attached");

  modem.cmd("AT+CGACT?", 3000, &r);
  if (r.indexOf("+CGACT: 1,1") < 0) {
    note("PDP context inactive -> AT+CGACT=1,1");
    if (modem.cmd("AT+CGACT=1,1", 150000) != AtResult::OK)
      return fail(c, "PDP context activation failed: " + modem.lastError,
                  "Set the correct CFG_APN for your operator, check data balance");
  }

  String ip;
  if (modem.cmd("AT+CGPADDR=1", 3000, &r) == AtResult::OK) {
    String f[3];
    int n = splitCsv(Modem::field(r, "+CGPADDR:"), f, 3);
    if (n >= 2) ip = unquote(f[1]);
  }
  info("IP address", ip.length() ? ip : String("none"));
  if (!ip.length() || ip == "0.0.0.0")
    return fail(c, "No IP address assigned", "Check APN (CFG_APN) and SIM data plan");
  g.dataUp = true;
  pass(c, "PDP active, IP " + ip);
}

// 7 ---------------------------------------------------------------------------
static void t_internet(TestCtx& c) {
  if (!g.dataUp) return skip(c, "no packet data (step 7)");
  String urc, r;
  AtResult res = modem.cmdUrc("AT+NETOPEN", "+NETOPEN:", 30000, &urc, &r);
  if (res == AtResult::OK) {
    int e = Modem::field(urc, "+NETOPEN:").toInt();
    if (e != 0) return fail(c, "AT+NETOPEN failed (+NETOPEN: " + String(e) + ")",
                            "PDP context not usable; re-check APN / CGACT");
    info("TCP/IP stack", "opened");
  } else if (r.indexOf("already opened") >= 0) {
    info("TCP/IP stack", "already open");
  } else {
    return fail(c, "AT+NETOPEN failed: " + modem.lastError, "Check PDP context / APN");
  }

  // DNS
  uint32_t t0 = millis();
  String cmd = String("AT+CDNSGIP=\"") + CFG_DNS_HOST + "\"";
  res = modem.cmdUrc(cmd.c_str(), "+CDNSGIP:", 20000, &urc, &r);
  uint32_t dnsMs = millis() - t0;
  // Two reply formats exist:
  //   documented : +CDNSGIP: 1,"host","ip1"[,"ip2"]   (0,<err> on failure)
  //   this FW    : +CDNSGIP: <n>,"host","ip"  one line per address (n = 1..N)
  // -> success = any +CDNSGIP line with a non-zero first field and an address.
  String ips;
  int nAddr = 0;
  String dnsErr = urc;
  for (int s = 0; s < (int)r.length();) {
    int e = r.indexOf('\n', s);
    if (e < 0) e = r.length();
    String l = r.substring(s, e);
    s = e + 1;
    if (!l.startsWith("+CDNSGIP:")) continue;
    String f[6];
    int n = splitCsv(l.substring(9), f, 6);
    if (n < 3 || f[0].toInt() == 0) { dnsErr = l; continue; }
    for (int k = 2; k < n; k++) {
      String ip = unquote(f[k]);
      if (!ip.length()) continue;
      nAddr++;
      if (nAddr <= 3) ips += (ips.length() ? ", " : "") + ip;
    }
  }
  if (res != AtResult::OK || nAddr == 0)
    return fail(c, "DNS lookup of " + String(CFG_DNS_HOST) + " failed: " +
                       (res != AtResult::OK ? modem.lastError : dnsErr),
                "Network has no internet route / DNS; check APN and data balance");
  info("DNS " CFG_DNS_HOST, String(nAddr) + " addresses in " + fmtMs(dnsMs) + " (" + ips + ")");

  // HTTP (plain). google.com/generate_204 must answer 204 - anything else is interception.
  g.useHttps = false;
  int st; long len; uint32_t ms; String err;
  bool ok = httpGet(CFG_HTTP_TEST_URL, st, len, ms, 30000, err);
  info("HTTP GET", st > 0 ? String(st) + " " + httpStatusText(st) + " in " + fmtMs(ms) : err);
  if (ok && st >= 200 && st < 300) {
    g.internetOk = true;
    return pass(c, "INTERNET CONNECTED: DNS " + fmtMs(dnsMs) + ", HTTP " + st + " in " + fmtMs(ms));
  }

  String httpProblem;
  if (st >= 300 && st < 400) {
    httpProblem = "Carrier redirects plain HTTP (" + String(st) + ")";
    info("Redirected to", g_lastRedirect.length() ? g_lastRedirect : String("(no Location header)"));
    note("The operator is answering instead of the real server: usually a 'buy data pack' / "
         "captive page when the SIM has no active internet package.");
  } else {
    httpProblem = "Plain HTTP failed: " + err;
  }

  // HTTPS cannot be redirected by the carrier - if it works, the internet path is fine.
  note("Trying HTTPS instead...");
  g.useHttps = true;
  ok = httpGet(CFG_HTTP_TEST_URL, st, len, ms, 45000, err);
  info("HTTPS GET", st > 0 ? String(st) + " " + httpStatusText(st) + " in " + fmtMs(ms) : err);
  if (ok && st >= 200 && st < 300) {
    g.internetOk = true;
    return warn(c, "INTERNET CONNECTED via HTTPS (" + String(st) + " in " + fmtMs(ms) + ")",
                httpProblem + "; HTTPS works, speed tests will use HTTPS",
                "Plain HTTP is intercepted by the operator. Check the SIM's data package / balance "
                "(redirect target: " + (g_lastRedirect.length() ? g_lastRedirect : String("?")) + ")");
  }
  g.useHttps = false;
  return fail(c, httpProblem + "; HTTPS also failed (" + err + ")",
              "No real internet access: the SIM most likely has NO ACTIVE DATA PACKAGE / balance. "
              "DNS works because the operator answers it. Recharge or activate an internet pack, "
              "then rerun step 8.");
}

// 8 ---------------------------------------------------------------------------
static void t_speed4g(TestCtx& c) {
  if (!g.internetOk) return skip(c, "no internet (step 8)");
  String rat = currentRat();
  info("Current radio", rat);
  note("Download happens inside the modem; UART speed does not limit this test.");
  double kbps; String err;
  if (!runSpeedTest(false, kbps, err))
    return fail(c, "4G speed test could not download: " + err,
                "Speed-test server unreachable or modem HTTP buffer error; try other URLs");

  char d[96];
  snprintf(d, sizeof d, "4G download %.2f Mbit/s (target %.1f) on %s",
           kbps / 1000.0, CFG_TARGET_4G_KBPS / 1000.0, rat.c_str());
  if (rat != "LTE")
    return warn(c, d, "Test did not run on LTE (radio = " + rat + ")", "Check LTE coverage / AT+CNMP=2");
  if (kbps < CFG_TARGET_4G_KBPS)
    return fail(c, String(d) + " - BELOW TARGET",
                "Cat-1 max is 10 Mbit/s; result depends on signal (CSQ/RSRP), cell load and "
                "server distance. Retest near a window / with better antenna.");
  pass(c, d);
}

// 9 ---------------------------------------------------------------------------
static bool waitRegistered2G(uint32_t timeoutS) {
  uint32_t t0 = millis();
  bool v = modem.verbose();
  modem.setVerbose(false);
  bool ok = false;
  while (millis() - t0 < timeoutS * 1000UL) {
    if ((isRegistered(regStat("AT+CREG?", "+CREG:")) || isRegistered(regStat("AT+CGREG?", "+CGREG:"))) &&
        currentRat() == "GSM") { ok = true; break; }
    Serial.printf("    ... waiting for 2G registration (%lu s)\n", (unsigned long)((millis() - t0) / 1000));
    delay(3000);
  }
  modem.setVerbose(v);
  return ok;
}

static void restoreAutoMode() {
  note("Restoring automatic network mode (AT+CNMP=2)...");
  modem.cmd("AT+CNMP=2", 10000);
  uint32_t t0 = millis();
  bool v = modem.verbose();
  modem.setVerbose(false);
  while (millis() - t0 < 90000) {
    if (isRegistered(regStat("AT+CEREG?", "+CEREG:"))) break;
    delay(3000);
  }
  modem.setVerbose(v);
  info("Radio after restore", currentRat());
}

static void t_speed2g(TestCtx& c) {
  if (!g.internetOk) return skip(c, "no internet (step 8)");
  note("Forcing GSM-only mode (AT+CNMP=13)");
  if (modem.cmd("AT+CNMP=13", 10000) != AtResult::OK)
    return fail(c, "Modem refused GSM-only mode: " + modem.lastError, "Firmware may not support 2G");

  if (!waitRegistered2G(90)) {
    restoreAutoMode();
    return fail(c, "No 2G (GSM) network registration within 90 s",
                "Operator may have shut down 2G here, or 2G bands not covered by antenna");
  }
  String full;
  currentRat(&full);
  info("2G cell (CPSI)", full);

  // make sure data is attached on 2G
  String r;
  for (int i = 0; i < 20; i++) {
    if (modem.cmd("AT+CGATT?", 3000, &r) == AtResult::OK && Modem::field(r, "+CGATT:").toInt() == 1) break;
    delay(2000);
  }

  double kbps; String err;
  bool ok = runSpeedTest(true, kbps, err);
  if (!ok) {
    note("Retrying after re-opening the TCP/IP stack");
    modem.cmd("AT+NETCLOSE", 10000);
    String u;
    modem.cmdUrc("AT+NETOPEN", "+NETOPEN:", 30000, &u);
    ok = runSpeedTest(true, kbps, err);
  }
  restoreAutoMode();

  if (!ok) return fail(c, "2G speed test could not download: " + err,
                       "GPRS/EDGE data not available on this operator, or server unreachable");
  char d[110];
  snprintf(d, sizeof d, "2G download %.0f kbit/s (target %d kbit/s)", kbps, CFG_TARGET_2G_KBPS);
  if (kbps < CFG_TARGET_2G_KBPS)
    return fail(c, String(d) + " - BELOW TARGET",
                "Physical limit: EDGE ~236 kbit/s, GPRS ~85 kbit/s. Set CFG_TARGET_2G_KBPS ~100.");
  pass(c, d);
}

// 10 --------------------------------------------------------------------------
static void t_gnssPower(TestCtx& c) {
  if (!g.atOk) return skip(c, "modem not responding (step 1)");
  String r, urc;
  modem.cmd("AT+CGNSSPWR?", 2000, &r);
  if (Modem::field(r, "+CGNSSPWR:").toInt() == 1) {
    info("GNSS engine", "already ON");
  } else {
    AtResult res = modem.cmdUrc("AT+CGNSSPWR=1", "+CGNSSPWR: READY", 20000, &urc);
    if (res == AtResult::ERROR)
      return fail(c, "AT+CGNSSPWR=1 rejected: " + modem.lastError,
                  "GNSS core not powered: check 1V8_GNSS (pin 97) and GNSS_PWRCTL (pin 98)");
    if (res == AtResult::TIMEOUT) {
      modem.cmd("AT+CGNSSPWR?", 2000, &r);
      if (Modem::field(r, "+CGNSSPWR:").toInt() != 1)
        return fail(c, "GNSS did not report READY within 20 s",
                    "Check 1V8_GNSS pin 97 = 1.8 V, PWRCTL pin 98 HIGH, GNSS UART pins 95/96 <-> 49/50");
      note("No READY! URC but engine reports ON");
    } else {
      info("GNSS engine", "+CGNSSPWR: READY!");
    }
  }
  if (CFG_GNSS_MODE > 0) {
    String cmd = "AT+CGNSSMODE=" + String(CFG_GNSS_MODE);
    modem.cmd(cmd.c_str(), 3000);
  }
  if (modem.cmd("AT+CGNSSMODE?", 2000, &r) == AtResult::OK)
    info("Constellation mode", Modem::field(r, "+CGNSSMODE:"));
  if (modem.cmd("AT+CVAUXS?", 2000, &r) == AtResult::OK)
    info("Antenna bias (CVAUXS)", Modem::field(r, "+CVAUXS:") + " (0 = off, passive antenna)");
  g.gnssOn = true;
  pass(c, "GNSS engine powered and ready");
}

// 11 --------------------------------------------------------------------------
static void t_gnssCold(TestCtx& c) {
  if (!g.gnssOn) return skip(c, "GNSS not powered (step 11)");
  g.gnssFixed = false;
  note("Cold start clears ephemeris/almanac/time. Device must be under OPEN SKY.");
  if (modem.cmd("AT+CGPSCOLD", 5000) != AtResult::OK)
    return fail(c, "AT+CGPSCOLD rejected: " + modem.lastError, "GNSS engine not running");
  delay(1000);
  GnssFix f;
  uint32_t ttff = waitForFix(CFG_GNSS_COLD_TIMEOUT_S, f);
  if (!ttff)
    return fail(c, "No GNSS fix after cold start (" + String(CFG_GNSS_COLD_TIMEOUT_S) + " s)",
                "Go outdoors / open sky. If step 15 shows 0 satellites: RF path pin 90 -> "
                "TA1343A SAW -> antenna, R13, antenna type (active needs AT+CVAUXS=1)");
  g.gnssFixed = true;
  g.fix = f;
  info("Time to first fix", fmtMs(ttff));
  String d = "Cold start TTFF " + fmtMs(ttff);
  if (ttff > 120000)
    return warn(c, d, "Cold TTFF slow (> 120 s)", "Weak satellite signal; check antenna / C/N0 in step 15");
  pass(c, d);
}

// 12 --------------------------------------------------------------------------
static void t_gnssPosition(TestCtx& c) {
  if (!g.gnssOn) return skip(c, "GNSS not powered (step 11)");
  GnssFix f;
  if (!readFix(f)) {
    if (!g.gnssFixed) return skip(c, "no GNSS fix (step 12)");
    return fail(c, "Fix lost: AT+CGNSSINFO has no position", "Signal dropped; check antenna / sky view");
  }
  g.fix = f;
  char b[64];
  snprintf(b, sizeof b, "%.6f, %.6f", f.lat, f.lon);
  info("Latitude, Longitude", b);
  info("Altitude", String(f.alt, 1) + " m");
  info("Speed / course", String(f.speedKmh, 1) + " km/h / " + String(f.course, 1) + " deg");
  info("Fix type", f.mode == 3 ? "3D" : f.mode == 2 ? "2D" : String(f.mode));
  info("Satellites used", String(f.svUsed));
  info("PDOP / HDOP / VDOP", String(f.pdop, 1) + " / " + String(f.hdop, 1) + " / " + String(f.vdop, 1));
  info("UTC date / time", f.date + " " + f.utc);
  snprintf(b, sizeof b, "https://maps.google.com/?q=%.6f,%.6f", f.lat, f.lon);
  info("Map", b);

  snprintf(b, sizeof b, "Position %.6f, %.6f (%s, HDOP %.1f, %d sats)", f.lat, f.lon,
           f.mode == 3 ? "3D" : "2D", f.hdop, f.svUsed);
  if (f.mode != 3 || f.hdop > 5.0f)
    return warn(c, b, "Poor accuracy (2D fix or HDOP > 5)", "More open sky / better antenna");
  pass(c, b);
}

// 13 --------------------------------------------------------------------------
static void t_gnssHot(TestCtx& c) {
  if (!g.gnssOn) return skip(c, "GNSS not powered (step 11)");
  if (!g.gnssFixed) return skip(c, "hot start needs a previous fix (step 12 failed)");
  if (modem.cmd("AT+CGPSHOT", 5000) != AtResult::OK)
    return fail(c, "AT+CGPSHOT rejected: " + modem.lastError, "GNSS engine not running");
  delay(1000);
  GnssFix f;
  uint32_t ttff = waitForFix(CFG_GNSS_HOT_TIMEOUT_S, f);
  if (!ttff)
    return fail(c, "No fix within " + String(CFG_GNSS_HOT_TIMEOUT_S) + " s after hot start",
                "Signal lost or ephemeris invalid; check sky view");
  info("Time to fix (hot)", fmtMs(ttff + 1000));
  String d = "Hot start TTFF " + fmtMs(ttff + 1000);
  if (ttff + 1000 > CFG_GNSS_HOT_TARGET_S * 1000UL)
    return warn(c, d, "Hot start slower than " + String(CFG_GNSS_HOT_TARGET_S) + " s",
                "Weak signal; check C/N0 in step 15");
  pass(c, d);
}

// 14 --------------------------------------------------------------------------
static void t_gnssStatus(TestCtx& c) {
  if (!g.gnssOn) return skip(c, "GNSS not powered (step 11)");
  static SkyView sv;
  sv = SkyView();
  note("Capturing NMEA for " + String(CFG_GNSS_SKYVIEW_S) + " s...");
  captureSky(sv, CFG_GNSS_SKYVIEW_S);
  if (sv.nmeaLines == 0) {
    note("No NMEA on UART -> routing NMEA to AT port (AT+CGNSSPORTSWITCH=0,1)");
    modem.cmd("AT+CGNSSPORTSWITCH=0,1", 2000);
    captureSky(sv, CFG_GNSS_SKYVIEW_S);
  }
  if (sv.nmeaLines == 0)
    return warn(c, "No NMEA received on UART", "NMEA output not routed to the ESP32 UART",
                "Check AT+CGNSSPORTSWITCH / AT+CGNSSTST support on this firmware");

  int totalView = 0, totalTracked = 0, maxCn0 = 0, strong = 0;
  for (int k = 0; k < C_N; k++) {
    int tracked = 0, mx = 0;
    for (int p = 1; p < 200; p++)
      if (sv.snr[k][p] > 0) {
        tracked++;
        if (sv.snr[k][p] > mx) mx = sv.snr[k][p];
        if (sv.snr[k][p] >= CFG_GNSS_GOOD_CN0) strong++;
      }
    if (sv.inView[k] || tracked)
      info(C_NAMES[k], String(sv.inView[k]) + " in view, " + tracked + " tracked, max C/N0 " + mx + " dB-Hz");
    totalView += sv.inView[k];
    totalTracked += tracked;
    if (mx > maxCn0) maxCn0 = mx;
  }
  info("NMEA sentences", String(sv.nmeaLines));
  info("RMC status", sv.rmcStatus == "A" ? "A (valid fix)" : sv.rmcStatus == "V" ? "V (no fix)" : "-");

  String d = String(totalView) + " sats in view, " + totalTracked + " tracked, " + strong +
             " >= " + CFG_GNSS_GOOD_CN0 + " dB-Hz, max C/N0 " + maxCn0;
  if (totalView == 0)
    return fail(c, "No satellites in view (GSV reports 00)",
                "RF front-end problem: pin 90 -> TA1343A SAW -> ANT8010 path, R13, antenna "
                "connection; or device is indoors");
  if (totalTracked == 0)
    return fail(c, "Satellites in almanac but none tracked (all C/N0 = 0)",
                "Antenna signal too weak / RF path broken / indoors");
  if (maxCn0 < CFG_GNSS_GOOD_CN0)
    return warn(c, d, "Weak satellite signals (max C/N0 < " + String(CFG_GNSS_GOOD_CN0) + " dB-Hz)",
                "Marginal antenna or RF path loss; test under open sky");
  pass(c, d);
}

// =============================================================================
//  Runner
// =============================================================================
static const TestDef TESTS[] = {
  {"Modem power-on / boot",       t_powerOn},
  {"Modem identity & health",     t_identity},
  {"SIM card",                    t_sim},
  {"Radio signal (GSM/LTE)",      t_signal},
  {"Network registration",        t_register},
  {"Network status (cell info)",  t_netinfo},
  {"Packet data (PDP / IP)",      t_packet},
  {"Internet (DNS + HTTP)",       t_internet},
  {"Internet speed 4G (LTE)",     t_speed4g},
  {"Internet speed 2G (GSM)",     t_speed2g},
  {"GNSS power-on",               t_gnssPower},
  {"GNSS cold start (TTFF)",      t_gnssCold},
  {"GNSS positioning",            t_gnssPosition},
  {"GNSS hot start (TTFF)",       t_gnssHot},
  {"GNSS status (sky view)",      t_gnssStatus},
};
static const int N_TESTS = sizeof(TESTS) / sizeof(TESTS[0]);
static TestResult results[N_TESTS];

static void runTest(int i) {
  Serial.println();
  Serial.printf("[STEP %02d/%02d] %s\n", i + 1, N_TESTS, TESTS[i].name);
  Serial.println("------------------------------------------------------------------");
  led(0, 0, 20);
  TestCtx c;
  uint32_t t0 = millis();
  TESTS[i].fn(c);
  TestResult& r = results[i];
  r.st = c.st; r.detail = c.detail; r.failMsg = c.failMsg; r.hint = c.hint;
  r.ms = millis() - t0;
  Serial.printf("  ==> %s  %s  [%s]\n", stName(r.st), r.detail.c_str(), fmtMs(r.ms).c_str());
  if (r.st == ST_FAIL || r.st == ST_WARN) {
    if (r.hint.length()) Serial.printf("      hint: %s\n", r.hint.c_str());
  }
}

static void printSummary() {
  int np = 0, nw = 0, nf = 0, ns = 0;
  Serial.println();
  Serial.println("==================================================================");
  Serial.println("                SIMCom A7672SA DEVICE TEST SUMMARY");
  Serial.println("==================================================================");
  for (int i = 0; i < N_TESTS; i++) {
    TestResult& r = results[i];
    Serial.printf(" %2d  %-28s %-5s %s\n", i + 1, TESTS[i].name, stName(r.st), r.detail.c_str());
    if (r.st == ST_PASS) np++;
    else if (r.st == ST_WARN) nw++;
    else if (r.st == ST_FAIL) nf++;
    else if (r.st == ST_SKIP) ns++;
  }
  Serial.println("------------------------------------------------------------------");
  Serial.printf(" PASS %d   WARN %d   FAIL %d   SKIP %d   |  RI pulses seen: %lu\n",
                np, nw, nf, ns, (unsigned long)g_riPulses);

  if (nf || nw || ns) {
    Serial.println();
    Serial.println(" FAILURE / WARNING MESSAGES");
    Serial.println("------------------------------------------------------------------");
    for (int i = 0; i < N_TESTS; i++) {
      TestResult& r = results[i];
      if (r.st != ST_FAIL && r.st != ST_WARN && r.st != ST_SKIP) continue;
      Serial.printf(" [%s] Step %d - %s\n", stName(r.st), i + 1, TESTS[i].name);
      Serial.printf("        reason: %s\n", r.failMsg.c_str());
      if (r.hint.length()) Serial.printf("        check : %s\n", r.hint.c_str());
    }
  }
  Serial.println("==================================================================");
  Serial.printf(" OVERALL: %s\n", nf ? "FAIL" : nw ? "PASS WITH WARNINGS" : ns ? "INCOMPLETE" : "PASS");
  Serial.println("==================================================================");
  if (nf) led(30, 0, 0);
  else if (nw || ns) led(25, 15, 0);
  else led(0, 30, 0);
}

static void runAll() {
  g = State();
  for (int i = 0; i < N_TESTS; i++) results[i] = TestResult();
  Serial.println();
  Serial.println("##################################################################");
  Serial.println("#      FLEET TRACKER - SIMCom A7672SA-FASE FULL DEVICE TEST      #");
  Serial.println("##################################################################");
  for (int i = 0; i < N_TESTS; i++) runTest(i);
  printSummary();
}

static void printHelp() {
  Serial.println();
  Serial.printf("Commands:  a = run all   1..%d = run one step   n = network status\n", N_TESTS);
  Serial.println("           s = summary   p = AT passthrough (#q to exit)   h = help");
  for (int i = 0; i < N_TESTS; i++) Serial.printf("   %2d  %s\n", i + 1, TESTS[i].name);
}

static void passthrough() {
  Serial.println("\n--- AT passthrough. Type AT commands, '#q' to quit ---");
  String in, line;
  for (;;) {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\r' || ch == '\n') {
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

static void IRAM_ATTR onRing() { g_riPulses++; }

// =============================================================================
void setup() {
  pinMode(PIN_MODEM_PWRKEY, OUTPUT); digitalWrite(PIN_MODEM_PWRKEY, LOW);
  pinMode(PIN_MODEM_RESET, OUTPUT);  digitalWrite(PIN_MODEM_RESET, LOW);
  pinMode(PIN_MODEM_DTR, OUTPUT);    digitalWrite(PIN_MODEM_DTR, HIGH);  // keep modem awake
  pinMode(PIN_MODEM_STATUS, INPUT);
  pinMode(PIN_MODEM_RI, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MODEM_RI), onRing, RISING);
  led(0, 0, 10);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 8000) delay(10);  // wait for USB serial monitor

  Serial1.setRxBufferSize(4096);
  Serial1.begin(MODEM_BAUD, SERIAL_8N1, PIN_MODEM_RX, PIN_MODEM_TX);
  modem.begin(Serial1);
  modem.setVerbose(CFG_VERBOSE_AT);

  Serial.println("\nFleet Tracker SIMCom test firmware");
  printHelp();
  if (CFG_AUTO_RUN) {
    Serial.printf("\nFull test starts in %d s (press any key + Enter to cancel)...\n", CFG_AUTO_RUN_DELAY_S);
    uint32_t ts = millis();
    while (millis() - ts < CFG_AUTO_RUN_DELAY_S * 1000UL) {
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        Serial.println("Auto-run cancelled.");
        return;
      }
      delay(20);
    }
    runAll();
    printHelp();
  }
}

void loop() {
  static String in;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\r' && ch != '\n') { in += ch; continue; }
    in.trim();
    if (!in.length()) continue;
    if (in == "a") { runAll(); printHelp(); }
    else if (in == "s") printSummary();
    else if (in == "n") { if (g.atOk) printNetworkStatus(); else Serial.println("Run step 1 first (modem not started)"); }
    else if (in == "p") passthrough();
    else if (in == "h") printHelp();
    else {
      int n = in.toInt();
      if (n >= 1 && n <= N_TESTS) { runTest(n - 1); }
      else Serial.println("Unknown command, 'h' for help");
    }
    in = "";
  }
  // print unsolicited modem messages (SMS, network events) while idle
  String l;
  if (Serial1.available()) modem.readLine(l, 20);
}

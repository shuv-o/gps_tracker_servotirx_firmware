#include "services/commands.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <memory>
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "fw_version.h"
#include "services/cellular.h"
#include "services/gnss.h"
#include "services/mqtt_client.h"
#include "services/obd/obd.h"
#include "services/offline_store.h"
#include "services/ota.h"
#include "services/reporter.h"

namespace cmd {

// Collects command output for MQTT replies.
class BufPrint : public Print {
 public:
  String s;
  BufPrint() { s.reserve(2048); }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    if (s.length() < 6000) s.concat((const char*)b, min(n, 6000 - s.length()));
    return n;
  }
};

typedef void (*Handler)(int argc, char** argv, Print& out);
struct Command { const char* name; const char* usage; Handler fn; };

static bool eq(const char* a, const char* b) { return !strcasecmp(a, b); }

// ------------------------------------------------------------------ handlers
static void cHelp(int, char**, Print& out);

static void cStatus(int, char**, Print& out) {
  out.printf("%s %s  hw %s  up %lus  boot #%lu (%s)%s%s\n", FW_NAME, FW_VERSION, HW_REVISION,
             (unsigned long)(millis() / 1000), (unsigned long)sup::bootCount(), sup::resetReason(),
             sup::safeMode() ? "  SAFE MODE" : "", sup::otaPendingVerify() ? "  OTA-UNCONFIRMED" : "");
  out.printf("device id %s  clock %s (%lu)\n", cell::deviceId(), clockx::sourceName(), (unsigned long)clockx::now());
  if (sup::lastCrash()[0]) out.printf("last crash: %s\n", sup::lastCrash());
  cell::print(out);
  mqtt::print(out);
  gnss::print(out);
  obd::print(out);
  out.printf("queue: RAM %u  flash %lu records (%lu bytes used)\n", (unsigned)bus::pending(),
             (unsigned long)offline::count(), (unsigned long)offline::bytes());
}

static void cNet(int, char**, Print& out) { cell::print(out); }
static void cMqtt(int argc, char** argv, Print& out) {
  if (argc > 1 && eq(argv[1], "reconnect")) { mqtt::reconnectNow(); out.println("reconnecting"); return; }
  mqtt::print(out);
}
static void cGnss(int, char**, Print& out) { gnss::print(out); }
static void cObd(int, char**, Print& out) { obd::print(out); }
static void cPids(int, char**, Print& out) { obd::printPids(out); }
static void cTasks(int, char**, Print& out) { sup::printTasks(out); }

static void cCounters(int, char**, Print& out) {
  forEachCounter([&](const char* n, uint32_t v) { out.printf("%-20s %lu\n", n, (unsigned long)v); });
}

static void cDtc(int argc, char** argv, Print& out) {
  if (argc > 2 && eq(argv[1], "clear") && eq(argv[2], "confirm")) { obd::requestDtcClear(); out.println("DTC clear requested"); return; }
  if (argc > 1 && eq(argv[1], "clear")) { out.println("this erases fault codes in the car: 'dtc clear confirm'"); return; }
  if (argc > 1 && eq(argv[1], "read")) { obd::requestDtcRead(); out.println("DTC read requested"); return; }
  JsonDocument d;
  obd::dtcJson(d.to<JsonObject>());
  serializeJsonPretty(d, out);
  out.println();
}

static void cQueue(int argc, char** argv, Print& out) {
  if (argc > 1 && eq(argv[1], "clear")) { offline::clear(); out.println("offline queue cleared"); return; }
  out.printf("RAM %u  flash %lu records, %lu bytes used\n", (unsigned)bus::pending(), (unsigned long)offline::count(),
             (unsigned long)offline::bytes());
}

static void cLogs(int argc, char** argv, Print& out) {
  size_t n = argc > 1 ? constrain(atoi(argv[1]), 200, 5500) : 3000;
  std::unique_ptr<char[]> buf(new char[n + 1]);
  logx::tail(buf.get(), n + 1);
  out.print(buf.get());
}

static void cLog(int argc, char** argv, Print& out) {
  if (argc > 1) {
    String err;
    if (!config::set("log_level", argv[1], err)) out.println(err);
  }
  out.printf("log level %u (0 off 1 err 2 warn 3 info 4 debug 5 verbose/AT trace)\n", logx::level());
}

static void cConfig(int argc, char** argv, Print& out) {
  const char* sub = argc > 1 ? argv[1] : "list";
  char v[200];
  if (eq(sub, "list")) {
    size_t n;
    const CfgItem* it = config::items(n);
    for (size_t i = 0; i < n; i++) {
      config::valueStr(it[i], v, sizeof v);
      out.printf("%-16s = %-24s  %s\n", it[i].key, v, it[i].help);
    }
  } else if (eq(sub, "get") && argc > 2) {
    const CfgItem* it = config::find(argv[2]);
    if (!it) { out.println("unknown key"); return; }
    config::valueStr(*it, v, sizeof v);
    out.printf("%s = %s\n", it->key, v);
  } else if (eq(sub, "set") && argc > 2) {
    String val;
    for (int i = 3; i < argc; i++) { if (i > 3) val += ' '; val += argv[i]; }
    String err;
    if (config::set(argv[2], val.c_str(), err)) out.printf("%s saved (some settings apply after 'reboot')\n", argv[2]);
    else out.printf("error: %s\n", err.c_str());
  } else if (eq(sub, "reset") && argc > 2 && eq(argv[2], "confirm")) {
    config::factoryReset();
    out.println("config reset to defaults; reboot to apply");
  } else {
    out.println("config list | get <key> | set <key> <value> | reset confirm");
  }
}

static void cReboot(int, char**, Print& out) {
  out.println("rebooting in 2 s");
  bus::event("reboot", "{\"why\":\"command\"}");
  delay(2000);
  sup::reboot("command");
}

static void cSafe(int argc, char** argv, Print& out) {
  Preferences nv;
  nv.begin("sys", false);
  if (argc > 1 && eq(argv[1], "on")) { nv.putBool("safe", true); out.println("safe mode on next boot"); }
  else if (argc > 1 && eq(argv[1], "off")) { nv.end(); sup::clearSafeMode(); out.println("safe mode cleared; reboot to resume"); return; }
  else out.printf("safe mode %s\n", sup::safeMode() ? "ACTIVE" : "off");
  nv.end();
}

// AT passthrough (runs on the cellular task)
static void cAt(int argc, char** argv, Print& out) {
  if (argc < 2) { out.println("usage: at AT+CSQ"); return; }
  String line;
  for (int i = 1; i < argc; i++) { if (i > 1) line += ' '; line += argv[i]; }
  if (!line.startsWith("AT") && !line.startsWith("at")) line = "AT" + line;
  auto res = std::make_shared<String>();
  bool done = cell::exec([line, res](AtChannel& at) {
    AtResp r;
    At st = at.cmd(15000, &r, "%s", line.c_str());
    *res = String(r.buf) + (st == At::OK ? "OK" : st == At::ERROR ? String("ERROR ") + at.lastError() : "TIMEOUT");
  }, 20000);
  out.println(done ? *res : String("timeout waiting for modem task"));
}

static void cModem(int argc, char** argv, Print& out) {
  if (argc > 2 && eq(argv[1], "recover")) {
    cell::forceRecovery(atoi(argv[2]));
    out.println("recovery requested (1 pdp, 2 radio, 3 soft reset, 4 hard reset, 5 power cycle)");
  } else {
    cell::print(out);
    out.println("modem recover <1-5>");
  }
}

static void cRat(int argc, char** argv, Print& out) {
  if (argc < 2) { out.printf("rat_mode %ld (0 auto 1 lte 2 gsm)\n", (long)cfg().rat_mode); return; }
  int m = eq(argv[1], "lte") ? 1 : eq(argv[1], "gsm") || eq(argv[1], "2g") ? 2 : 0;
  cell::setRatMode(m);
  out.printf("network mode -> %s\n", m == 1 ? "LTE only" : m == 2 ? "GSM only" : "auto");
}

static void cGps(int argc, char** argv, Print& out) {
  if (argc < 2) { gnss::print(out); out.println("gps cold|warm|hot|power|agnss"); return; }
  String how = argv[1];
  auto ok = std::make_shared<bool>(false);
  cell::exec([how, ok](AtChannel& at) {
    *ok = how == "agnss" ? gnss::requestAgnss(at) : gnss::restart(at, how.c_str());
  }, 70000);
  out.printf("gps %s -> %s\n", how.c_str(), *ok ? "ok" : "failed");
}

static void cOta(int argc, char** argv, Print& out) {
  if (argc < 2) { out.println("ota <url> [md5] [force] [version=x.y.z]"); return; }
  auto req = std::make_shared<ota::Request>();
  memset(req.get(), 0, sizeof(ota::Request));
  strlcpy(req->url, argv[1], sizeof req->url);
  for (int i = 2; i < argc; i++) {
    if (eq(argv[i], "force")) req->force = true;
    else if (!strncmp(argv[i], "version=", 8)) strlcpy(req->version, argv[i] + 8, sizeof req->version);
    else if (strlen(argv[i]) == 32) strlcpy(req->md5, argv[i], sizeof req->md5);
  }
  auto err = std::make_shared<String>();
  out.println("OTA started (progress in log / events)");
  cell::exec([req, err](AtChannel& at) { ota::run(at, *req, *err); }, 30 * 60000UL);
  out.printf("OTA failed: %s\n", err->c_str());  // on success the device reboots before this
}

static void cCan(int argc, char** argv, Print& out) {
  if (argc > 2 && eq(argv[1], "sniff")) { obd::setSniff(eq(argv[2], "on")); out.printf("sniff %s\n", argv[2]); return; }
  if (argc > 1 && eq(argv[1], "rescan")) { obd::requestRescan(); out.println("rescan requested"); return; }
  if (argc > 3 && eq(argv[1], "send")) {
    uint32_t id = strtoul(argv[2], nullptr, 16);
    uint8_t d[8] = {0};
    uint8_t n = 0;
    for (const char* h = argv[3]; h[0] && h[1] && n < 8; h += 2) { char b[3] = {h[0], h[1], 0}; d[n++] = strtoul(b, nullptr, 16); }
    out.println(obd::sendRaw(id, id > 0x7FF, d, n) ? "queued" : "queue full");
    return;
  }
  obd::print(out);
  out.println("can sniff on|off | rescan | send <hexid> <hexdata>");
}

static void cReport(int argc, char** argv, Print& out) {
  const char* w = argc > 1 ? argv[1] : "health";
  out.printf("report %s -> %s\n", w, report::send(w) ? "queued" : "unknown (loc|obd|dtc|health|info)");
}

static void cVer(int, char**, Print& out) {
  out.printf("%s %s built %s %s, hw %s\n", FW_NAME, FW_VERSION, __DATE__, __TIME__, HW_REVISION);
}

// clang-format off
static const Command COMMANDS[] = {
  {"help",     "list commands",                                   cHelp},
  {"status",   "everything at a glance",                          cStatus},
  {"net",      "cellular network details",                        cNet},
  {"mqtt",     "broker connection [reconnect]",                   cMqtt},
  {"gnss",     "location / fix details",                          cGnss},
  {"gps",      "cold|warm|hot|power|agnss restart",               cGps},
  {"obd",      "vehicle bus state",                               cObd},
  {"pids",     "live OBD-II values",                              cPids},
  {"dtc",      "fault codes [read | clear confirm]",              cDtc},
  {"can",      "sniff on|off | rescan | send <id> <hex>",         cCan},
  {"tasks",    "task stacks, heartbeats, heap",                   cTasks},
  {"counters", "telemetry counters",                              cCounters},
  {"queue",    "offline message queue [clear]",                   cQueue},
  {"logs",     "recent log lines [bytes]",                        cLogs},
  {"log",      "log level <0-5>",                                 cLog},
  {"config",   "list | get | set | reset confirm",                cConfig},
  {"at",       "send AT command to the modem",                    cAt},
  {"modem",    "modem info | recover <1-5>",                      cModem},
  {"rat",      "auto | lte | gsm",                                cRat},
  {"report",   "send now: loc|obd|dtc|health|info",               cReport},
  {"ota",      "<url> [md5] [force] firmware update",             cOta},
  {"safemode", "on | off",                                        cSafe},
  {"reboot",   "restart the device",                              cReboot},
  {"ver",      "firmware version",                                cVer},
};
// clang-format on

static void cHelp(int, char**, Print& out) {
  for (auto& c : COMMANDS) out.printf("  %-9s %s\n", c.name, c.usage);
}

bool runLine(const char* line, Print& out) {
  char buf[256];
  strlcpy(buf, line, sizeof buf);
  char* argv[12];
  int argc = 0;
  for (char* p = strtok(buf, " \t\r\n"); p && argc < 12; p = strtok(nullptr, " \t\r\n")) argv[argc++] = p;
  if (!argc) return true;
  CNT_INC(cmds);
  for (auto& c : COMMANDS) {
    if (eq(argv[0], c.name)) {
      c.fn(argc, argv, out);
      return true;
    }
  }
  out.printf("unknown command '%s' - try 'help'\n", argv[0]);
  return false;
}

void handleRemote(const char* payload) {
  JsonDocument in;
  String id, line;
  if (payload[0] == '{' && !deserializeJson(in, payload)) {
    id = in["id"] | "";
    line = in["cmd"] | "";
  } else {
    line = payload;
  }
  LOGI("CMD", "remote: %s", line.c_str());
  BufPrint out;
  bool ok = runLine(line.c_str(), out);
  JsonDocument resp;
  if (id.length()) resp["id"] = id;
  resp["cmd"] = line;
  resp["ok"] = ok;
  resp["ts"] = clockx::now();
  resp["out"] = out.s;
  String s;
  serializeJson(resp, s);
  bus::publish("cmd/resp", s.c_str(), 1, false, false);
}

// ------------------------------------------------------------------ console
static void consoleTask(void*) {
  int hb = sup::registerTask("console", 3600000UL, false);  // may block on long commands (OTA)
  char line[256];
  size_t n = 0;
  Serial.println("\nconsole ready - type 'help'");
  for (;;) {
    sup::beat(hb);
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (!n) continue;
        line[n] = 0;
        n = 0;
        Serial.printf("> %s\n", line);
        runLine(line, Serial);
      } else if (c == 8 || c == 127) {
        if (n) n--;
      } else if (n < sizeof line - 1) {
        line[n++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void startConsole() { xTaskCreatePinnedToCore(consoleTask, "console", 8192, nullptr, 2, nullptr, 0); }

}  // namespace cmd

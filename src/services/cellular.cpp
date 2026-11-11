#include "services/cellular.h"
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <memory>
#include "board_pins.h"
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/deadline.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "drivers/modem_hw.h"
#include "services/commands.h"
#include "services/gnss.h"
#include "services/mqtt_client.h"
#include "services/offline_store.h"
#include "services/ota.h"
#include "services/reporter.h"

namespace cell {

static const char* TAG = "NET";
static AtChannel s_at;
static TaskHandle_t s_task = nullptr;
static int s_hb = -1;
static SemaphoreHandle_t s_mtx;
static NetInfo s_info{};
static char s_devId[24] = "";

// state machine
static NetPhase s_phase = NetPhase::Off;
static uint32_t s_phaseSince = 0;
static uint8_t s_level = 0;              // recovery level reached
static uint32_t s_backoffUntil = 0;
static uint8_t s_powerCycles = 0;
static uint32_t s_nextRefresh = 0, s_nextSlow = 0, s_nextClock = 0;
static uint32_t s_regLostSince = 0;
static uint8_t s_atFailStreak = 0;
static bool s_forced2G = false;
static uint32_t s_lteRetryAt = 0;
static bool s_lteTrial = false;
static uint32_t s_lteTrialDeadline = 0;
static uint32_t s_mqttOkSince = 0;
static volatile uint8_t s_forceLevel = 0;
static bool s_ratDirty = true;            // apply AT+CNMP on the next Register pass
static volatile int8_t s_ratRequest = -1; // rat mode change requested by another task
static uint32_t s_curBaud = 115200;
static void handleRatRequest();

// URC flags
static volatile bool f_modemRebooted = false, f_simLost = false, f_pdpLost = false, f_timeUpdate = false;
static volatile bool f_dataFailure = false;
static char s_dataFailWhy[48];

struct JobBox {
  Job fn;
  SemaphoreHandle_t done;
  JobBox() : done(xSemaphoreCreateBinary()) {}
  ~JobBox() { vSemaphoreDelete(done); }
};
static QueueHandle_t s_jobQ;

struct Lock {
  Lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(s_mtx); }
};

const char* phaseName(NetPhase p) {
  switch (p) {
    case NetPhase::Off:      return "off";
    case NetPhase::PowerOn:  return "power_on";
    case NetPhase::Sync:     return "at_sync";
    case NetPhase::Sim:      return "sim";
    case NetPhase::Register: return "registering";
    case NetPhase::Attach:   return "attaching";
    case NetPhase::Online:   return "online";
    case NetPhase::Recover:  return "recovering";
  }
  return "?";
}

const char* deviceId() { return s_devId; }
bool online() { return s_phase == NetPhase::Online; }

static void setPhase(NetPhase p) {
  if (p != s_phase) LOGI(TAG, "%s -> %s", phaseName(s_phase), phaseName(p));
  if (p == NetPhase::Register) s_ratDirty = true;
  s_phase = p;
  s_phaseSince = millis();
  Lock l;
  s_info.phase = p;
  s_info.dataUp = p == NetPhase::Online;
  if (p == NetPhase::Online) s_info.onlineSinceMs = millis();
}

static uint32_t inPhaseMs() { return millis() - s_phaseSince; }

static void failNote(const char* why) {
  Lock l;
  strlcpy(s_info.lastFailure, why, sizeof s_info.lastFailure);
}

// ------------------------------------------------------------------ URCs
static void onUrc(const char* line) {
  if (!strncmp(line, "+CMQTT", 6)) { mqtt::onUrc(line); return; }
  if (!strcmp(line, "RDY") || !strncmp(line, "*ATREADY", 8)) {
    if (s_phase > NetPhase::Sync) f_modemRebooted = true;
  } else if (!strncmp(line, "+CPIN: NOT", 10) || !strncmp(line, "+CPIN: SIM REMOVED", 18)) {
    f_simLost = true;
  } else if (!strncmp(line, "+CGEV:", 6) && (strstr(line, "DEACT") || strstr(line, "DETACH"))) {
    f_pdpLost = true;
  } else if (!strncmp(line, "+CTZV", 5) || !strncmp(line, "*PSUTTZ", 7)) {
    f_timeUpdate = true;
  } else if (!strncmp(line, "+CMTI", 5)) {
    LOGI(TAG, "SMS received: %s", line);
  }
  LOGD(TAG, "URC %s", line);
}

static void onMqttRx(const char* topic, const char* payload, size_t len) { mqtt::onRx(topic, payload, len); }

// ------------------------------------------------------------------ helpers
static int regStat(const char* q, const char* prefix) {
  AtResp r;
  char v[48];
  if (s_at.cmd(3000, &r, "%s", q) != At::OK || !r.get(prefix, v, sizeof v)) return -1;
  const char* c = strchr(v, ',');
  return c ? atoi(c + 1) : -1;
}
static bool isReg(int s) { return s == 1 || s == 5; }

static int splitCsv(char* s, char** tok, int max) {
  int n = 0;
  for (char* p = s; n < max;) {
    while (*p == ' ') p++;
    tok[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = 0;
    p = c + 1;
  }
  return n;
}

static void unquote(char* s) {
  size_t n = strlen(s);
  if (n && s[n - 1] == '"') s[n - 1] = 0;
  if (s[0] == '"') memmove(s, s + 1, strlen(s));
}

static void readClock() {
  AtResp r;
  char v[40];
  if (s_at.cmd(2000, &r, "AT+CCLK?") != At::OK || !r.get("+CCLK:", v, sizeof v)) return;
  unquote(v);  // yy/MM/dd,hh:mm:ss+zz (zz = quarter hours)
  int y, mo, d, h, mi, s, tz = 0;
  char sign = '+';
  if (sscanf(v, "%d/%d/%d,%d:%d:%d%c%d", &y, &mo, &d, &h, &mi, &s, &sign, &tz) >= 6 && y >= 23 && y < 80) {
    uint32_t local = clockx::fromYmdHms(2000 + y, mo, d, h, mi, s);
    long off = (sign == '-' ? -1 : 1) * tz * 15L * 60L;
    clockx::set(local - off, clockx::NETWORK);
  }
}

static void readIdentity() {
  AtResp r;
  char model[24] = "", fw[40] = "", imei[20] = "";
  if (s_at.cmd(2000, &r, "AT+CGMM") == At::OK) r.first(model, sizeof model);
  if (s_at.cmd(2000, &r, "AT+CGMR") == At::OK) r.get("+CGMR:", fw, sizeof fw);
  if (!(s_at.cmd(2000, &r, "AT+SIMEI?") == At::OK && r.get("+SIMEI:", imei, sizeof imei)) &&
      s_at.cmd(2000, &r, "AT+CGSN") == At::OK)
    r.first(imei, sizeof imei);
  Lock l;
  if (model[0]) strlcpy(s_info.model, model, sizeof s_info.model);
  if (fw[0]) strlcpy(s_info.modemFw, fw, sizeof s_info.modemFw);
  if (imei[0]) strlcpy(s_info.imei, imei, sizeof s_info.imei);
  if (s_info.imei[0] && strcmp(s_devId, s_info.imei)) {
    strlcpy(s_devId, s_info.imei, sizeof s_devId);
    LOGI(TAG, "device id (IMEI) %s", s_devId);
  }
}

static void refreshSignal(bool slow) {
  AtResp r;
  char v[160];
  int csq = 99, rsrp = 0, rsrq = 0;
  bool ok = false;
  if (s_at.cmd(2000, &r, "AT+CSQ") == At::OK && r.get("+CSQ:", v, sizeof v)) { csq = atoi(v); ok = true; }
  char rat[12] = "", mccmnc[8] = "", tac[12] = "", band[20] = "";
  uint32_t cid = 0;
  if (s_at.cmd(2000, &r, "AT+CPSI?") == At::OK && r.get("+CPSI:", v, sizeof v)) {
    char* t[16];
    int n = splitCsv(v, t, 16);
    strlcpy(rat, t[0], sizeof rat);
    if (n > 4) { strlcpy(mccmnc, t[2], sizeof mccmnc); strlcpy(tac, t[3], sizeof tac); cid = strtoul(t[4], nullptr, 0); }
    if (!strcmp(rat, "LTE") && n > 6) strlcpy(band, t[6], sizeof band);
    else if (n > 5) strlcpy(band, t[5], sizeof band);
  }
  if (s_at.cmd(2000, &r, "AT+CESQ") == At::OK && r.get("+CESQ:", v, sizeof v)) {
    char* t[6];
    if (splitCsv(v, t, 6) == 6) {
      int q = atoi(t[4]), p = atoi(t[5]);
      if (p != 255) rsrp = -141 + p;
      if (q != 255) rsrq = (int)lround(-20 + q * 0.5);
    }
  }
  s_atFailStreak = ok ? 0 : s_atFailStreak + 1;
  {
    Lock l;
    s_info.csq = csq;
    s_info.rsrp = rsrp;
    s_info.rsrq = rsrq;
    if (rat[0]) strlcpy(s_info.rat, rat, sizeof s_info.rat);
    if (mccmnc[0]) strlcpy(s_info.mccmnc, mccmnc, sizeof s_info.mccmnc);
    if (tac[0]) strlcpy(s_info.tac, tac, sizeof s_info.tac);
    if (band[0]) strlcpy(s_info.band, band, sizeof s_info.band);
    if (cid) s_info.cellId = cid;
    s_info.forced2G = s_forced2G;
    s_info.recoveryLevel = s_level;
  }
  if (!slow) return;
  float vb = 0;
  int temp = 0;
  if (s_at.cmd(2000, &r, "AT+CBC") == At::OK && r.get("+CBC:", v, sizeof v)) vb = atof(v);
  if (s_at.cmd(2000, &r, "AT+CPMUTEMP") == At::OK && r.get("+CPMUTEMP:", v, sizeof v)) temp = atoi(v);
  char op[24] = "";
  if (s_at.cmd(5000, &r, "AT+COPS?") == At::OK && r.get("+COPS:", v, sizeof v)) {
    char* t[4];
    if (splitCsv(v, t, 4) >= 3) { unquote(t[2]); strlcpy(op, t[2], sizeof op); }
  }
  Lock l;
  if (vb > 0) s_info.vbat = vb;
  s_info.modemTemp = temp;
  if (op[0]) strlcpy(s_info.op, op, sizeof s_info.op);
}

// ------------------------------------------------------------------ recovery
static void recover(const char* why) {
  failNote(why);
  mqtt::networkLost();
  bool forced = s_forceLevel != 0;
  if (forced) { s_level = s_forceLevel - 1; s_forceLevel = 0; }

  // 4G -> 2G failover: LTE registers but data keeps failing
  if (!forced && cfg().rat_mode == 0 && !s_forced2G && s_level >= 1 && !strcmp(s_info.rat, "LTE")) {
    s_forced2G = true;
    s_lteRetryAt = after(cfg().lte_retry_s * 1000UL);
    CNT_INC(rat_fallbacks);
    char d[96];
    snprintf(d, sizeof d, "{\"from\":\"LTE\",\"to\":\"GSM\",\"why\":\"%s\"}", why);
    bus::event("rat_fallback", d);
    setPhase(NetPhase::Register);
    return;
  }

  if (s_level < 5) s_level++;
  LOGW(TAG, "RECOVERY L%u: %s", s_level, why);
  char d[96];
  snprintf(d, sizeof d, "{\"level\":%u,\"why\":\"%s\"}", s_level, why);
  bus::event("net_recovery", d);
  setPhase(NetPhase::Recover);

  switch (s_level) {
    case 1:
      CNT_INC(pdp_reattach);
      s_at.cmd(40000, nullptr, "AT+CGACT=0,1");
      setPhase(NetPhase::Attach);
      break;
    case 2:
      CNT_INC(radio_cycles);
      s_at.cmd(15000, nullptr, "AT+CFUN=0");
      delay(3000);
      s_at.cmd(15000, nullptr, "AT+CFUN=1");
      setPhase(NetPhase::Register);
      break;
    case 3:
      CNT_INC(modem_soft_resets);
      s_at.cmd(5000, nullptr, "AT+CRESET");
      delay(8000);
      setPhase(NetPhase::Sync);
      break;
    case 4:
      CNT_INC(modem_hard_resets);
      modemhw::hardReset();
      delay(8000);
      setPhase(NetPhase::Sync);
      break;
    default: {
      CNT_INC(modem_power_cycles);
      s_at.cmd(5000, nullptr, "AT+CPOF");
      if (!modemhw::powerOffHard(12000)) LOGE(TAG, "modem refuses to power off");
      delay(5000);
      // keep power cycling, but with growing pauses (2,4,8..30 min) to save power
      uint32_t mins = min<uint32_t>(30, 2u << min<uint8_t>(s_powerCycles, 4));
      s_powerCycles++;
      s_backoffUntil = after(mins * 60000UL);
      LOGW(TAG, "modem power cycled; next full retry pause %lu min", (unsigned long)mins);
      setPhase(NetPhase::Off);
      break;
    }
  }
  gnss::modemReset();  // engine restarts with the modem
}

// ------------------------------------------------------------------ phases
static void initModem() {
  s_at.cmd(1000, nullptr, "ATE0");
  s_at.cmd(1000, nullptr, "AT+CMEE=2");
  s_at.cmd(1000, nullptr, "AT+CTZU=1");
  s_at.cmd(1000, nullptr, "AT+CREG=0");
  s_at.cmd(1000, nullptr, "AT+CEREG=0");
  s_at.cmd(1000, nullptr, "AT+CGREG=0");
  s_at.cmd(1000, nullptr, "AT+CGEREP=2,0");
  s_at.cmd(1000, nullptr, "AT+CMGF=1");
  s_at.cmd(1000, nullptr, "AT+CNMI=2,1,0,0,0");
  s_at.cmd(1000, nullptr, "AT+CSCLK=0");
  s_at.cmd(1000, nullptr, "AT+COPS=3,0");
  readIdentity();
}

static void applyRat() {
  int want = cfg().rat_mode == 1 ? 38 : cfg().rat_mode == 2 ? 13 : (s_forced2G ? 13 : 2);
  AtResp r;
  char v[16];
  int cur = -1;
  if (s_at.cmd(2000, &r, "AT+CNMP?") == At::OK && r.get("+CNMP:", v, sizeof v)) cur = atoi(v);
  if (cur != want) {
    LOGI(TAG, "network mode %d -> %d", cur, want);
    s_at.cmd(10000, nullptr, "AT+CNMP=%d", want);
  }
}

static void phaseOff() {
  if (!due(s_backoffUntil)) return;
  setPhase(NetPhase::PowerOn);
  if (!modemhw::isOn() && !modemhw::powerOn(15000)) {
    failNote("modem did not power on (STATUS stays off)");
    LOGE(TAG, "modem power-on failed");
    s_backoffUntil = after(30000);
    setPhase(NetPhase::Off);
    return;
  }
  setPhase(NetPhase::Sync);
}

static void applyBaud() {
  uint32_t want = cfg().modem_baud;
  if (want == s_curBaud) return;
  if (s_at.cmd(2000, nullptr, "AT+IPR=%lu", (unsigned long)want) == At::OK) {
    delay(100);
    s_at.setBaud(want);
    if (s_at.ping(1000)) { s_curBaud = want; LOGI(TAG, "modem UART now %lu baud", (unsigned long)want); return; }
  }
  LOGW(TAG, "baud %lu not usable, staying at %lu", (unsigned long)want, (unsigned long)s_curBaud);
  s_at.setBaud(s_curBaud);
}

static void phaseSync() {
  if (s_at.ping(500)) {
    applyBaud();
    initModem();
    s_atFailStreak = 0;
    setPhase(NetPhase::Sim);
    return;
  }
  if (!modemhw::isOn() && inPhaseMs() > 10000) { setPhase(NetPhase::Off); return; }
  // modem restarts at its default 115200: try the other rate before escalating
  if (inPhaseMs() > 8000 && (inPhaseMs() / 4000) % 2) {
    uint32_t alt = s_curBaud == 115200 ? (uint32_t)cfg().modem_baud : 115200;
    if (alt != s_curBaud) {
      s_at.setBaud(alt);
      if (s_at.ping(500)) { LOGW(TAG, "modem answers at %lu baud", (unsigned long)alt); s_curBaud = alt; return; }
      s_at.setBaud(s_curBaud);
    }
  }
  if (inPhaseMs() > 30000) {
    if (s_level < 3) s_level = 3;  // AT dead: skip the soft levels
    recover("modem not answering AT");
  }
  delay(300);
}

static void phaseSim() {
  AtResp r;
  char v[32];
  At res = s_at.cmd(5000, &r, "AT+CPIN?");
  if (res == At::OK && r.get("+CPIN:", v, sizeof v)) {
    if (!strcmp(v, "READY")) {
      char iccid[24] = "", imsi[20] = "";
      if (s_at.cmd(2000, &r, "AT+CICCID") == At::OK) r.get("+ICCID:", iccid, sizeof iccid);
      if (s_at.cmd(2000, &r, "AT+CIMI") == At::OK) r.first(imsi, sizeof imsi);
      {
        Lock l;
        strlcpy(s_info.iccid, iccid, sizeof s_info.iccid);
        strlcpy(s_info.imsi, imsi, sizeof s_info.imsi);
      }
      setPhase(NetPhase::Register);
      return;
    }
    if (!strcmp(v, "SIM PIN") && cfg().sim_pin[0]) {
      s_at.cmd(10000, nullptr, "AT+CPIN=\"%s\"", cfg().sim_pin);
      delay(2000);
      return;
    }
    failNote(v);
  } else {
    failNote(s_at.lastError());
  }
  if (inPhaseMs() > 60000) {
    LOGE(TAG, "SIM not ready: %s", s_info.lastFailure);
    recover("SIM not ready");
  } else {
    delay(2000);
  }
}

static uint32_t s_nextRegPoll = 0;
static void phaseRegister() {
  if (s_ratDirty) { s_ratDirty = false; applyRat(); }
  if (!due(s_nextRegPoll)) return;
  s_nextRegPoll = after(2000);
  int ce = regStat("AT+CEREG?", "+CEREG:");
  int cr = regStat("AT+CREG?", "+CREG:");
  int cg = regStat("AT+CGREG?", "+CGREG:");
  if (isReg(ce) || isReg(cr) || isReg(cg)) {
    refreshSignal(true);
    LOGI(TAG, "registered on %s (%s) CSQ %d", s_info.op, s_info.rat, s_info.csq);
    setPhase(NetPhase::Attach);
    return;
  }
  if (ce == 3 || cr == 3) failNote("registration denied");
  if (inPhaseMs() > (uint32_t)cfg().reg_timeout_s * 1000UL) {
    CNT_INC(reg_failures);
    recover(ce == 3 || cr == 3 ? "registration denied" : "registration timeout");
  }
}

static void phaseAttach() {
  const Config& c = cfg();
  AtResp r;
  char v[96];
  if (c.apn[0]) {
    s_at.cmd(3000, &r, "AT+CGDCONT?");
    if (!r.has(c.apn)) s_at.cmd(5000, nullptr, "AT+CGDCONT=1,\"IP\",\"%s\"", c.apn);
    if (c.apn_user[0]) s_at.cmd(5000, nullptr, "AT+CGAUTH=1,1,\"%s\",\"%s\"", c.apn_pass, c.apn_user);
  }
  bool attached = s_at.cmd(3000, &r, "AT+CGATT?") == At::OK && r.get("+CGATT:", v, sizeof v) && atoi(v) == 1;
  if (!attached) attached = s_at.cmd(75000, nullptr, "AT+CGATT=1") == At::OK;
  if (attached) {
    s_at.cmd(3000, &r, "AT+CGACT?");
    if (!r.has("+CGACT: 1,1") && s_at.cmd(150000, nullptr, "AT+CGACT=1,1") != At::OK) attached = false;
  }
  char ip[48] = "";
  if (attached && s_at.cmd(3000, &r, "AT+CGPADDR=1") == At::OK && r.get("+CGPADDR:", v, sizeof v)) {
    char* t[3];
    int n = splitCsv(v, t, 3);
    if (n >= 2) { unquote(t[1]); strlcpy(ip, t[1], sizeof ip); }
  }
  if (!ip[0] || !strcmp(ip, "0.0.0.0")) {
    if (inPhaseMs() > 120000) recover("no IP address (APN / data plan?)");
    else delay(3000);
    return;
  }
  {
    Lock l;
    strlcpy(s_info.ip, ip, sizeof s_info.ip);
  }
  s_regLostSince = 0;
  s_backoffUntil = 0;
  setPhase(NetPhase::Online);
  readClock();
  char d[200];
  snprintf(d, sizeof d, "{\"rat\":\"%s\",\"op\":\"%s\",\"csq\":%d,\"ip\":\"%s\",\"fb2g\":%s}", s_info.rat, s_info.op,
           s_info.csq, ip, s_forced2G ? "true" : "false");
  bus::event("net_online", d);
}

// Distinguish "broker down" from "no internet" before resetting anything.
static bool internetReachable() {
  char urc[96];
  s_at.cmdUrc(15000, "+NETOPEN:", urc, sizeof urc, nullptr, "AT+NETOPEN");  // may say "already opened"
  // a well-known host first, so a broken broker DNS name never looks like "no internet"
  static const char* const HOSTS[] = {"www.google.com", "cloudflare.com"};
  for (const char* h : HOSTS) {
    At r = s_at.cmdUrc(20000, "+CDNSGIP:", urc, sizeof urc, nullptr, "AT+CDNSGIP=\"%s\"", h);
    if (r == At::OK && strncmp(urc, "+CDNSGIP: 0", 11) != 0) return true;
  }
  return false;
}

static void phaseOnline() {
  uint32_t now = millis();
  if (f_modemRebooted) { f_modemRebooted = false; LOGW(TAG, "modem rebooted by itself"); mqtt::networkLost(); gnss::modemReset(); setPhase(NetPhase::Sync); return; }
  if (f_simLost) { f_simLost = false; mqtt::networkLost(); setPhase(NetPhase::Sim); return; }
  if (f_pdpLost) { f_pdpLost = false; LOGW(TAG, "PDP context lost"); mqtt::networkLost(); setPhase(NetPhase::Attach); return; }
  if (s_forceLevel) { recover("forced by command"); return; }

  if (f_dataFailure) {
    f_dataFailure = false;
    if (internetReachable()) LOGW(TAG, "data failure (%s) but DNS works -> broker side, no reset", s_dataFailWhy);
    else { recover(s_dataFailWhy); return; }
  }

  if (due(s_nextRefresh)) {
    bool slow = due(s_nextSlow);
    s_nextRefresh = after(20000);
    if (slow) s_nextSlow = after(120000);
    refreshSignal(slow);
    if (s_atFailStreak >= 3) { recover("AT timeouts while online"); return; }
    int ce = regStat("AT+CEREG?", "+CEREG:"), cr = regStat("AT+CREG?", "+CREG:");
    if (!isReg(ce) && !isReg(cr)) {
      if (!s_regLostSince) s_regLostSince = now;
      if (now - s_regLostSince > 60000) {
        LOGW(TAG, "registration lost");
        mqtt::networkLost();
        bus::event("net_lost", nullptr);
        setPhase(NetPhase::Register);
        return;
      }
    } else {
      s_regLostSince = 0;
    }
  }
  if (f_timeUpdate || due(s_nextClock)) {
    f_timeUpdate = false;
    s_nextClock = after(3600000UL);
    readClock();
  }

  // healthy for 5 min -> forget past escalation
  if (mqtt::connected()) {
    if (!s_mqttOkSince) s_mqttOkSince = now;
    if (now - s_mqttOkSince > 300000UL && (s_level || s_powerCycles)) {
      LOGI(TAG, "stable again, recovery level reset");
      s_level = 0;
      s_powerCycles = 0;
    }
  } else {
    s_mqttOkSince = 0;
  }

  // LTE trial while on forced 2G
  if (s_lteTrial && mqtt::connected()) {
    s_lteTrial = false;
    CNT_INC(rat_restores);
    bus::event("rat_restore", "{\"to\":\"LTE\"}");
  } else if (s_lteTrial && due(s_lteTrialDeadline)) {
    s_lteTrial = false;
    s_forced2G = true;
    s_lteRetryAt = after(cfg().lte_retry_s * 1000UL);
    LOGW(TAG, "LTE trial failed, back to 2G");
    mqtt::networkLost();
    setPhase(NetPhase::Register);
  } else if (s_forced2G && due(s_lteRetryAt)) {
    LOGI(TAG, "trying LTE again");
    s_forced2G = false;
    s_lteTrial = true;
    s_lteTrialDeadline = after(240000UL);
    mqtt::networkLost();
    setPhase(NetPhase::Register);
  }
}

static void netStep() {
  switch (s_phase) {
    case NetPhase::Off:      phaseOff(); break;
    case NetPhase::PowerOn:  setPhase(NetPhase::Sync); break;
    case NetPhase::Sync:     phaseSync(); break;
    case NetPhase::Sim:      phaseSim(); break;
    case NetPhase::Register: phaseRegister(); break;
    case NetPhase::Attach:   phaseAttach(); break;
    case NetPhase::Online:   phaseOnline(); break;
    case NetPhase::Recover:  setPhase(NetPhase::Sync); break;
  }
}

// ------------------------------------------------------------------ delivery
static void deliver() {
  if (ota::inProgress()) return;
  if (mqtt::connected()) {
    for (int i = 0; i < 5 && mqtt::connected(); i++) {  // replay stored data, oldest first
      OutMsg* m = offline::peek();
      if (!m) break;
      bool ok = mqtt::publish(s_at, m->suffix, m->payload, m->len, m->qos, m->retain);
      bus::release(m);
      if (!ok) break;
      offline::ack();
    }
    for (int i = 0; i < 10; i++) {
      OutMsg* m = bus::take(0);
      if (!m) break;
      if (!mqtt::publish(s_at, m->suffix, m->payload, m->len, m->qos, m->retain)) {
        if (m->persist) offline::store(m);
        else CNT_INC(msg_dropped);
      }
      bus::release(m);
    }
  } else {
    // keep RAM free: persistable data goes to flash until we are back
    while (OutMsg* m = bus::take(0)) {
      if (m->persist) offline::store(m);
      else CNT_INC(msg_dropped);
      bus::release(m);
    }
  }
}

static void onMqttConnected(AtChannel& at) {
  char buf[96];
  snprintf(buf, sizeof buf, "{\"online\":true,\"ts\":%lu,\"fw\":\"%s\"}", (unsigned long)clockx::now(), report::fwVersion());
  mqtt::publish(at, "status", buf, strlen(buf), 1, true);
  String info;
  report::buildInfo(info);
  mqtt::publish(at, "info", info.c_str(), info.length(), 1, true);
}

// ------------------------------------------------------------------ task
static void task(void*) {
  s_hb = sup::registerTask("cell", 90000);
  modemhw::begin();
  s_at.begin(Serial1, PIN_MODEM_RX, PIN_MODEM_TX, 115200);  // modem default; applyBaud() switches
  s_at.setUrcHandler(onUrc);
  s_at.setMqttRxHandler(onMqttRx);
  mqtt::init();
  mqtt::setConnectedHook(onMqttConnected);
  setPhase(NetPhase::Off);

  char cmdBuf[512];
  for (;;) {
    sup::beat(s_hb);
    s_at.poll();

    JobBox* jb;
    while (xQueueReceive(s_jobQ, &jb, 0) == pdTRUE) {
      auto* sp = reinterpret_cast<std::shared_ptr<JobBox>*>(jb);
      sup::setNote(s_hb, "job");
      (*sp)->fn(s_at);
      xSemaphoreGive((*sp)->done);
      delete sp;
      sup::setNote(s_hb, "");
    }

    sup::setNote(s_hb, phaseName(s_phase));
    handleRatRequest();
    netStep();
    bool on = s_phase == NetPhase::Online;
    if (on) {
      mqtt::step(s_at);
      while (mqtt::popCommand(cmdBuf, sizeof cmdBuf)) cmd::handleRemote(cmdBuf);
    }
    deliver();
    if (s_phase >= NetPhase::Sim && !sup::safeMode()) gnss::step(s_at, on && mqtt::connected());
    vTaskDelay(pdMS_TO_TICKS(on ? 50 : 20));
  }
}

void start() {
  s_mtx = xSemaphoreCreateMutex();
  s_jobQ = xQueueCreate(6, sizeof(void*));
  uint64_t mac = ESP.getEfuseMac();
  snprintf(s_devId, sizeof s_devId, "esp-%08lx", (unsigned long)(mac >> 16));
  memset(&s_info, 0, sizeof s_info);
  s_info.csq = 99;
  xTaskCreatePinnedToCore(task, "cell", 16384, nullptr, 5, &s_task, 0);
}

bool exec(Job job, uint32_t timeoutMs) {
  if (xTaskGetCurrentTaskHandle() == s_task) { job(s_at); return true; }
  auto* sp = new std::shared_ptr<JobBox>(std::make_shared<JobBox>());
  (*sp)->fn = std::move(job);
  std::shared_ptr<JobBox> mine = *sp;  // keep alive even if we time out
  if (xQueueSend(s_jobQ, &sp, pdMS_TO_TICKS(1000)) != pdTRUE) { delete sp; return false; }
  return xSemaphoreTake(mine->done, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void dataFailure(const char* why) {
  strlcpy(s_dataFailWhy, why, sizeof s_dataFailWhy);
  f_dataFailure = true;
}
void dataSuccess() { s_atFailStreak = 0; }

void forceRecovery(uint8_t level) { s_forceLevel = constrain(level, 1, 5); }

void setRatMode(int mode) {
  String err;
  char v[4];
  snprintf(v, sizeof v, "%d", mode);
  config::set("rat_mode", v, err);
  s_ratRequest = mode;  // applied by the cellular task
}

static void handleRatRequest() {
  if (s_ratRequest < 0) return;
  s_ratRequest = -1;
  s_forced2G = false;
  s_lteTrial = false;
  if (s_phase >= NetPhase::Register) {
    mqtt::networkLost();
    setPhase(NetPhase::Register);
  }
}

void snapshot(NetInfo& out) {
  Lock l;
  out = s_info;
  out.phase = s_phase;
  out.recoveryLevel = s_level;
  out.forced2G = s_forced2G;
}

void toJson(JsonObject o) {
  NetInfo n;
  snapshot(n);
  o["ph"] = phaseName(n.phase);
  o["rat"] = n.rat;
  o["csq"] = n.csq;
  if (n.rsrp) o["rsrp"] = n.rsrp;
  if (n.rsrq) o["rsrq"] = n.rsrq;
  o["op"] = n.op;
  o["band"] = n.band;
  o["fb2g"] = n.forced2G;
  o["rl"] = n.recoveryLevel;
  o["vbat"] = serialized(String(n.vbat, 2));
  o["mtemp"] = n.modemTemp;
  if (n.lastFailure[0]) o["fail"] = n.lastFailure;
}

void print(Print& out) {
  NetInfo n;
  snapshot(n);
  out.printf("NET phase=%s rat=%s%s op='%s' (%s) band=%s\n", phaseName(n.phase), n.rat, n.forced2G ? " [forced 2G]" : "",
             n.op, n.mccmnc, n.band);
  out.printf("  CSQ %d  RSRP %d dBm  RSRQ %d dB  TAC %s  cell %lu  IP %s\n", n.csq, n.rsrp, n.rsrq, n.tac,
             (unsigned long)n.cellId, n.ip);
  out.printf("  recovery level %u  online %lus  last failure '%s'\n", n.recoveryLevel,
             n.phase == NetPhase::Online ? (unsigned long)((millis() - n.onlineSinceMs) / 1000) : 0UL, n.lastFailure);
  out.printf("  modem %s fw %s  IMEI %s  ICCID %s  VBAT %.2f V  %d C  RI pulses %lu\n", n.model, n.modemFw, n.imei,
             n.iccid, n.vbat, n.modemTemp, (unsigned long)modemhw::ringCount());
}

}  // namespace cell

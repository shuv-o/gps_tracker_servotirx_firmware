#include "services/mqtt_client.h"
#include <freertos/queue.h>
#include "core/clock.h"
#include "core/config.h"
#include "core/counters.h"
#include "core/deadline.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "services/cellular.h"

namespace mqtt {

static const char* TAG = "MQTT";

enum class St : uint8_t { Stopped, Connected };
static volatile St s_st = St::Stopped;
static bool s_started = false;          // CMQTTSTART done
static uint32_t s_nextAttemptMs = 0;
static uint32_t s_fails = 0;
static uint8_t s_broker = 0;            // 0 primary, 1 fallback
static uint32_t s_connectedSince = 0;
static volatile bool s_lost = false;
static int s_lastLostCause = 0;
static char s_lastError[48] = "";
static ConnectedHook s_hook = nullptr;
static QueueHandle_t s_cmdQ;
static bool s_healthyMarked = false;

struct CmdItem { char buf[512]; };

void init() { s_cmdQ = xQueueCreate(4, sizeof(CmdItem)); }
void setConnectedHook(ConnectedHook h) { s_hook = h; }
bool connected() { return s_st == St::Connected && !s_lost; }

void topic(char* out, size_t n, const char* suffix) {
  snprintf(out, n, "%s/%s/%s", cfg().topic_root, cell::deviceId(), suffix);
}

void onUrc(const char* line) {
  if (!strncmp(line, "+CMQTTCONNLOST", 14)) {
    sscanf(line, "+CMQTTCONNLOST: %*d,%d", &s_lastLostCause);
    s_lost = true;
  } else if (!strncmp(line, "+CMQTTNONET", 11)) {
    s_lost = true;
  }
}

void onRx(const char* t, const char* payload, size_t len) {
  char want[96];
  topic(want, sizeof want, "cmd");
  if (strcmp(t, want)) { LOGW(TAG, "message on unexpected topic %s", t); return; }
  CmdItem it;
  size_t n = min(len, sizeof(it.buf) - 1);
  memcpy(it.buf, payload, n);
  it.buf[n] = 0;
  if (xQueueSend(s_cmdQ, &it, 0) != pdTRUE) LOGW(TAG, "command queue full, dropped");
}

bool popCommand(char* buf, size_t n) {
  CmdItem it;
  if (xQueueReceive(s_cmdQ, &it, 0) != pdTRUE) return false;
  strlcpy(buf, it.buf, n);
  return true;
}

static void hardStop(AtChannel& at) {
  at.cmdUrc(8000, "+CMQTTDISC", nullptr, 0, nullptr, "AT+CMQTTDISC=0,60");
  at.cmd(3000, nullptr, "AT+CMQTTREL=0");
  at.cmdUrc(8000, "+CMQTTSTOP", nullptr, 0, nullptr, "AT+CMQTTSTOP");
  s_started = false;
}

static void onConnectFail(const char* why) {
  strlcpy(s_lastError, why, sizeof s_lastError);
  s_fails++;
  CNT_INC(mqtt_connect_fail);
  uint32_t backoff = min<uint32_t>(300, 5u << min<uint32_t>(s_fails - 1, 6));
  backoff += esp_random() % (backoff / 4 + 1);
  s_nextAttemptMs = after(backoff * 1000);
  LOGW(TAG, "connect failed (%s), attempt %lu, retry in %lu s", why, (unsigned long)s_fails, (unsigned long)backoff);
  if (s_fails % 3 == 0 && cfg().mqtt_host2[0]) {
    s_broker ^= 1;
    CNT_INC(broker_failovers);
    LOGW(TAG, "switching to %s broker", s_broker ? "fallback" : "primary");
  }
  if (s_fails >= 3) cell::dataFailure("mqtt connect");
}

static bool sendWithPrompt(AtChannel& at, const char* data, size_t len, const char* fmtCmd, int a, int b) {
  char c[48];
  snprintf(c, sizeof c, fmtCmd, a, b);
  return at.cmdData(10000, data, len, nullptr, "%s", c) == At::OK;
}

static bool connectNow(AtChannel& at) {
  const Config& c = cfg();
  const char* host = s_broker ? c.mqtt_host2 : c.mqtt_host;
  int port = s_broker ? c.mqtt_port2 : c.mqtt_port;
  if (!host[0]) { onConnectFail("no broker host configured"); return false; }

  if (s_started) hardStop(at);
  char urc[64];
  At r = at.cmdUrc(12000, "+CMQTTSTART:", urc, sizeof urc, nullptr, "AT+CMQTTSTART");
  if (r != At::OK && !strstr(urc, "23")) {   // 23 = already started
    // stale session from a previous run: stop and retry once
    hardStop(at);
    r = at.cmdUrc(12000, "+CMQTTSTART:", urc, sizeof urc, nullptr, "AT+CMQTTSTART");
    if (r != At::OK) { onConnectFail("CMQTTSTART"); return false; }
  }
  s_started = true;

  char clientId[48];
  snprintf(clientId, sizeof clientId, "ft-%s", cell::deviceId());
  if (at.cmd(5000, nullptr, "AT+CMQTTACCQ=0,\"%s\",%d", clientId, c.mqtt_tls ? 1 : 0) != At::OK) {
    onConnectFail("CMQTTACCQ");
    return false;
  }
  if (c.mqtt_tls) {
    at.cmd(3000, nullptr, "AT+CSSLCFG=\"sslversion\",0,4");
    at.cmd(3000, nullptr, "AT+CSSLCFG=\"authmode\",0,0");
    at.cmd(3000, nullptr, "AT+CSSLCFG=\"enableSNI\",0,1");
    at.cmd(3000, nullptr, "AT+CMQTTSSLCFG=0,0");
  }

  // Last will: retained "offline" on the status topic
  char t[96], will[96];
  topic(t, sizeof t, "status");
  snprintf(will, sizeof will, "{\"online\":false}");
  if (!sendWithPrompt(at, t, strlen(t), "AT+CMQTTWILLTOPIC=%d,%d", 0, (int)strlen(t)) ||
      !sendWithPrompt(at, will, strlen(will), "AT+CMQTTWILLMSG=%d,%d,1", 0, (int)strlen(will))) {
    LOGW(TAG, "could not set last will");
  }

  LOGI(TAG, "connecting %s:%d (%s)", host, port, c.mqtt_tls ? "tls" : "tcp");
  if (c.mqtt_user[0])
    r = at.cmdUrc(45000, "+CMQTTCONNECT:", urc, sizeof urc, nullptr, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%ld,1,\"%s\",\"%s\"",
                  host, port, (long)c.mqtt_keepalive, c.mqtt_user, c.mqtt_pass);
  else
    r = at.cmdUrc(45000, "+CMQTTCONNECT:", urc, sizeof urc, nullptr, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%ld,1", host,
                  port, (long)c.mqtt_keepalive);
  int cid = -1, err = -1;
  sscanf(urc, "+CMQTTCONNECT: %d,%d", &cid, &err);
  if (r != At::OK || err != 0) {
    char why[96];
    snprintf(why, sizeof why, "CONNECT err %d (%s)", err, at.lastError());
    onConnectFail(why);
    return false;
  }

  // subscribe to commands
  topic(t, sizeof t, "cmd");
  bool sub = sendWithPrompt(at, t, strlen(t), "AT+CMQTTSUB=%d,%d,1", 0, (int)strlen(t)) &&
             at.waitLine("+CMQTTSUB:", 10000, urc, sizeof urc) && strstr(urc, ",0");
  if (!sub) {
    // without the command topic the device cannot be managed remotely: retry the whole session
    hardStop(at);
    onConnectFail("command subscribe failed");
    return false;
  }

  s_st = St::Connected;
  s_lost = false;
  s_fails = 0;
  s_connectedSince = millis();
  s_healthyMarked = false;
  s_lastError[0] = 0;
  CNT_INC(mqtt_connects);
  LOGI(TAG, "connected to %s broker", s_broker ? "fallback" : "primary");
  cell::dataSuccess();
  if (s_hook) s_hook(at);
  return true;
}

void step(AtChannel& at) {
  if (s_lost && s_st == St::Connected) {
    s_st = St::Stopped;
    CNT_INC(mqtt_conn_lost);
    LOGW(TAG, "connection lost (cause %d)", s_lastLostCause);
    s_nextAttemptMs = after(3000);
    bus::event("mqtt_lost", nullptr);
  }
  if (s_st == St::Connected) {
    // confirm a freshly installed OTA image once we have been connected for 30 s
    if (!s_healthyMarked && millis() - s_connectedSince > 30000) {
      s_healthyMarked = true;
      sup::markHealthy();
    }
    return;
  }
  if (!due(s_nextAttemptMs)) return;
  s_lost = false;
  connectNow(at);
}

void networkLost() {
  if (s_st == St::Connected) CNT_INC(mqtt_conn_lost);
  s_st = St::Stopped;
  s_started = false;  // modem stack is gone with the network / reset
  s_nextAttemptMs = after(2000);
}

void reconnectNow() {
  s_lost = true;
  s_nextAttemptMs = 0;
}

void disconnect(AtChannel& at) {
  if (s_started) hardStop(at);
  s_st = St::Stopped;
}

bool publish(AtChannel& at, const char* suffix, const char* payload, size_t len, uint8_t qos, bool retain) {
  if (!connected()) return false;
  char t[96];
  topic(t, sizeof t, suffix);
  char urc[48] = "";
  bool ok = sendWithPrompt(at, t, strlen(t), "AT+CMQTTTOPIC=%d,%d", 0, (int)strlen(t)) &&
            sendWithPrompt(at, payload, len, "AT+CMQTTPAYLOAD=%d,%d", 0, (int)len) &&
            at.cmdUrc(qos ? 20000 : 8000, "+CMQTTPUB:", urc, sizeof urc, nullptr, "AT+CMQTTPUB=0,%d,60%s", qos,
                      retain ? ",1" : "") == At::OK &&
            strstr(urc, ",0") != nullptr;
  if (ok) {
    CNT_INC(mqtt_pub_ok);
    LOGD(TAG, "pub %s (%u B)", suffix, (unsigned)len);
  } else {
    CNT_INC(mqtt_pub_fail);
    strlcpy(s_lastError, at.lastError(), sizeof s_lastError);
    LOGW(TAG, "publish %s failed: %s %s", suffix, at.lastError(), urc);
    s_lost = true;  // force a clean reconnect
  }
  return ok;
}

void toJson(JsonObject o) {
  o["conn"] = connected();
  o["broker"] = s_broker;
  o["up_s"] = connected() ? (millis() - s_connectedSince) / 1000 : 0;
  o["fails"] = s_fails;
  if (s_lastError[0]) o["err"] = s_lastError;
}

void print(Print& out) {
  const Config& c = cfg();
  out.printf("MQTT %s  broker=%s (%s:%ld)  connected %lus  fails=%lu  last error='%s'\n",
             connected() ? "CONNECTED" : "disconnected", s_broker ? "fallback" : "primary",
             s_broker ? c.mqtt_host2 : c.mqtt_host, (long)(s_broker ? c.mqtt_port2 : c.mqtt_port),
             connected() ? (unsigned long)((millis() - s_connectedSince) / 1000) : 0UL, (unsigned long)s_fails,
             s_lastError);
  char t[96];
  topic(t, sizeof t, "#");
  out.printf("topics: %s\n", t);
}

}  // namespace mqtt

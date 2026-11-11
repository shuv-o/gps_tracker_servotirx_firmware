#include "services/ota.h"
#include <Update.h>
#include "core/counters.h"
#include "core/log.h"
#include "core/msgbus.h"
#include "core/supervisor.h"
#include "drivers/status_led.h"
#include "fw_version.h"
#include "services/mqtt_client.h"

namespace ota {

static const char* TAG = "OTA";
static const size_t CHUNK = 32 * 1024;
static const size_t READ_BLOCK = 2048;
static volatile bool s_busy = false;
static volatile uint8_t s_pct = 0;

bool inProgress() { return s_busy; }
uint8_t progress() { return s_pct; }

static void notify(AtChannel& at, const char* state, const char* extra) {
  char buf[200];
  snprintf(buf, sizeof buf, "{\"ev\":\"ota\",\"state\":\"%s\",\"pct\":%u%s%s}", state, s_pct, extra ? "," : "",
           extra ? extra : "");
  // publish directly: the delivery loop is blocked while we download
  mqtt::publish(at, "event", buf, strlen(buf), 0, false);
}

// One HTTP GET of [from, to] into the modem buffer. Returns HTTP status, fills total size.
static int fetchRange(AtChannel& at, const char* url, size_t from, size_t to, long* bodyLen, long* total) {
  at.cmd(3000, nullptr, "AT+HTTPTERM");
  if (at.cmd(5000, nullptr, "AT+HTTPINIT") != At::OK) return -1;
  at.cmd(3000, nullptr, "AT+HTTPPARA=\"URL\",\"%s\"", url);
  if (!strncmp(url, "https", 5)) {
    at.cmd(3000, nullptr, "AT+CSSLCFG=\"sslversion\",0,4");
    at.cmd(3000, nullptr, "AT+CSSLCFG=\"authmode\",0,0");
    at.cmd(3000, nullptr, "AT+CSSLCFG=\"enableSNI\",0,1");
    at.cmd(3000, nullptr, "AT+HTTPPARA=\"SSLCFG\",0");
  }
  at.cmd(3000, nullptr, "AT+HTTPPARA=\"USERDATA\",\"Range: bytes=%u-%u\"", (unsigned)from, (unsigned)to);
  char urc[64];
  if (at.cmdUrc(90000, "+HTTPACTION:", urc, sizeof urc, nullptr, "AT+HTTPACTION=0") != At::OK) return -2;
  int method = 0, status = 0;
  *bodyLen = 0;
  sscanf(urc, "+HTTPACTION: %d,%d,%ld", &method, &status, bodyLen);
  *total = 0;
  if (status == 206) {
    AtResp h;
    if (at.cmd(5000, &h, "AT+HTTPHEAD") == At::OK) {
      const char* p = strcasestr(h.buf, "Content-Range:");
      if (p) {
        const char* slash = strchr(p, '/');
        if (slash) *total = atol(slash + 1);
      }
    }
  } else if (status == 200) {
    *total = *bodyLen;  // server ignored Range: whole file is in the modem buffer
  }
  return status;
}

bool run(AtChannel& at, const Request& req, String& err) {
  if (s_busy) { err = "already running"; return false; }
  if (!req.force && req.version[0] && !strcmp(req.version, FW_VERSION)) {
    err = "already on version " FW_VERSION;
    return false;
  }
  s_busy = true;
  s_pct = 0;
  led::set(led::OTA);
  LOGI(TAG, "start %s", req.url);
  notify(at, "start", nullptr);

  static uint8_t buf[READ_BLOCK];
  size_t done = 0;
  long total = -1;
  bool wholeFileMode = false;
  bool ok = true;

  while (ok && (total < 0 || done < (size_t)total)) {
    size_t to = done + CHUNK - 1;
    if (total > 0 && to >= (size_t)total) to = total - 1;
    long bodyLen = 0, t = 0;
    int status = -1;
    for (int attempt = 1; attempt <= 3; attempt++) {
      status = fetchRange(at, req.url, done, to, &bodyLen, &t);
      if (status == 206 || status == 200) break;
      LOGW(TAG, "range %u-%u attempt %d: status %d", (unsigned)done, (unsigned)to, attempt, status);
      sup::feed();
      delay(2000);
    }
    if (status != 206 && status != 200) { err = "HTTP status " + String(status); ok = false; break; }

    if (total < 0) {  // first chunk: learn the size and open the partition
      total = t;
      wholeFileMode = status == 200;
      if (total <= 0) { err = "unknown file size"; ok = false; break; }
      if (!Update.begin(total)) { err = "image too large / Update.begin failed"; ok = false; break; }
      if (req.md5[0] && !Update.setMD5(req.md5)) { err = "bad md5 string"; ok = false; break; }
      LOGI(TAG, "image %ld bytes%s", total, wholeFileMode ? " (no Range support, single download)" : "");
    }

    // copy this response out of the modem buffer
    size_t respLen = bodyLen;
    if (respLen == 0) { err = "server returned an empty range"; ok = false; break; }
    size_t ofs = 0;
    while (ofs < respLen) {
      size_t want = min(READ_BLOCK, respLen - ofs);
      size_t got = 0;
      At r = At::ERROR;
      for (int attempt = 0; attempt < 3 && r != At::OK; attempt++)
        r = at.cmdReadBlock(10000, "+HTTPREAD:", buf, want, &got, "AT+HTTPREAD=%u,%u", (unsigned)ofs, (unsigned)want);
      if (r != At::OK || got == 0) { err = "HTTPREAD failed at " + String(done + ofs); ok = false; break; }
      if (Update.write(buf, got) != got) { err = String("flash write: ") + Update.errorString(); ok = false; break; }
      ofs += got;
      uint8_t pct = (uint8_t)(((done + ofs) * 100ULL) / total);
      if (pct / 10 != s_pct / 10) { s_pct = pct; LOGI(TAG, "%u%%", pct); notify(at, "progress", nullptr); }
      s_pct = pct;
      sup::feed();
    }
    done += ofs;
    if (wholeFileMode) break;
  }
  at.cmd(3000, nullptr, "AT+HTTPTERM");

  if (ok && done != (size_t)total) { err = "incomplete " + String(done) + "/" + String(total); ok = false; }
  if (ok && !Update.end(true)) { err = String("verify failed: ") + Update.errorString(); ok = false; }
  if (!ok) {
    Update.abort();
    CNT_INC(ota_fail);
    LOGE(TAG, "failed: %s", err.c_str());
    char e[120];
    snprintf(e, sizeof e, "\"err\":\"%s\"", err.c_str());
    notify(at, "failed", e);
    s_busy = false;
    return false;
  }
  s_pct = 100;
  LOGI(TAG, "image written and verified, rebooting into new firmware");
  notify(at, "installed", "\"reboot\":true");
  delay(1500);
  sup::reboot("ota installed");
  return true;
}

}  // namespace ota

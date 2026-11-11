#include "drivers/at_channel.h"
#include <stdarg.h>
#include "core/counters.h"
#include "core/log.h"
#include "core/supervisor.h"

static const char* TAG = "AT";

// Lines that can arrive at any time. A line that starts with the running
// command's own prefix (e.g. "+CPIN" for AT+CPIN?) is treated as its response.
static const char* const URC_PREFIXES[] = {
    "+CMQTTCONNLOST", "+CMQTTNONET", "+CMQTTRXSTART", "+CMQTTPING", "+CMTI", "+CGEV", "RING", "*ATREADY",
    "RDY", "+CPIN", "SMS DONE", "PB DONE", "+CGNSSPWR", "NO CARRIER", "+CREG", "+CEREG", "+CGREG",
    "+CTZV", "*PSUTTZ", "DST:", "+HTTP_PEER_CLOSED", "+HTTP_NONET_EVENT", "+IPCLOSE", "+CIPEVENT",
    "+CLIP", "+CPMUTEMP", "+NETCLOSE"};

// ---------------------------------------------------------------- AtResp
void AtResp::add(const char* line) {
  size_t n = strlen(line);
  if (len + n + 2 >= sizeof buf) return;  // keep what fits
  memcpy(buf + len, line, n);
  len += n;
  buf[len++] = '\n';
  buf[len] = 0;
}

bool AtResp::get(const char* prefix, char* out, size_t outLen, int nth) const {
  size_t pl = strlen(prefix);
  const char* p = buf;
  while (p && *p) {
    if (!strncmp(p, prefix, pl)) {
      if (nth-- == 0) {
        const char* s = p + pl;
        while (*s == ' ') s++;
        const char* e = strchr(s, '\n');
        size_t n = e ? (size_t)(e - s) : strlen(s);
        if (n >= outLen) n = outLen - 1;
        memcpy(out, s, n);
        out[n] = 0;
        return true;
      }
    }
    p = strchr(p, '\n');
    if (p) p++;
  }
  if (outLen) out[0] = 0;
  return false;
}

bool AtResp::first(char* out, size_t outLen) const {
  const char* p = buf;
  while (p && *p) {
    const char* e = strchr(p, '\n');
    size_t n = e ? (size_t)(e - p) : strlen(p);
    if (!(n == 2 && !strncmp(p, "OK", 2)) && n) {
      if (n >= outLen) n = outLen - 1;
      memcpy(out, p, n);
      out[n] = 0;
      return true;
    }
    p = e ? e + 1 : nullptr;
  }
  out[0] = 0;
  return false;
}

// ---------------------------------------------------------------- AtChannel
void AtChannel::begin(HardwareSerial& port, int rxPin, int txPin, uint32_t baud) {
  _port = &port;
  _port->setRxBufferSize(8192);
  _port->begin(baud, SERIAL_8N1, rxPin, txPin);
  _err[0] = 0;
}

void AtChannel::setBaud(uint32_t baud) { _port->updateBaudRate(baud); }

AtChannel::ReadKind AtChannel::readLine(uint32_t timeoutMs, bool allowPrompt) {
  uint32_t t0 = millis();
  for (;;) {
    while (_port->available()) {
      char c = (char)_port->read();
      _lastRx = millis();
      if (c == '\r') continue;
      if (c == '\n') {
        if (_lineLen == 0) continue;
        _line[_lineLen] = 0;
        _lineLen = 0;
        LOGV(TAG, "<< %s", _line);
        return LINE;
      }
      if (allowPrompt && _lineLen == 0 && c == '>') {
        // prompt is "> " without newline
        if (_port->available() && _port->peek() == ' ') _port->read();
        return PROMPT;
      }
      if (_lineLen < sizeof(_line) - 1) _line[_lineLen++] = c;
    }
    if (millis() - t0 >= timeoutMs) return NONE;
    sup::feed();
    vTaskDelay(1);
  }
}

bool AtChannel::isUrc(const char* line) const {
  if (_cmdPrefix[0] && !strncmp(line, _cmdPrefix, strlen(_cmdPrefix))) return false;
  for (const char* p : URC_PREFIXES)
    if (!strncmp(line, p, strlen(p))) return true;
  return false;
}

void AtChannel::dispatchUrc(const char* line) {
  if (!strncmp(line, "+CMQTTRXSTART", 13)) { handleMqttRx(); return; }
  if (_urc) _urc(line);
}

size_t AtChannel::readRaw(uint8_t* dst, size_t n, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < n && millis() - t0 < timeoutMs) {
    int a = _port->available();
    if (a > 0) {
      size_t k = _port->read(dst + got, min((size_t)a, n - got));
      got += k;
      _lastRx = millis();
    } else {
      sup::feed();
      vTaskDelay(1);
    }
  }
  return got;
}

// +CMQTTRXSTART: 0,<topic_len>,<payload_len>
// +CMQTTRXTOPIC: 0,<len>\r\n<topic>\r\n
// +CMQTTRXPAYLOAD: 0,<len>\r\n<payload>\r\n   (may repeat for long payloads)
// +CMQTTRXEND: 0
void AtChannel::handleMqttRx() {
  int c = 0, tl = 0, pl = 0;
  sscanf(_line, "+CMQTTRXSTART: %d,%d,%d", &c, &tl, &pl);
  static char topic[160];
  static char payload[4096];
  size_t topicLen = 0, payloadLen = 0;
  bool overflow = false;
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    if (readLine(1000, false) != LINE) continue;
    int len = 0;
    if (!strncmp(_line, "+CMQTTRXTOPIC:", 14)) {
      sscanf(_line, "+CMQTTRXTOPIC: %*d,%d", &len);
      size_t take = min((size_t)len, sizeof(topic) - 1);
      topicLen = readRaw((uint8_t*)topic, take, 2000);
      if ((size_t)len > take) {  // topic longer than buffer: discard the rest
        uint8_t junk[32];
        size_t rest = len - take;
        while (rest) { size_t k = readRaw(junk, min(rest, sizeof junk), 1000); if (!k) break; rest -= k; }
      }
      topic[topicLen] = 0;
    } else if (!strncmp(_line, "+CMQTTRXPAYLOAD:", 16)) {
      sscanf(_line, "+CMQTTRXPAYLOAD: %*d,%d", &len);
      size_t room = sizeof(payload) - 1 - payloadLen;
      size_t take = min((size_t)len, room);
      payloadLen += readRaw((uint8_t*)payload + payloadLen, take, 3000);
      if ((size_t)len > take) {
        overflow = true;
        uint8_t junk[64];
        size_t rest = len - take;
        while (rest) { size_t k = readRaw(junk, min(rest, sizeof junk), 1000); if (!k) break; rest -= k; }
      }
    } else if (!strncmp(_line, "+CMQTTRXEND:", 12)) {
      payload[payloadLen] = 0;
      CNT_INC(mqtt_rx);
      if (overflow) LOGW(TAG, "MQTT RX payload truncated to %u bytes", (unsigned)payloadLen);
      if (_mqttRx) _mqttRx(topic, payload, payloadLen);
      return;
    } else if (isUrc(_line)) {
      if (_urc) _urc(_line);
    }
  }
  LOGW(TAG, "incomplete MQTT RX sequence");
}

void AtChannel::poll() {
  while (_port->available()) {
    if (readLine(50, false) != LINE) break;
    // Anything arriving with no command running is unsolicited.
    _cmdPrefix[0] = 0;
    dispatchUrc(_line);
  }
}

void AtChannel::sendCmd(const char* fmt, va_list ap) {
  poll();  // deliver pending URCs first
  char buf[512];
  vsnprintf(buf, sizeof buf, fmt, ap);
  strlcpy(_cmd, buf, sizeof _cmd);
  // own response prefix: AT+XYZ=.. / AT+XYZ? -> "+XYZ"
  _cmdPrefix[0] = 0;
  if (!strncmp(buf, "AT+", 3) || !strncmp(buf, "AT*", 3)) {
    size_t i = 0;
    const char* s = buf + 2;
    while (s[i] && s[i] != '=' && s[i] != '?' && i < sizeof(_cmdPrefix) - 1) { _cmdPrefix[i] = s[i]; i++; }
    _cmdPrefix[i] = 0;
  }
  _err[0] = 0;
  LOGV(TAG, ">> %s", buf);
  _port->print(buf);
  _port->print('\r');
}

At AtChannel::collect(uint32_t timeoutMs, AtResp* resp, const char* urcPrefix, char* urcOut, size_t urcLen) {
  bool gotOk = false, gotUrc = urcPrefix == nullptr;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    uint32_t el = millis() - t0;
    if (el >= timeoutMs || readLine(timeoutMs - el, false) != LINE) break;
    if (!strcmp(_line, _cmd)) continue;  // echo
    if (urcPrefix && !strncmp(_line, urcPrefix, strlen(urcPrefix))) {
      if (urcOut && !gotUrc) strlcpy(urcOut, _line, urcLen);
      gotUrc = true;
      if (resp) resp->add(_line);
    } else if (isUrc(_line)) {
      dispatchUrc(_line);
      continue;
    } else if (!strcmp(_line, "OK")) {
      gotOk = true;
    } else if (!strcmp(_line, "ERROR") || !strncmp(_line, "+CME ERROR", 10) || !strncmp(_line, "+CMS ERROR", 10)) {
      strlcpy(_err, _line, sizeof _err);
      if (resp) resp->add(_line);
      CNT_INC(at_errors);
      LOGD(TAG, "%s -> %s", _cmd, _line);
      _cmdPrefix[0] = 0;
      return At::ERROR;
    } else if (resp) {
      resp->add(_line);
    }
    if (gotOk && gotUrc) { _cmdPrefix[0] = 0; return At::OK; }
  }
  snprintf(_err, sizeof _err, gotOk ? "TIMEOUT waiting %s" : "TIMEOUT", urcPrefix ? urcPrefix : "");
  CNT_INC(at_timeouts);
  LOGD(TAG, "%s -> %s", _cmd, _err);
  _cmdPrefix[0] = 0;
  return At::TIMEOUT;
}

At AtChannel::cmd(uint32_t timeoutMs, AtResp* resp, const char* fmt, ...) {
  if (resp) resp->clear();
  va_list ap;
  va_start(ap, fmt);
  sendCmd(fmt, ap);
  va_end(ap);
  return collect(timeoutMs, resp, nullptr, nullptr, 0);
}

At AtChannel::cmdUrc(uint32_t timeoutMs, const char* urcPrefix, char* urcOut, size_t urcLen, AtResp* resp,
                     const char* fmt, ...) {
  if (resp) resp->clear();
  if (urcOut && urcLen) urcOut[0] = 0;
  va_list ap;
  va_start(ap, fmt);
  sendCmd(fmt, ap);
  va_end(ap);
  return collect(timeoutMs, resp, urcPrefix, urcOut, urcLen);
}

At AtChannel::cmdData(uint32_t timeoutMs, const void* data, size_t len, AtResp* resp, const char* fmt, ...) {
  if (resp) resp->clear();
  va_list ap;
  va_start(ap, fmt);
  sendCmd(fmt, ap);
  va_end(ap);
  uint32_t t0 = millis();
  for (;;) {
    uint32_t el = millis() - t0;
    ReadKind k = el >= 5000 ? NONE : readLine(5000 - el, true);  // prompt must come within 5 s
    if (k == PROMPT) break;
    if (k == NONE) {
      _port->write((uint8_t)0x1B);  // cancel: modem may enter data mode late and eat our next command
      strlcpy(_err, "no prompt", sizeof _err);
      CNT_INC(at_timeouts);
      _cmdPrefix[0] = 0;
      return At::TIMEOUT;
    }
    if (!strcmp(_line, _cmd)) continue;
    if (!strcmp(_line, "ERROR") || !strncmp(_line, "+CME ERROR", 10)) {
      strlcpy(_err, _line, sizeof _err);
      CNT_INC(at_errors);
      _cmdPrefix[0] = 0;
      return At::ERROR;
    }
    if (isUrc(_line)) dispatchUrc(_line);
  }
  _port->write((const uint8_t*)data, len);
  return collect(timeoutMs, resp, nullptr, nullptr, 0);
}

bool AtChannel::waitLine(const char* prefix, uint32_t timeoutMs, char* out, size_t outLen) {
  _cmdPrefix[0] = 0;
  uint32_t t0 = millis();
  size_t pl = strlen(prefix);
  while (millis() - t0 < timeoutMs) {
    uint32_t el = millis() - t0;
    if (el >= timeoutMs || readLine(timeoutMs - el, false) != LINE) return false;
    if (!strncmp(_line, prefix, pl)) {
      if (out) strlcpy(out, _line, outLen);
      return true;
    }
    if (isUrc(_line)) dispatchUrc(_line);
  }
  return false;
}

bool AtChannel::ping(uint32_t timeoutMs) { return cmd(timeoutMs, nullptr, "AT") == At::OK; }

// AT+HTTPREAD=<ofs>,<len> on A76xx answers:
//   OK \r\n +HTTPREAD: <n> \r\n <n bytes> \r\n +HTTPREAD: 0
// (some firmware sends the data before OK). Both orders are handled.
At AtChannel::cmdReadBlock(uint32_t timeoutMs, const char* hdrPrefix, uint8_t* dst, size_t cap, size_t* got,
                           const char* fmt, ...) {
  *got = 0;
  va_list ap;
  va_start(ap, fmt);
  sendCmd(fmt, ap);
  va_end(ap);
  size_t hl = strlen(hdrPrefix);
  bool sawOk = false, sawEnd = false;
  uint32_t t0 = millis(), dataAt = 0;
  while (millis() - t0 < timeoutMs) {
    // after OK + data, give the trailing "+HTTPREAD: 0" a short grace period only
    uint32_t el = millis() - t0;
    if (el >= timeoutMs) break;
    uint32_t wait = (sawOk && *got) ? 300 : timeoutMs - el;
    if (readLine(wait, false) != LINE) {
      if (sawOk && *got) break;
      continue;
    }
    if (!strcmp(_line, _cmd)) continue;
    if (!strncmp(_line, hdrPrefix, hl)) {
      long n = atol(_line + hl);
      if (n <= 0) { sawEnd = true; }
      else {
        size_t take = min((size_t)n, cap - *got);
        size_t k = readRaw(dst + *got, take, 5000);
        *got += k;
        dataAt = millis();
        if (k < take) { strlcpy(_err, "short block", sizeof _err); _cmdPrefix[0] = 0; return At::TIMEOUT; }
        if ((size_t)n > take) {  // more than requested: drain
          uint8_t junk[64];
          size_t rest = n - take;
          while (rest) { size_t j = readRaw(junk, min(rest, sizeof junk), 1000); if (!j) break; rest -= j; }
        }
      }
    } else if (!strcmp(_line, "OK")) {
      sawOk = true;
    } else if (!strcmp(_line, "ERROR") || !strncmp(_line, "+CME ERROR", 10)) {
      strlcpy(_err, _line, sizeof _err);
      CNT_INC(at_errors);
      _cmdPrefix[0] = 0;
      return At::ERROR;
    } else if (isUrc(_line)) {
      dispatchUrc(_line);
    }
    if (sawOk && sawEnd && *got) break;
  }
  (void)dataAt;
  _cmdPrefix[0] = 0;
  if (*got && sawOk) return At::OK;
  strlcpy(_err, "TIMEOUT", sizeof _err);
  CNT_INC(at_timeouts);
  return At::TIMEOUT;
}

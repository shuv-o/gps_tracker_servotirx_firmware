#include "modem_at.h"

static bool isFinalError(const String& l) {
  return l == "ERROR" || l.startsWith("+CME ERROR") || l.startsWith("+CMS ERROR");
}

bool Modem::readLine(String& out, uint32_t timeoutMs, bool log) {
  uint32_t t0 = millis();
  for (;;) {
    while (_p->available()) {
      char c = (char)_p->read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (_buf.length() == 0) continue;
        out = _buf;
        _buf = "";
        if (log && _verbose) Serial.printf("      << %s\n", out.c_str());
        return true;
      }
      _buf += c;
      if (_buf.length() > 1024) {  // runaway line protection
        out = _buf;
        _buf = "";
        return true;
      }
    }
    if (millis() - t0 >= timeoutMs) return false;
    delay(1);
  }
}

void Modem::drain() {
  String l;
  while (_p->available()) {
    if (!readLine(l, 30)) break;
  }
}

AtResult Modem::cmd(const char* command, uint32_t timeoutMs, String* resp) {
  drain();
  lastError = "";
  if (_verbose) Serial.printf("      >> %s\n", command);
  _p->print(command);
  _p->print('\r');

  String acc, line;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!readLine(line, timeoutMs - (millis() - t0))) break;
    if (line == command) continue;  // echo
    acc += line;
    acc += '\n';
    if (line == "OK") {
      if (resp) *resp = acc;
      return AtResult::OK;
    }
    if (isFinalError(line)) {
      lastError = line;
      if (resp) *resp = acc;
      return AtResult::ERROR;
    }
  }
  lastError = "TIMEOUT (no reply)";
  if (resp) *resp = acc;
  return AtResult::TIMEOUT;
}

AtResult Modem::cmdUrc(const char* command, const char* urcPrefix, uint32_t timeoutMs,
                       String* urcLine, String* resp) {
  drain();
  lastError = "";
  if (_verbose) Serial.printf("      >> %s\n", command);
  _p->print(command);
  _p->print('\r');

  bool gotOk = false, gotUrc = false;
  String acc, line;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!readLine(line, timeoutMs - (millis() - t0))) break;
    if (line == command) continue;
    acc += line;
    acc += '\n';
    if (line == "OK") gotOk = true;
    else if (isFinalError(line)) {
      lastError = line;
      if (resp) *resp = acc;
      return AtResult::ERROR;
    } else if (line.startsWith(urcPrefix)) {
      if (!gotUrc && urcLine) *urcLine = line;  // keep the FIRST matching line
      gotUrc = true;
    }
    if (gotOk && gotUrc) {
      if (resp) *resp = acc;
      return AtResult::OK;
    }
  }
  lastError = gotOk ? String("TIMEOUT waiting for ") + urcPrefix : "TIMEOUT (no reply)";
  if (resp) *resp = acc;
  return AtResult::TIMEOUT;
}

bool Modem::waitLine(const char* prefix, uint32_t timeoutMs, String* line) {
  String l;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!readLine(l, timeoutMs - (millis() - t0))) return false;
    if (l.startsWith(prefix)) {
      if (line) *line = l;
      return true;
    }
  }
  return false;
}

String Modem::field(const String& resp, const char* prefix) {
  int p = resp.indexOf(prefix);
  if (p < 0) return "";
  p += strlen(prefix);
  int e = resp.indexOf('\n', p);
  String v = resp.substring(p, e < 0 ? resp.length() : e);
  v.trim();
  return v;
}

String Modem::firstLine(const String& resp) {
  int s = 0;
  while (s < (int)resp.length()) {
    int e = resp.indexOf('\n', s);
    if (e < 0) e = resp.length();
    String l = resp.substring(s, e);
    l.trim();
    if (l.length() && l != "OK") return l;
    s = e + 1;
  }
  return "";
}

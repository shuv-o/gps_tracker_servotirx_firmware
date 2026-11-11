#pragma once
// AT command channel for the SIMCom A76xx.
//
// NOT thread safe by design: only the cellular task talks to the modem.
// Other tasks send requests to the cellular task (see services/cellular.h).
//
// Features
//  - printf-style commands with timeouts, final result detection
//  - "OK + result URC" commands (HTTPACTION, CMQTTCONNECT, NETOPEN ...)
//  - '>' prompt commands with binary payload (CMQTTTOPIC/PAYLOAD ...)
//  - unsolicited result codes are routed to a handler even mid-command
//  - incoming MQTT messages (+CMQTTRX...) are read by exact length
#include <Arduino.h>

enum class At : uint8_t { OK, ERROR, TIMEOUT };

// Collected response lines of one command ('\n' separated).
class AtResp {
 public:
  char   buf[1536];
  size_t len = 0;
  void clear() { len = 0; buf[0] = 0; }
  void add(const char* line);
  // Copy the text after `prefix` from the n-th matching line. Returns false if absent.
  bool get(const char* prefix, char* out, size_t outLen, int nth = 0) const;
  bool has(const char* text) const { return strstr(buf, text) != nullptr; }
  // First line that is not "OK"
  bool first(char* out, size_t outLen) const;
};

typedef void (*UrcHandler)(const char* line);
typedef void (*MqttRxHandler)(const char* topic, const char* payload, size_t len);

class AtChannel {
 public:
  void begin(HardwareSerial& port, int rxPin, int txPin, uint32_t baud);
  void setBaud(uint32_t baud);
  void setUrcHandler(UrcHandler h) { _urc = h; }
  void setMqttRxHandler(MqttRxHandler h) { _mqttRx = h; }

  // Send command, wait for OK / ERROR / +CME ERROR.
  At cmd(uint32_t timeoutMs, AtResp* resp, const char* fmt, ...) __attribute__((format(printf, 4, 5)));
  // Send command, wait for OK *and* a line starting with urcPrefix (copied to urcOut).
  At cmdUrc(uint32_t timeoutMs, const char* urcPrefix, char* urcOut, size_t urcLen, AtResp* resp,
            const char* fmt, ...) __attribute__((format(printf, 7, 8)));
  // Send command, wait for '>' prompt, write data, wait for final result.
  At cmdData(uint32_t timeoutMs, const void* data, size_t len, AtResp* resp, const char* fmt, ...)
      __attribute__((format(printf, 6, 7)));
  // Command that returns "<hdrPrefix> <n>" followed by n raw bytes (e.g. AT+HTTPREAD).
  // Copies up to cap bytes into dst, *got = bytes received.
  At cmdReadBlock(uint32_t timeoutMs, const char* hdrPrefix, uint8_t* dst, size_t cap, size_t* got,
                  const char* fmt, ...) __attribute__((format(printf, 7, 8)));
  // Wait for a line starting with prefix (others are dispatched as URCs).
  bool waitLine(const char* prefix, uint32_t timeoutMs, char* out = nullptr, size_t outLen = 0);
  // Read exactly n raw bytes (binary safe).
  size_t readRaw(uint8_t* dst, size_t n, uint32_t timeoutMs);
  // Dispatch whatever unsolicited data is pending. Non blocking.
  void poll();
  // Quick "AT" check.
  bool ping(uint32_t timeoutMs = 600);
  // Raw passthrough for the debug console (bypasses parsing).
  void writeRaw(const char* s) { _port->print(s); }

  const char* lastError() const { return _err; }
  uint32_t lastRxMs() const { return _lastRx; }

 private:
  enum ReadKind : uint8_t { NONE, LINE, PROMPT };
  ReadKind readLine(uint32_t timeoutMs, bool allowPrompt);
  void sendCmd(const char* fmt, va_list ap);
  bool isUrc(const char* line) const;
  void dispatchUrc(const char* line);
  void handleMqttRx();
  At collect(uint32_t timeoutMs, AtResp* resp, const char* urcPrefix, char* urcOut, size_t urcLen);

  HardwareSerial* _port = nullptr;
  UrcHandler _urc = nullptr;
  MqttRxHandler _mqttRx = nullptr;
  char _line[600];
  size_t _lineLen = 0;
  char _cmd[64];        // command being executed (for echo + own-response detection)
  char _cmdPrefix[24];  // e.g. "+CPIN" for AT+CPIN?
  char _err[64];
  uint32_t _lastRx = 0;
};

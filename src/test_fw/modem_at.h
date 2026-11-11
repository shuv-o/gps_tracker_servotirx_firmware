#pragma once
#include <Arduino.h>

enum class AtResult { OK, ERROR, TIMEOUT };

// Minimal line-based AT command driver for SIMCom A76xx.
class Modem {
public:
  void begin(HardwareSerial& port) { _p = &port; }
  void setVerbose(bool v) { _verbose = v; }
  bool verbose() const { return _verbose; }

  // Send a command and wait for the final result (OK / ERROR / +CME ERROR / +CMS ERROR).
  AtResult cmd(const char* command, uint32_t timeoutMs = 2000, String* resp = nullptr);

  // Send a command and wait for BOTH "OK" and a URC line starting with urcPrefix
  // (in any order). The URC line is returned in urcLine.
  AtResult cmdUrc(const char* command, const char* urcPrefix, uint32_t timeoutMs,
                  String* urcLine, String* resp = nullptr);

  // Wait for a line beginning with prefix.
  bool waitLine(const char* prefix, uint32_t timeoutMs, String* line = nullptr);

  // Read one complete line (CR/LF stripped, empty lines skipped).
  bool readLine(String& out, uint32_t timeoutMs, bool log = true);

  // Discard (and log) anything already waiting in the RX buffer.
  void drain();

  void sendRaw(const String& s) { _p->print(s); }

  // Return text after `prefix` on the first matching line of resp ("" if none).
  static String field(const String& resp, const char* prefix);
  // First line of resp that is not "OK".
  static String firstLine(const String& resp);

  String lastError;   // "+CME ERROR: ...", "ERROR", "TIMEOUT", ...

private:
  HardwareSerial* _p = nullptr;
  bool _verbose = true;
  String _buf;
};

#pragma once
// Thread-safe logger: USB serial + RAM ring buffer (retrievable over MQTT with "logs").
//
//   LOGE("NET", "registration failed: %s", why);
//   LOGI("OBD", "protocol %s", name);
//
// Levels: 0 off, 1 error, 2 warn, 3 info, 4 debug, 5 verbose (AT traffic)
#include <Arduino.h>

enum LogLevel : uint8_t { LOG_OFF = 0, LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_VERBOSE };

namespace logx {
void begin(size_t ringBytes = 32 * 1024);
void setLevel(uint8_t lvl);
uint8_t level();
void setSerialEcho(bool on);      // off while the console is in AT passthrough
void write(uint8_t lvl, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
// Copy the newest `maxBytes` of the ring into out (null terminated). Returns length.
size_t tail(char* out, size_t maxBytes);
uint32_t errorCount();
uint32_t warnCount();
}  // namespace logx

#define LOGE(tag, ...) do { if (logx::level() >= LOG_ERROR)   logx::write(LOG_ERROR,   tag, __VA_ARGS__); } while (0)
#define LOGW(tag, ...) do { if (logx::level() >= LOG_WARN)    logx::write(LOG_WARN,    tag, __VA_ARGS__); } while (0)
#define LOGI(tag, ...) do { if (logx::level() >= LOG_INFO)    logx::write(LOG_INFO,    tag, __VA_ARGS__); } while (0)
#define LOGD(tag, ...) do { if (logx::level() >= LOG_DEBUG)   logx::write(LOG_DEBUG,   tag, __VA_ARGS__); } while (0)
#define LOGV(tag, ...) do { if (logx::level() >= LOG_VERBOSE) logx::write(LOG_VERBOSE, tag, __VA_ARGS__); } while (0)

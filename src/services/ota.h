#pragma once
// Firmware update over the cellular link (runs on the cellular task).
//
//  - Downloads with HTTP(S) Range requests in 32 KB chunks through the modem
//    (each chunk retried 3x, so a weak link does not restart the whole download)
//  - Streams into the inactive OTA partition, optional MD5 check
//  - Reboots into the new image in PENDING_VERIFY state; the supervisor
//    confirms it after MQTT works, otherwise the bootloader rolls back.
//
// Trigger: MQTT  {"cmd":"ota https://host/fw.bin <md5> [force]"}
//          console: ota <url> [md5] [force]
#include <Arduino.h>
#include "drivers/at_channel.h"

namespace ota {
struct Request {
  char url[200];
  char md5[33];
  char version[24];
  bool force;
};
// Blocking. Returns false with reason on failure; on success it reboots.
bool run(AtChannel& at, const Request& req, String& err);
bool inProgress();
uint8_t progress();   // 0..100
}  // namespace ota

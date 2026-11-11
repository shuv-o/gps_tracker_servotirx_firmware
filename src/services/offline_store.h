#pragma once
// Store-and-forward queue in flash (LittleFS) for messages produced while
// MQTT is unreachable. Segment files (/q/<n>.log), one record per line:
//   <suffix>\t<qos>\t<retain>\t<payload>\n
// Oldest segment is dropped when the store is full (keeps the newest data).
// Thread safe.
#include <Arduino.h>
#include "core/msgbus.h"

namespace offline {
bool begin();
bool store(const OutMsg* m);
// Peek the oldest record (allocates an OutMsg). Call ack() after it was sent.
OutMsg* peek();
void ack();            // drop the record returned by the last peek()
uint32_t count();      // records waiting
uint32_t bytes();      // bytes used
void clear();
}  // namespace offline

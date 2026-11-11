#pragma once
// ISO 15765-2 (ISO-TP) request/response for OBD-II on CAN (ISO 15765-4).
// 11-bit: functional 0x7DF, physical 0x7E0..7E7, responses 0x7E8..7EF
// 29-bit: functional 0x18DB33F1, physical 0x18DAxxF1, responses 0x18DAF1xx
#include <Arduino.h>
#include "drivers/tcan4550.h"

namespace isotp {
enum Result : int { TIMEOUT = -1, NEGATIVE = -2, TX_FAIL = -3, SEQUENCE = -4 };

typedef void (*FrameHook)(const CanFrame& f);   // receives frames we did not consume
void setHook(FrameHook h);

uint32_t functionalId(bool ext);
uint32_t requestIdFor(bool ext, uint32_t responseId);   // 0x7E8 -> 0x7E0
bool isResponseId(bool ext, uint32_t id);

// Send a request (<= 7 bytes) and return the response length (>0) or a Result.
// rxFilter: only accept this response id (0 = first OBD responder).
int request(Tcan4550& can, bool ext, uint32_t txId, const uint8_t* req, uint8_t reqLen, uint8_t* resp,
            size_t cap, uint32_t rxFilter, uint32_t timeoutMs, uint32_t* fromId = nullptr, uint8_t* nrc = nullptr);

bool sendSingle(Tcan4550& can, bool ext, uint32_t txId, const uint8_t* req, uint8_t reqLen);
}  // namespace isotp

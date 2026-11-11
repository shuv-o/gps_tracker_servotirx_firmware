#include "services/obd/isotp.h"
#include "core/deadline.h"
#include "core/supervisor.h"

namespace isotp {

static FrameHook s_hook = nullptr;
static const uint8_t PAD = 0x00;

void setHook(FrameHook h) { s_hook = h; }

uint32_t functionalId(bool ext) { return ext ? 0x18DB33F1 : 0x7DF; }

uint32_t requestIdFor(bool ext, uint32_t rid) {
  if (!ext) return rid - 8;
  uint8_t ecu = rid & 0xFF;               // 0x18DAF1xx -> 0x18DAxxF1
  return 0x18DA00F1 | ((uint32_t)ecu << 8);
}

bool isResponseId(bool ext, uint32_t id) {
  return ext ? (id & 0xFFFFFF00) == 0x18DAF100 : (id >= 0x7E8 && id <= 0x7EF);
}

static bool tx(Tcan4550& can, bool ext, uint32_t id, const uint8_t* d, uint8_t n) {
  CanFrame f{};
  f.id = id;
  f.ext = ext;
  f.dlc = 8;
  for (uint8_t i = 0; i < 8; i++) f.data[i] = i < n ? d[i] : PAD;
  return can.send(f);
}

bool sendSingle(Tcan4550& can, bool ext, uint32_t txId, const uint8_t* req, uint8_t reqLen) {
  uint8_t b[8];
  b[0] = reqLen & 0x0F;
  memcpy(b + 1, req, reqLen);
  return tx(can, ext, txId, b, reqLen + 1);
}

static bool matches(const uint8_t* req, uint8_t reqLen, const uint8_t* r, size_t n) {
  if (n < 1) return false;
  if (r[0] == 0x7F) return n >= 2 && r[1] == req[0];
  if (r[0] != (uint8_t)(req[0] + 0x40)) return false;
  // modes 01/02/09 echo the PID, 22 echoes two DID bytes
  if ((req[0] == 0x01 || req[0] == 0x09) && reqLen >= 2) return n >= 2 && r[1] == req[1];
  if (req[0] == 0x22 && reqLen >= 3) return n >= 3 && r[1] == req[1] && r[2] == req[2];
  return true;
}

int request(Tcan4550& can, bool ext, uint32_t txId, const uint8_t* req, uint8_t reqLen, uint8_t* resp, size_t cap,
            uint32_t rxFilter, uint32_t timeoutMs, uint32_t* fromId, uint8_t* nrc) {
  sup::feed();
  // flush stale frames
  CanFrame f;
  while (can.receive(f)) if (s_hook) s_hook(f);
  if (!sendSingle(can, ext, txId, req, reqLen)) return TX_FAIL;

  uint32_t deadline = after(timeoutMs);
  uint32_t lockedId = 0;   // ECU we are assembling a multi-frame message from
  size_t total = 0, got = 0;
  uint8_t nextSeq = 1;
  uint8_t pendingExt = 0;

  while (!due(deadline)) {
    if (!can.receive(f)) { vTaskDelay(1); continue; }
    if (f.ext != ext || !isResponseId(ext, f.id) || (rxFilter && f.id != rxFilter) || (lockedId && f.id != lockedId)) {
      if (s_hook) s_hook(f);
      continue;
    }
    uint8_t type = f.data[0] >> 4;
    if (type == 0 && !lockedId) {                       // single frame
      uint8_t n = f.data[0] & 0x0F;
      if (n == 0 || n > 7) continue;
      if (!matches(req, reqLen, f.data + 1, n)) { if (s_hook) s_hook(f); continue; }
      if (f.data[1] == 0x7F) {
        if (f.data[3] == 0x78 && ++pendingExt <= 6) { deadline = after(5000); sup::feed(); continue; }  // response pending
        if (nrc) *nrc = f.data[3];
        if (fromId) *fromId = f.id;
        return NEGATIVE;
      }
      n = min<size_t>(n, cap);
      memcpy(resp, f.data + 1, n);
      if (fromId) *fromId = f.id;
      return n;
    }
    if (type == 1 && !lockedId) {                       // first frame
      total = ((f.data[0] & 0x0F) << 8) | f.data[1];
      if (total < 8 || !matches(req, reqLen, f.data + 2, 6)) { if (s_hook) s_hook(f); continue; }
      lockedId = f.id;
      got = min<size_t>(6, cap);
      memcpy(resp, f.data + 2, got);
      uint8_t fc[3] = {0x30, 0x00, 0x00};               // continue, no block limit, STmin 0
      tx(can, ext, requestIdFor(ext, f.id), fc, 3);
      nextSeq = 1;
      deadline = after(1000);                       // N_Cr
      continue;
    }
    if (type == 2 && lockedId) {                        // consecutive frame
      if ((f.data[0] & 0x0F) != nextSeq) return SEQUENCE;
      nextSeq = (nextSeq + 1) & 0x0F;
      for (int i = 1; i < 8 && got < total; i++)
        if (got < cap) resp[got++] = f.data[i]; else got++;
      if (got >= total) {
        if (fromId) *fromId = lockedId;
        return min(total, cap);
      }
      deadline = after(1000);
      sup::feed();
    }
  }
  return TIMEOUT;
}

}  // namespace isotp

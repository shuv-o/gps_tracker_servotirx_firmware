#pragma once
// TI TCAN4550-Q1 CAN controller (Bosch M_CAN core) over SPI - classic CAN only
// (OBD-II never uses CAN FD). 40 MHz crystal.
//
// MRAM layout (2 KB @ 0x8000):
//   0x000 1 std filter | 0x010 1 ext filter | 0x020 RX FIFO0 32 x 16 B | 0x300 TX FIFO 8 x 16 B
#include <Arduino.h>

struct CanFrame {
  uint32_t id;
  bool     ext;
  uint8_t  dlc;
  uint8_t  data[8];
  uint16_t ts;
};

struct CanStatus {
  uint32_t psr, ecr, ir, devIr;
  uint8_t  tec, rec, lec;
  bool     busOff, errPassive, errWarn;
};

class Tcan4550 {
 public:
  // SPI + reset + ID check. Returns false if the chip does not answer.
  bool begin();
  // (Re)configure M_CAN. acceptAll=false keeps only OBD response IDs
  // (0x7E8-0x7EF, 0x18DAF1xx). listenOnly = bus monitoring (never transmits).
  bool configure(uint16_t kbps, bool listenOnly, bool acceptAll, bool noRetransmit);
  bool send(const CanFrame& f);
  bool receive(CanFrame& f);          // non blocking
  CanStatus status();
  void recoverBusOff();
  bool alive();                       // SPI still talks to the chip
  void hardReset();
  uint16_t bitrate() const { return _kbps; }

  uint32_t rd(uint16_t addr);
  void     wr(uint16_t addr, uint32_t v);

 private:
  void rdN(uint16_t addr, uint32_t* out, uint8_t n);
  void wrN(uint16_t addr, const uint32_t* in, uint8_t n);
  bool setMode(uint32_t modeBits);
  bool enterInit();
  uint16_t _kbps = 0;
};

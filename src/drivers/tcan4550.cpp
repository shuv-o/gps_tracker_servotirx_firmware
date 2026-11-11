#include "drivers/tcan4550.h"
#include <SPI.h>
#include "board_pins.h"
#include "core/counters.h"
#include "core/log.h"

static const char* TAG = "CAN";
static SPIClass s_spi(FSPI);
static const SPISettings SPI_CFG(8000000, MSBFIRST, SPI_MODE0);

// ---- TCAN4550 device registers
enum : uint16_t {
  REG_ID0 = 0x0000, REG_ID1 = 0x0004, REG_REV = 0x0008, REG_SPI_STATUS = 0x000C,
  REG_MODE = 0x0800, REG_DEV_IR = 0x0820, REG_DEV_IE = 0x0830,
};
static constexpr uint16_t MRAM = 0x8000, MRAM_SIZE = 0x0800;
// ---- M_CAN registers (base 0x1000)
enum : uint16_t {
  M_TEST = 0x1010, M_CCCR = 0x1018, M_NBTP = 0x101C, M_TSCC = 0x1020, M_ECR = 0x1040, M_PSR = 0x1044,
  M_IR = 0x1050, M_IE = 0x1054, M_ILE = 0x105C, M_GFC = 0x1080, M_SIDFC = 0x1084, M_XIDFC = 0x1088,
  M_XIDAM = 0x1090, M_RXF0C = 0x10A0, M_RXF0S = 0x10A4, M_RXF0A = 0x10A8, M_RXBC = 0x10AC,
  M_RXF1C = 0x10B0, M_RXESC = 0x10BC, M_TXBC = 0x10C0, M_TXFQS = 0x10C4, M_TXESC = 0x10C8,
  M_TXBAR = 0x10D0, M_TXEFC = 0x10F0,
};
enum : uint32_t { CCCR_INIT = 1u << 0, CCCR_CCE = 1u << 1, CCCR_MON = 1u << 5, CCCR_DAR = 1u << 6 };
enum : uint32_t { MODE_SEL_MASK = 0xC0, MODE_STANDBY = 0x40, MODE_NORMAL = 0x80, MODE_WD_EN = 0x04,
                  MODE_DEV_RESET = 0x02 };
// MRAM layout (offsets inside MRAM)
static constexpr uint16_t SID_OFS = 0x000, XID_OFS = 0x010, RXF0_OFS = 0x020, RXF0_N = 32, TXB_OFS = 0x300, TXB_N = 8;

// ---------------------------------------------------------------- SPI
void Tcan4550::rdN(uint16_t addr, uint32_t* out, uint8_t n) {
  s_spi.beginTransaction(SPI_CFG);
  digitalWrite(PIN_CAN_CS, LOW);
  s_spi.transfer32((0x41u << 24) | ((uint32_t)addr << 8) | n);
  for (uint8_t i = 0; i < n; i++) out[i] = s_spi.transfer32(0);
  digitalWrite(PIN_CAN_CS, HIGH);
  s_spi.endTransaction();
}

void Tcan4550::wrN(uint16_t addr, const uint32_t* in, uint8_t n) {
  s_spi.beginTransaction(SPI_CFG);
  digitalWrite(PIN_CAN_CS, LOW);
  s_spi.transfer32((0x61u << 24) | ((uint32_t)addr << 8) | n);
  for (uint8_t i = 0; i < n; i++) s_spi.transfer32(in[i]);
  digitalWrite(PIN_CAN_CS, HIGH);
  s_spi.endTransaction();
}

uint32_t Tcan4550::rd(uint16_t addr) { uint32_t v; rdN(addr, &v, 1); return v; }
void Tcan4550::wr(uint16_t addr, uint32_t v) { wrN(addr, &v, 1); }

// ---------------------------------------------------------------- setup
void Tcan4550::hardReset() {
  digitalWrite(PIN_CAN_RST, HIGH);
  delayMicroseconds(100);
  digitalWrite(PIN_CAN_RST, LOW);
  delay(10);  // device restarts in standby
}

bool Tcan4550::alive() { return rd(REG_ID0) == 0x4E414354; }  // "TCAN"

bool Tcan4550::begin() {
  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  pinMode(PIN_CAN_RST, OUTPUT);
  digitalWrite(PIN_CAN_RST, LOW);
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  s_spi.begin(PIN_CAN_SCLK, PIN_CAN_MISO, PIN_CAN_MOSI, -1);
  hardReset();
  uint32_t id0 = rd(REG_ID0), id1 = rd(REG_ID1), rev = rd(REG_REV);
  if (id0 != 0x4E414354 || id1 != 0x30353534) {
    LOGE(TAG, "TCAN4550 not found (id 0x%08lX 0x%08lX)", (unsigned long)id0, (unsigned long)id1);
    return false;
  }
  LOGI(TAG, "TCAN4550 rev 0x%08lX", (unsigned long)rev);
  return true;
}

bool Tcan4550::setMode(uint32_t modeBits) {
  uint32_t v = rd(REG_MODE);
  v &= ~(MODE_SEL_MASK | MODE_WD_EN | MODE_DEV_RESET);  // never write the reset bit, watchdog off
  v |= modeBits;
  wr(REG_MODE, v);
  delay(2);
  return (rd(REG_MODE) & MODE_SEL_MASK) == modeBits;
}

bool Tcan4550::enterInit() {
  wr(M_CCCR, CCCR_INIT);
  uint32_t t0 = millis();
  while (!(rd(M_CCCR) & CCCR_INIT)) {
    if (millis() - t0 > 50) return false;
  }
  wr(M_CCCR, CCCR_INIT | CCCR_CCE);
  return true;
}

static uint32_t nbtpFor(uint16_t kbps) {
  // 40 MHz clock, ~80 % sample point
  switch (kbps) {
    case 1000: return (7u << 25) | (0u << 16) | (30u << 8) | 7u;   // 40 tq
    case 500:  return (15u << 25) | (0u << 16) | (62u << 8) | 15u; // 80 tq (bench-verified 0x1E003E0F)
    case 250:  return (15u << 25) | (1u << 16) | (62u << 8) | 15u;
    case 125:  return (15u << 25) | (3u << 16) | (62u << 8) | 15u;
    default:   return 0;
  }
}

bool Tcan4550::configure(uint16_t kbps, bool listenOnly, bool acceptAll, bool noRetransmit) {
  uint32_t nbtp = nbtpFor(kbps);
  if (!nbtp) { LOGE(TAG, "unsupported bitrate %u", kbps); return false; }

  // clear SPI + device interrupt flags (write back what is set = W1C, never 0xFFFFFFFF)
  wr(REG_SPI_STATUS, rd(REG_SPI_STATUS));
  wr(REG_DEV_IR, rd(REG_DEV_IR));

  if (!setMode(MODE_STANDBY)) LOGW(TAG, "standby mode not confirmed");
  if (!enterInit()) { LOGE(TAG, "M_CAN did not enter INIT"); return false; }

  // clear message RAM (avoids ECC errors on uninitialised words)
  static const uint32_t zeros[32] = {0};
  for (uint16_t a = 0; a < MRAM_SIZE; a += sizeof zeros) wrN(MRAM + a, zeros, 32);

  uint32_t cccr = CCCR_INIT | CCCR_CCE;
  if (listenOnly) cccr |= CCCR_MON;
  if (noRetransmit) cccr |= CCCR_DAR;
  wr(M_CCCR, cccr);
  wr(M_NBTP, nbtp);
  wr(M_TSCC, 1);  // timestamp counter on

  // acceptance filters: OBD responses only (unless acceptAll)
  //   std classic: 0x7E8 mask 0x7F8  -> 0x7E8..0x7EF into FIFO0
  //   ext classic: 0x18DAF100 mask 0x1FFFFF00 -> 0x18DAF1xx into FIFO0
  wr(MRAM + SID_OFS, (2u << 30) | (1u << 27) | (0x7E8u << 16) | 0x7F8u);
  uint32_t xf[2] = {(1u << 29) | 0x18DAF100u, (2u << 30) | 0x1FFFFF00u};
  wrN(MRAM + XID_OFS, xf, 2);
  wr(M_SIDFC, (1u << 16) | SID_OFS);
  wr(M_XIDFC, (1u << 16) | XID_OFS);
  wr(M_XIDAM, 0x1FFFFFFF);
  // GFC: non-matching std/ext -> FIFO0 (accept all) or reject; always reject remote frames
  wr(M_GFC, acceptAll ? 0x03u : 0x2Bu);

  wr(M_RXF0C, ((uint32_t)RXF0_N << 16) | RXF0_OFS);  // blocking mode
  wr(M_RXF1C, 0);
  wr(M_RXBC, 0);
  wr(M_RXESC, 0);                                     // 8 byte data fields
  wr(M_TXBC, ((uint32_t)TXB_N << 24) | TXB_OFS);      // TX FIFO, no dedicated buffers
  wr(M_TXESC, 0);
  wr(M_TXEFC, 0);

  wr(M_IE, (1u << 0) | (1u << 25));  // RX FIFO0 new message, bus-off
  wr(M_ILE, 1);
  wr(M_IR, rd(M_IR));

  if (!setMode(MODE_NORMAL)) LOGW(TAG, "normal mode not confirmed (MODE=0x%08lX)", (unsigned long)rd(REG_MODE));
  wr(M_CCCR, cccr & ~(CCCR_INIT | CCCR_CCE));  // start
  delay(2);
  _kbps = kbps;
  LOGI(TAG, "configured %u kbit/s%s%s", kbps, listenOnly ? " listen-only" : "", acceptAll ? " accept-all" : "");
  return !(rd(M_CCCR) & CCCR_INIT);
}

// ---------------------------------------------------------------- data
bool Tcan4550::send(const CanFrame& f) {
  uint32_t fqs = rd(M_TXFQS);
  if (fqs & (1u << 21)) { CNT_INC(can_tx_fail); return false; }  // FIFO full
  uint8_t put = (fqs >> 16) & 0x1F;
  uint32_t el[4];
  el[0] = f.ext ? ((1u << 30) | (f.id & 0x1FFFFFFF)) : ((f.id & 0x7FF) << 18);
  el[1] = (uint32_t)(f.dlc & 0x0F) << 16;
  el[2] = f.data[0] | (f.data[1] << 8) | (f.data[2] << 16) | ((uint32_t)f.data[3] << 24);
  el[3] = f.data[4] | (f.data[5] << 8) | (f.data[6] << 16) | ((uint32_t)f.data[7] << 24);
  wrN(MRAM + TXB_OFS + put * 16, el, 4);
  wr(M_TXBAR, 1u << put);
  CNT_INC(can_tx);
  return true;
}

bool Tcan4550::receive(CanFrame& f) {
  uint32_t s = rd(M_RXF0S);
  if ((s & 0x7F) == 0) return false;
  uint8_t gi = (s >> 8) & 0x3F;
  uint32_t el[4];
  rdN(MRAM + RXF0_OFS + gi * 16, el, 4);
  wr(M_RXF0A, gi);
  f.ext = el[0] & (1u << 30);
  f.id = f.ext ? (el[0] & 0x1FFFFFFF) : ((el[0] >> 18) & 0x7FF);
  f.dlc = (el[1] >> 16) & 0x0F;
  if (f.dlc > 8) f.dlc = 8;
  f.ts = el[1] & 0xFFFF;
  for (int i = 0; i < 4; i++) { f.data[i] = (el[2] >> (8 * i)) & 0xFF; f.data[4 + i] = (el[3] >> (8 * i)) & 0xFF; }
  CNT_INC(can_rx);
  return true;
}

CanStatus Tcan4550::status() {
  CanStatus st{};
  st.psr = rd(M_PSR);
  st.ecr = rd(M_ECR);
  st.ir = rd(M_IR);
  st.devIr = rd(REG_DEV_IR);
  st.tec = st.ecr & 0xFF;
  st.rec = (st.ecr >> 8) & 0x7F;
  st.lec = st.psr & 0x07;
  st.errPassive = st.psr & (1u << 5);
  st.errWarn = st.psr & (1u << 6);
  st.busOff = st.psr & (1u << 7);
  return st;
}

void Tcan4550::recoverBusOff() {
  CNT_INC(can_bus_off);
  LOGW(TAG, "bus-off recovery");
  wr(M_IR, rd(M_IR));
  wr(M_CCCR, rd(M_CCCR) & ~(CCCR_INIT | CCCR_CCE));
}

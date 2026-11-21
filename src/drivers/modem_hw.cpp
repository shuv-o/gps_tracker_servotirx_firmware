#include "drivers/modem_hw.h"
#include "board_pins.h"
#include "core/log.h"
#include "core/supervisor.h"

static uint32_t s_ring = 0;  // written only by the ISR
static void IRAM_ATTR onRing() { __atomic_fetch_add(&s_ring, 1, __ATOMIC_RELAXED); }

static void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { sup::feed(); vTaskDelay(pdMS_TO_TICKS(20)); }
}

namespace modemhw {

void begin() {
  pinMode(PIN_MODEM_PWRKEY, OUTPUT);
  digitalWrite(PIN_MODEM_PWRKEY, LOW);
  pinMode(PIN_MODEM_RESET, OUTPUT);
  digitalWrite(PIN_MODEM_RESET, LOW);
  pinMode(PIN_MODEM_DTR, OUTPUT);
  digitalWrite(PIN_MODEM_DTR, HIGH);  // awake
  pinMode(PIN_MODEM_STATUS, INPUT);
  pinMode(PIN_MODEM_RI, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MODEM_RI), onRing, RISING);
}

bool isOn() { return digitalRead(PIN_MODEM_STATUS) == LOW; }

static bool waitStatus(bool on, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (isOn() == on) return true;
    waitMs(50);
  }
  return isOn() == on;
}

bool powerOn(uint32_t ms) {
  if (isOn()) return true;
  LOGI("MHW", "PWRKEY pulse (power on)");
  digitalWrite(PIN_MODEM_PWRKEY, HIGH);
  waitMs(MODEM_PWRKEY_PULSE_MS);
  digitalWrite(PIN_MODEM_PWRKEY, LOW);
  return waitStatus(true, ms);
}

bool powerOffHard(uint32_t ms) {
  if (!isOn()) return true;
  LOGW("MHW", "PWRKEY long press (power off)");
  digitalWrite(PIN_MODEM_PWRKEY, HIGH);
  waitMs(3000);
  digitalWrite(PIN_MODEM_PWRKEY, LOW);
  return waitStatus(false, ms);
}

void hardReset() {
  LOGW("MHW", "RESET pulse");
  digitalWrite(PIN_MODEM_RESET, HIGH);
  waitMs(MODEM_RESET_PULSE_MS);
  digitalWrite(PIN_MODEM_RESET, LOW);
}

void setAwake(bool awake) { digitalWrite(PIN_MODEM_DTR, awake ? HIGH : LOW); }
uint32_t ringCount() { return __atomic_load_n(&s_ring, __ATOMIC_RELAXED); }

}  // namespace modemhw

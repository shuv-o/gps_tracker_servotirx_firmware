#pragma once
// Fleet Tracker (Nabil V0.3) - ESP32-S3 pin map
// Source: Hardware Verification & Bring-up Status Report, 2026-11-03

// ---------- SIMCom A7672SA-FASE (Stack-4) ----------
#define PIN_MODEM_PWRKEY   7   // OUT -> Q1 -> PWRKEY (pin 1). HIGH pulse = press key
#define PIN_MODEM_RESET   15   // OUT -> Q4 -> RESET  (pin 16). HIGH pulse = reset
#define PIN_MODEM_DTR     16   // OUT -> Q5 -> DTR    (pin 3). HIGH = DTR low = awake
#define PIN_MODEM_STATUS   8   // IN  <- Q3 <- STATUS (pin 66). INVERTED: LOW = modem ON
#define PIN_MODEM_RI       5   // IN  <- Q6 <- RI     (pin 4).  INVERTED: rising edge = ring
#define PIN_MODEM_TX      17   // UART1 TX -> TXS0102 -> SIM_RXD (pin 10)
#define PIN_MODEM_RX      18   // UART1 RX <- TXS0102 <- SIM_TXD (pin 9)
#define MODEM_BAUD    115200

#define MODEM_PWRKEY_PULSE_MS 1000   // verified on bench
#define MODEM_RESET_PULSE_MS  2500   // verified on bench

// ---------- TCAN4550-Q1 CAN FD (Stack-1) - not used by this test ----------
#define PIN_CAN_CS   10
#define PIN_CAN_MOSI 11
#define PIN_CAN_SCLK 12
#define PIN_CAN_MISO 13
#define PIN_CAN_RST  14   // active HIGH
#define PIN_CAN_INT   4   // active LOW

// ---------- Misc ----------
#define PIN_RGB_LED  21   // addressable RGB LED data

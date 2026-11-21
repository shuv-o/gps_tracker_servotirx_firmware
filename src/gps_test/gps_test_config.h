#pragma once
// ================= Active GPS antenna test - user settings =================

// Cellular radio while testing.
//   4 = flight mode (cellular TX/RX off, GNSS keeps working)  <- default
//   0 = minimum functionality
//   1 = full (only if you want GNSS measured with LTE running)
// Cellular off removes LTE TX bursts: no self-jamming and no VBAT ripple
// reaching the antenna LNA through the bias resistor, so you measure the
// antenna alone.
#define CFG_CFUN_MODE          4

// GNSS start at boot: 0 = cold (honest TTFF, clears ephemeris), 1 = warm, 2 = hot
#define CFG_BOOT_START         0

// AT+CGNSSMODE value, 0 = leave the modem's current setting unchanged
#define CFG_GNSS_MODE          0

// Live dashboard / satellite table / automatic diagnosis timing
#define CFG_DASH_EVERY_S       5     // one-line status + per-constellation line
#define CFG_TABLE_EVERY_S      30    // full satellite table (elevation, azimuth, C/N0 bar)
#define CFG_DIAG_AFTER_S       300   // automatic verdict after this many seconds (0 = off)
#define CFG_SAT_STALE_MS       8000  // satellite not reported for this long is dropped

// Bias-tee on this board: 100 ohm from VBAT to the antenna side of the 47 pF cap
#define CFG_BIAS_R_OHMS        100.0f
#define CFG_ANT_MIN_V          2.7f  // most active antennas need >= 2.7 V
#define CFG_ANT_MIN_MA         2.0f  // below this the antenna is probably not connected
#define CFG_ANT_MAX_MA         35.0f // above this: short circuit / damaged antenna

// C/N0 thresholds (dB-Hz) used by the verdict
#define CFG_CN0_GOOD           35    // healthy active antenna outdoors: 38-48
#define CFG_CN0_OK             30    // enough to decode ephemeris and fix
#define CFG_CN0_WEAK           25    // below this: tracking only, no fix

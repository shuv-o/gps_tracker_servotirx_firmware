#pragma once
// ===================== SIMCom device test - user settings =====================

// Carrier APN. Leave "" to let the network assign the default APN.
// Bangladesh examples: Grameenphone "gpinternet", Robi "internet",
// Banglalink "blweb", Teletalk "wap".
#define CFG_APN              ""
#define CFG_SIM_PIN          ""      // only if the SIM is PIN locked

#define CFG_VERBOSE_AT       true    // print every AT command / response (">>" / "<<")
#define CFG_AUTO_RUN         true    // run the full test automatically after boot
#define CFG_AUTO_RUN_DELAY_S 5       // seconds to wait for the serial monitor

// ---- Network ----
#define CFG_SIGNAL_TIMEOUT_S   30
#define CFG_NET_REG_TIMEOUT_S  90
#define CFG_DNS_HOST           "www.google.com"
#define CFG_HTTP_TEST_URL      "http://www.google.com/generate_204"

// ---- Speed test targets (kbit/s) ----
#define CFG_TARGET_4G_KBPS   8000   // 8 Mbit/s on LTE Cat-1 (max 10 Mbit/s)
#define CFG_TARGET_2G_KBPS   1000   // 1 Mbit/s requested. NOTE: EDGE max ~236 kbit/s,
                                    // GPRS ~85 kbit/s -> realistic target is ~100.
#define CFG_SPEED_RUNS_4G    2      // best of N downloads

// ---- GNSS ----
#define CFG_GNSS_MODE          0     // 0 = don't change; else value for AT+CGNSSMODE
#define CFG_GNSS_COLD_TIMEOUT_S 300  // cold start time-to-first-fix limit
#define CFG_GNSS_HOT_TIMEOUT_S   60
#define CFG_GNSS_HOT_TARGET_S    10  // hot start should fix within this
#define CFG_GNSS_SKYVIEW_S        6  // seconds of NMEA captured for the status step
#define CFG_GNSS_GOOD_CN0        30  // dB-Hz considered a healthy signal

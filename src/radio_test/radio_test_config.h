#pragma once
// ================= ESP32-S3 Wi-Fi + Bluetooth LE test - user settings =================

// ---- Wi-Fi access point (the tracker creates its own network) ----
#define CFG_AP_SSID_PREFIX   "FleetTracker-"   // + last 4 hex digits of the MAC
#define CFG_AP_PASSWORD      "tracker123"      // 8..63 chars ("" = open network)
#define CFG_AP_CHANNEL       6
#define CFG_AP_MAX_CLIENTS   4

// ---- Optional: also join an existing Wi-Fi (router / phone hotspot) ----
// Leave CFG_STA_SSID "" to skip. Or type at runtime:  join <ssid> <password>
#define CFG_STA_SSID         ""
#define CFG_STA_PASSWORD     ""
#define CFG_STA_TIMEOUT_S    20
#define CFG_STA_SPEED_URL    "http://speed.cloudflare.com/__down?bytes=2000000"

// ---- Bluetooth LE ----
#define CFG_BLE_NAME_PREFIX  "FleetTracker-"   // + last 4 hex digits of the MAC
#define CFG_BLE_SCAN_S       6

// ---- Antenna verdict thresholds (dBm) ----
// The ESP32-S3-WROOM-1U has NO built-in antenna: it needs an antenna on its own U.FL.
#define CFG_RSSI_GOOD        -65
#define CFG_RSSI_WEAK        -80

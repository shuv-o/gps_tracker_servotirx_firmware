// =============================================================================
//  Fleet Tracker - ESP32-S3 Wi-Fi + Bluetooth LE test firmware  (pio env: radio_test)
//
//  At boot (automatic):
//    1. chip info + MAC addresses
//    2. Wi-Fi scan (networks, RSSI, channel, security) -> antenna verdict
//    3. Wi-Fi ACCESS POINT "FleetTracker-XXXX" + web page at http://192.168.4.1
//       (status, client list with RSSI, /speed throughput test, /json)
//    4. optional: join a router/hotspot (station) + internet speed test
//    5. BLE scan (nearby devices, RSSI) -> antenna verdict
//    6. BLE peripheral "FleetTracker-XXXX": test with the nRF Connect phone app
//       (counter notifications every second, write text -> echoed back)
//    7. PASS/FAIL summary
//  The SIMCom modem is NOT used (PWRKEY/RESET held inactive).
//
//  Serial monitor 115200. Commands: 'h'
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <esp_chip_info.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "board_pins.h"
#include "radio_test_config.h"

// ---------------------------------------------------------------------------
//  Results
// ---------------------------------------------------------------------------
enum Res { R_NOTRUN, R_PASS, R_WARN, R_FAIL };
static const char* RES_TXT[] = {"not run", "PASS", "WARN", "FAIL"};
struct Step { const char* name; Res res; String detail; };
static Step g_steps[] = {
    {"Chip / memory", R_NOTRUN, ""},  {"Wi-Fi scan", R_NOTRUN, ""},
    {"Wi-Fi access point", R_NOTRUN, ""}, {"Wi-Fi station (router)", R_NOTRUN, ""},
    {"BLE scan", R_NOTRUN, ""},       {"BLE peripheral", R_NOTRUN, ""},
};
enum { S_CHIP, S_SCAN, S_AP, S_STA, S_BLESCAN, S_BLEADV, S_N };

static void setRes(int s, Res r, const String& d) {
  g_steps[s].res = r;
  g_steps[s].detail = d;
  Serial.printf("  ==> [%s] %s: %s\n", RES_TXT[r], g_steps[s].name, d.c_str());
}

static String g_suffix;  // last 4 hex of MAC
static WebServer server(80);
static String g_lastScanHtml = "<i>not scanned yet</i>";
static int g_bleSeen = 0, g_bleBest = -127;
static String g_bleAddr;

// BLE
static BLEServer* g_bleSrv = nullptr;
static BLECharacteristic* g_txChar = nullptr;
static volatile bool g_bleConnected = false;
static volatile bool g_bleRestartAdv = false;
static uint32_t g_bleCounter = 0;
static const char* SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";  // Nordic UART-style
static const char* TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";   // notify -> phone
static const char* RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";   // write  <- phone

static String macStr(const uint8_t* m) {
  char b[18];
  snprintf(b, sizeof b, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  return b;
}

static const char* rssiQual(int r) {
  return r >= CFG_RSSI_GOOD ? "good" : r >= CFG_RSSI_WEAK ? "weak" : "very weak";
}

// ---------------------------------------------------------------------------
//  1. Chip
// ---------------------------------------------------------------------------
static void testChip() {
  Serial.println("\n[1] CHIP / MEMORY");
  esp_chip_info_t ci;
  esp_chip_info(&ci);
  Serial.printf("    model %s rev %d, %d cores @ %lu MHz, SDK %s\n", ESP.getChipModel(), ci.revision, ci.cores,
                (unsigned long)ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Serial.printf("    features: %s%s%s\n", (ci.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi-b/g/n " : "",
                (ci.features & CHIP_FEATURE_BLE) ? "BLE " : "", (ci.features & CHIP_FEATURE_BT) ? "BT-Classic" : "(no BT Classic)");
  Serial.printf("    flash %lu MB, PSRAM %lu KB (free %lu KB), heap free %lu KB\n",
                (unsigned long)(ESP.getFlashChipSize() >> 20), (unsigned long)(ESP.getPsramSize() >> 10),
                (unsigned long)(ESP.getFreePsram() >> 10), (unsigned long)(ESP.getFreeHeap() >> 10));
  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_WIFI_STA);  Serial.printf("    MAC Wi-Fi station  %s\n", macStr(m).c_str());
  esp_read_mac(m, ESP_MAC_WIFI_SOFTAP); Serial.printf("    MAC Wi-Fi AP       %s\n", macStr(m).c_str());
  esp_read_mac(m, ESP_MAC_BT);        Serial.printf("    MAC Bluetooth      %s\n", macStr(m).c_str());
  char sfx[5];
  snprintf(sfx, sizeof sfx, "%02X%02X", m[4], m[5]);
  g_suffix = sfx;
  Serial.println("    NOTE: ESP32-S3-WROOM-1U has no PCB antenna - an antenna must be on the module's U.FL.");
  bool ok = (ci.features & CHIP_FEATURE_WIFI_BGN) && (ci.features & CHIP_FEATURE_BLE);
  setRes(S_CHIP, ok ? R_PASS : R_FAIL,
         String(ESP.getChipModel()) + ", PSRAM " + String(ESP.getPsramSize() >> 20) + " MB");
}

// ---------------------------------------------------------------------------
//  2. Wi-Fi scan
// ---------------------------------------------------------------------------
static const char* authStr(wifi_auth_mode_t a) {
  switch (a) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/3";
    default: return "?";
  }
}

static void testWifiScan() {
  Serial.println("\n[2] WI-FI SCAN (2.4 GHz)");
  uint32_t t0 = millis();
  int n = WiFi.scanNetworks(false, true);
  uint32_t dt = millis() - t0;
  if (n < 0) {
    setRes(S_SCAN, R_FAIL, "scan error " + String(n) + " (radio not working)");
    return;
  }
  Serial.printf("    %d networks in %lu ms\n", n, (unsigned long)dt);
  Serial.println("    #   RSSI  ch  security  BSSID              SSID");
  int best = -127;
  int perCh[15] = {0};
  String html = "<table><tr><th>RSSI</th><th>ch</th><th>sec</th><th>SSID</th></tr>";
  for (int i = 0; i < n && i < 40; i++) {
    int r = WiFi.RSSI(i), ch = WiFi.channel(i);
    if (r > best) best = r;
    if (ch > 0 && ch < 15) perCh[ch]++;
    String ssid = WiFi.SSID(i).length() ? WiFi.SSID(i) : String("(hidden)");
    Serial.printf("    %-3d %4d  %2d  %-8s  %s  %s\n", i + 1, r, ch, authStr(WiFi.encryptionType(i)),
                  WiFi.BSSIDstr(i).c_str(), ssid.c_str());
    html += "<tr><td>" + String(r) + "</td><td>" + ch + "</td><td>" + authStr(WiFi.encryptionType(i)) +
            "</td><td>" + ssid + "</td></tr>";
  }
  html += "</table>";
  g_lastScanHtml = html;
  Serial.print("    networks per channel: ");
  for (int c = 1; c <= 13; c++) if (perCh[c]) Serial.printf("ch%d=%d ", c, perCh[c]);
  Serial.println();
  WiFi.scanDelete();
  if (n == 0)
    setRes(S_SCAN, R_FAIL, "0 networks - antenna missing on the ESP32 U.FL, or no Wi-Fi nearby");
  else if (best < CFG_RSSI_WEAK)
    setRes(S_SCAN, R_WARN, String(n) + " networks but strongest only " + best +
                               " dBm -> check ESP32 antenna / U.FL connector");
  else
    setRes(S_SCAN, R_PASS, String(n) + " networks, strongest " + best + " dBm (" + rssiQual(best) + ")");
}

// ---------------------------------------------------------------------------
//  3. Access point + web server
// ---------------------------------------------------------------------------
static void onWifiEvent(WiFiEvent_t e, WiFiEventInfo_t info) {
  switch (e) {
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.printf("\n  [AP] client CONNECTED    %s (aid %d)\n", macStr(info.wifi_ap_staconnected.mac).c_str(),
                    info.wifi_ap_staconnected.aid);
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.printf("\n  [AP] client DISCONNECTED %s\n", macStr(info.wifi_ap_stadisconnected.mac).c_str());
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.printf("  [AP] client got IP %s -> open http://%s\n",
                    IPAddress(info.wifi_ap_staipassigned.ip.addr).toString().c_str(),
                    WiFi.softAPIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("  [STA] disconnected, reason %d\n", info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("  [STA] got IP %s\n", WiFi.localIP().toString().c_str());
      break;
    default: break;
  }
}

static String clientsText(bool html) {
  wifi_sta_list_t list;
  String s;
  if (esp_wifi_ap_get_sta_list(&list) != ESP_OK || list.num == 0) return html ? "<i>none</i>" : "none";
  for (int i = 0; i < list.num; i++) {
    int r = list.sta[i].rssi;
    s += macStr(list.sta[i].mac) + "  RSSI " + String(r) + " dBm (" + rssiQual(r) + ")";
    s += html ? "<br>" : "\n    ";
  }
  return s;
}

static String jsonStatus() {
  String j = "{";
  j += "\"uptime_s\":" + String(millis() / 1000);
  j += ",\"heap\":" + String(ESP.getFreeHeap());
  j += ",\"ap_ssid\":\"" + WiFi.softAPSSID() + "\"";
  j += ",\"ap_clients\":" + String(WiFi.softAPgetStationNum());
  j += ",\"sta_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  j += ",\"sta_rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  j += ",\"ble_connected\":" + String(g_bleConnected ? "true" : "false");
  j += ",\"ble_seen\":" + String(g_bleSeen);
  j += ",\"ble_counter\":" + String(g_bleCounter);
  j += "}";
  return j;
}

static void handleRoot() {
  String h = F("<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width'>"
               "<meta http-equiv=refresh content=5><title>Fleet Tracker radio test</title>"
               "<style>body{font-family:sans-serif;margin:16px}td,th{padding:2px 8px;text-align:left}"
               "h2{margin-top:20px}</style></head><body><h1>Fleet Tracker radio test</h1>");
  h += "<p>Uptime " + String(millis() / 1000) + " s, free heap " + String(ESP.getFreeHeap() / 1024) + " KB</p>";
  h += "<h2>Test results</h2><table>";
  for (int i = 0; i < S_N; i++)
    h += "<tr><td><b>" + String(RES_TXT[g_steps[i].res]) + "</b></td><td>" + g_steps[i].name + "</td><td>" +
         g_steps[i].detail + "</td></tr>";
  h += "</table><h2>Access point</h2><p>SSID <b>" + WiFi.softAPSSID() + "</b>, channel " + String(CFG_AP_CHANNEL) +
       ", clients " + String(WiFi.softAPgetStationNum()) + "</p><p>" + clientsText(true) + "</p>";
  h += "<h2>Speed test</h2><p><a href='/speed?kb=1024'>download 1 MB</a> &middot; "
       "<a href='/speed?kb=4096'>4 MB</a> (result also printed on the serial monitor)</p>";
  h += "<h2>Bluetooth LE</h2><p>Advertising as <b>" + String(CFG_BLE_NAME_PREFIX) + g_suffix + "</b>, " +
       (g_bleConnected ? "phone CONNECTED" : "not connected") + ", last scan saw " + g_bleSeen + " devices</p>";
  h += "<h2>Wi-Fi networks seen</h2>" + g_lastScanHtml + "<p><a href='/json'>JSON</a></p></body></html>";
  server.send(200, "text/html", h);
}

static void handleSpeed() {
  uint32_t kb = server.hasArg("kb") ? server.arg("kb").toInt() : 1024;
  if (kb == 0 || kb > 16384) kb = 1024;
  static uint8_t buf[1460];
  memset(buf, 'x', sizeof buf);
  uint32_t total = kb * 1024, sent = 0, t0 = millis();
  server.setContentLength(total);
  server.send(200, "application/octet-stream", "");
  WiFiClient c = server.client();
  while (sent < total && c.connected()) {
    size_t n = total - sent < sizeof buf ? total - sent : sizeof buf;
    size_t w = c.write(buf, n);
    if (w == 0) { delay(1); if (millis() - t0 > 60000) break; continue; }
    sent += w;
  }
  uint32_t dt = millis() - t0;
  float mbps = dt ? sent * 8.0f / dt / 1000.0f : 0;
  Serial.printf("\n  [AP] speed test: sent %lu KB in %lu ms = %.2f Mbit/s %s\n", (unsigned long)(sent / 1024),
                (unsigned long)dt, mbps, sent < total ? "(client stopped early)" : "");
}

static void testAccessPoint() {
  Serial.println("\n[3] WI-FI ACCESS POINT");
  String ssid = String(CFG_AP_SSID_PREFIX) + g_suffix;
  bool ok = WiFi.softAP(ssid.c_str(), strlen(CFG_AP_PASSWORD) ? CFG_AP_PASSWORD : nullptr, CFG_AP_CHANNEL, 0,
                        CFG_AP_MAX_CLIENTS);
  delay(300);
  if (!ok) { setRes(S_AP, R_FAIL, "WiFi.softAP() failed"); return; }
  server.on("/", handleRoot);
  server.on("/json", [] { server.send(200, "application/json", jsonStatus()); });
  server.on("/speed", handleSpeed);
  server.onNotFound([] { server.send(404, "text/plain", "not found"); });
  server.begin();
  int8_t pwr = 0;
  esp_wifi_get_max_tx_power(&pwr);
  Serial.printf("    SSID      %s\n    password  %s\n    channel   %d   TX power %.1f dBm\n", ssid.c_str(),
                strlen(CFG_AP_PASSWORD) ? CFG_AP_PASSWORD : "(open)", CFG_AP_CHANNEL, pwr / 4.0f);
  Serial.printf("    web page  http://%s   (status, client RSSI, /speed test)\n",
                WiFi.softAPIP().toString().c_str());
  Serial.println("    -> connect a phone/laptop to this network to complete the AP test");
  setRes(S_AP, R_PASS, ssid + " up at " + WiFi.softAPIP().toString() + " (connect a phone to verify)");
}

// ---------------------------------------------------------------------------
//  4. Station (optional)
// ---------------------------------------------------------------------------
static void testStation(const String& ssid, const String& pass) {
  Serial.println("\n[4] WI-FI STATION (join router / hotspot)");
  if (!ssid.length()) {
    setRes(S_STA, R_NOTRUN, "skipped (no SSID set) - type: join <ssid> <password>");
    return;
  }
  Serial.printf("    joining '%s' ...\n", ssid.c_str());
  WiFi.disconnect(false);
  delay(100);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < CFG_STA_TIMEOUT_S * 1000UL) delay(200);
  if (WiFi.status() != WL_CONNECTED) {
    setRes(S_STA, R_FAIL, "could not join '" + ssid + "' in " + String(CFG_STA_TIMEOUT_S) +
                              " s (wrong password, out of range, or 5 GHz-only network)");
    return;
  }
  int r = WiFi.RSSI();
  Serial.printf("    connected in %lu ms: IP %s  GW %s  DNS %s  RSSI %d dBm  ch %d\n",
                (unsigned long)(millis() - t0), WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str(), r, WiFi.channel());
  if (WiFi.channel() != CFG_AP_CHANNEL)
    Serial.println("    note: the AP moved to the router's channel (one radio, one channel)");

  // Internet download test
  HTTPClient http;
  http.setTimeout(15000);
  float mbps = 0;
  uint32_t got = 0;
  if (http.begin(CFG_STA_SPEED_URL)) {
    uint32_t t1 = millis();
    int code = http.GET();
    if (code == 200) {
      WiFiClient* s = http.getStreamPtr();
      static uint8_t buf[2048];
      int len = http.getSize();
      while (http.connected() && (len < 0 || got < (uint32_t)len) && millis() - t1 < 30000) {
        size_t a = s->available();
        if (a) got += s->readBytes(buf, a > sizeof buf ? sizeof buf : a);
        else delay(1);
      }
      uint32_t dt = millis() - t1;
      mbps = dt ? got * 8.0f / dt / 1000.0f : 0;
      Serial.printf("    internet download %lu KB in %lu ms = %.2f Mbit/s\n", (unsigned long)(got / 1024),
                    (unsigned long)dt, mbps);
    } else {
      Serial.printf("    HTTP test failed: %d %s\n", code, http.errorToString(code).c_str());
    }
    http.end();
  }
  String d = "'" + ssid + "' RSSI " + r + " dBm (" + rssiQual(r) + ")";
  if (got) d += ", internet " + String(mbps, 1) + " Mbit/s";
  else d += ", no internet";
  setRes(S_STA, (r < CFG_RSSI_WEAK || !got) ? R_WARN : R_PASS, d);
}

// ---------------------------------------------------------------------------
//  5/6. Bluetooth LE
// ---------------------------------------------------------------------------
class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    g_bleConnected = true;
    Serial.println("\n  [BLE] phone CONNECTED");
  }
  void onDisconnect(BLEServer*) override {
    g_bleConnected = false;
    g_bleRestartAdv = true;  // restart advertising from loop()
    Serial.println("\n  [BLE] phone DISCONNECTED");
  }
};

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();  // std::string (core 2.x) or String (core 3.x)
    String s = String(v.c_str());
    Serial.printf("\n  [BLE] phone wrote %u bytes: \"%s\" -> echoed back\n", (unsigned)s.length(), s.c_str());
    if (g_txChar && g_bleConnected) {
      String e = "echo: " + s;
      g_txChar->setValue((uint8_t*)e.c_str(), e.length());
      g_txChar->notify();
    }
  }
};

static void bleInit() {
  static bool done = false;
  if (done) return;
  BLEDevice::init((String(CFG_BLE_NAME_PREFIX) + g_suffix).c_str());
  BLEDevice::setMTU(185);
  done = true;
}

static void testBleScan() {
  Serial.printf("\n[5] BLE SCAN (%d s, active)\n", CFG_BLE_SCAN_S);
  bleInit();
  BLEScan* sc = BLEDevice::getScan();
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(99);
  BLEScanResults res = sc->start(CFG_BLE_SCAN_S, false);
  int n = res.getCount();
  g_bleSeen = n;
  g_bleBest = -127;
  Serial.println("    #   RSSI  address            name");
  for (int i = 0; i < n; i++) {
    BLEAdvertisedDevice d = res.getDevice(i);
    int r = d.getRSSI();
    if (r > g_bleBest) { g_bleBest = r; g_bleAddr = d.getAddress().toString().c_str(); }
    if (i < 30)
      Serial.printf("    %-3d %4d  %s  %s\n", i + 1, r, d.getAddress().toString().c_str(),
                    d.haveName() ? d.getName().c_str() : "");
  }
  sc->clearResults();
  if (n == 0)
    setRes(S_BLESCAN, R_WARN, "0 devices - no BLE devices nearby (turn on phone Bluetooth) or antenna problem");
  else if (g_bleBest < CFG_RSSI_WEAK)
    setRes(S_BLESCAN, R_WARN, String(n) + " devices, strongest only " + g_bleBest + " dBm -> check antenna");
  else
    setRes(S_BLESCAN, R_PASS, String(n) + " devices, strongest " + g_bleBest + " dBm (" + rssiQual(g_bleBest) + ")");
}

static void testBlePeripheral() {
  Serial.println("\n[6] BLE PERIPHERAL (GATT server)");
  bleInit();
  g_bleSrv = BLEDevice::createServer();
  g_bleSrv->setCallbacks(new SrvCb());
  BLEService* svc = g_bleSrv->createService(SVC_UUID);
  g_txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_txChar->addDescriptor(new BLE2902());
  g_txChar->setValue("hello from Fleet Tracker");
  BLECharacteristic* rx =
      svc->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("    advertising as  %s%s\n", CFG_BLE_NAME_PREFIX, g_suffix.c_str());
  Serial.printf("    BT address      %s\n", BLEDevice::getAddress().toString().c_str());
  Serial.println("    test with the 'nRF Connect' app: SCAN -> CONNECT -> Nordic UART service");
  Serial.println("      TX (6e400003) notify = counter every second | RX (6e400002) write text = echo");
  setRes(S_BLEADV, R_PASS, "advertising as " + String(CFG_BLE_NAME_PREFIX) + g_suffix + " (connect a phone to verify)");
}

// ---------------------------------------------------------------------------
//  Summary / commands
// ---------------------------------------------------------------------------
static void summary() {
  Serial.println("\n======================= RADIO TEST SUMMARY =======================");
  for (int i = 0; i < S_N; i++)
    Serial.printf("  [%-7s] %-24s %s\n", RES_TXT[g_steps[i].res], g_steps[i].name, g_steps[i].detail.c_str());
  Serial.printf("  AP clients: %d -> %s\n", WiFi.softAPgetStationNum(), clientsText(false).c_str());
  Serial.printf("  BLE phone connected: %s | heap free %lu KB (min %lu KB)\n", g_bleConnected ? "YES" : "no",
                (unsigned long)(ESP.getFreeHeap() / 1024), (unsigned long)(ESP.getMinFreeHeap() / 1024));
  Serial.println("==================================================================");
}

static void help() {
  Serial.println("\n================ RADIO TEST - commands ================");
  Serial.println("  s              summary            w   Wi-Fi scan again");
  Serial.println("  b              BLE scan again     c   AP clients + RSSI");
  Serial.println("  join SSID PASS join a router/hotspot + internet speed test");
  Serial.println("  leave          disconnect from the router (AP stays up)");
  Serial.println("  a              run all tests again                h  help");
  Serial.println("=======================================================");
}

static void runAll() {
  testWifiScan();
  testStation(CFG_STA_SSID, CFG_STA_PASSWORD);
  testBleScan();
  summary();
}

static void command(String in) {
  in.trim();
  if (!in.length()) return;
  if (in == "h") help();
  else if (in == "s") summary();
  else if (in == "w") testWifiScan();
  else if (in == "b") testBleScan();
  else if (in == "c") Serial.printf("AP clients: %d\n    %s\n", WiFi.softAPgetStationNum(), clientsText(false).c_str());
  else if (in == "a") runAll();
  else if (in == "leave") { WiFi.disconnect(false); Serial.println("left the router network"); }
  else if (in.startsWith("join ")) {
    String rest = in.substring(5);
    rest.trim();
    int sp = rest.lastIndexOf(' ');
    String ssid = sp > 0 ? rest.substring(0, sp) : rest;
    String pass = sp > 0 ? rest.substring(sp + 1) : "";
    testStation(ssid, pass);
  } else Serial.println("unknown command - 'h' for help");
}

// ---------------------------------------------------------------------------
void setup() {
  // keep the modem untouched: PWRKEY / RESET drivers inactive
  pinMode(PIN_MODEM_PWRKEY, OUTPUT); digitalWrite(PIN_MODEM_PWRKEY, LOW);
  pinMode(PIN_MODEM_RESET, OUTPUT);  digitalWrite(PIN_MODEM_RESET, LOW);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 8000) delay(10);
  delay(500);
  Serial.println("\n####################################################################");
  Serial.println("#  Fleet Tracker - ESP32-S3 Wi-Fi + Bluetooth LE test               #");
  Serial.println("####################################################################");

  testChip();
  WiFi.onEvent(onWifiEvent);
  WiFi.mode(WIFI_AP_STA);
  // Wi-Fi modem sleep MUST stay enabled when Bluetooth runs too, otherwise the
  // Wi-Fi/BT coexistence code aborts in esp_bt_controller_enable().
  WiFi.setSleep(true);
  testWifiScan();
  testAccessPoint();
  testStation(CFG_STA_SSID, CFG_STA_PASSWORD);
  testBleScan();
  testBlePeripheral();
  summary();
  help();
}

void loop() {
  static String in;
  static uint32_t lastNotify = 0, lastClients = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') { command(in); in = ""; }
    else in += ch;
  }
  server.handleClient();

  uint32_t now = millis();
  if (g_bleRestartAdv) {
    g_bleRestartAdv = false;
    delay(200);
    BLEDevice::startAdvertising();
    Serial.println("  [BLE] advertising again");
  }
  if (g_bleConnected && g_txChar && now - lastNotify >= 1000) {
    lastNotify = now;
    g_bleCounter++;
    String v = "count " + String(g_bleCounter) + " up " + String(now / 1000) + "s";
    g_txChar->setValue((uint8_t*)v.c_str(), v.length());
    g_txChar->notify();
  }
  if (now - lastClients >= 30000) {  // periodic client RSSI = antenna check from the device side
    lastClients = now;
    if (WiFi.softAPgetStationNum())
      Serial.printf("  [AP] %d client(s): %s\n", WiFi.softAPgetStationNum(), clientsText(false).c_str());
  }
  delay(2);
}

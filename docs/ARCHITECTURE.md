# Fleet Tracker firmware – architecture

## Source tree
```
include/
  board_pins.h          all GPIO assignments (one place)
  fw_version.h          bump FW_VERSION on every release
src/
  app/main.cpp          start-up order, task creation
  core/                 no hardware here
    config.*            runtime settings table (NVS) – console + MQTT editable
    log.*               logger + RAM ring buffer ("logs" command)
    counters.h          telemetry counters (X-macro, auto-serialised)
    clock.*             UTC clock, GNSS > network priority
    msgbus.*            outgoing message queue (any task → cellular task)
    supervisor.*        watchdogs, safe mode, crash info, OTA confirm/rollback
  drivers/              hardware only, no policy
    at_channel.*        SIMCom AT parser
    modem_hw.*          PWRKEY / RESET / DTR / STATUS / RI
    tcan4550.*          CAN controller (SPI, M_CAN)
    status_led.*        RGB LED patterns
  services/             behaviour
    cellular.*          network state machine, recovery, 4G→2G failover, delivery
    mqtt_client.*       A7672 native MQTT
    gnss.*              location + fallback chain
    ota.*               firmware download + install
    offline_store.*     flash queue while offline
    reporter.*          what to send and when (+ JSON builders)
    commands.*          console + remote command table
    obd/                isotp, pid_table, obd (protocol detect, polling, DTC, trips)
  test_fw/              separate bench test build (env:modem_test)
docs/                   this file, MQTT_PROTOCOL.md
```

## Tasks
| task | core | prio | owns | heartbeat |
|---|---|---|---|---|
| cell | 0 | 5 | modem UART (exclusively): network, MQTT, GNSS, OTA, delivery, remote commands | 90 s |
| obd | 1 | 6 | TCAN4550 SPI | 60 s |
| app | 1 | 3 | reporting policy, LED | 30 s |
| console | 0 | 2 | USB serial | – |
| loop | 1 | 1 | supervisor tick | – |

Rules that keep it stable:
- **Only the cell task touches the modem.** Other tasks use `cell::exec(job)`; it runs inline when already on the cell task.
- Shared state lives in its owning module behind a mutex; others call `snapshot()` / `toJson()`.
- No AT or SPI I/O while holding a state mutex.
- Long waits call `sup::feed()` (the AT driver does this automatically).

## Failover chains
| area | chain |
|---|---|
| Network | L1 PDP re-attach → L2 radio off/on → L3 `AT+CRESET` → L4 RESET pin → L5 power cycle with 2…30 min backoff. Resets to L0 after 5 min stable. |
| Radio | LTE data failing → force GSM (2G) → retry LTE every `lte_retry_s`; kept if MQTT reconnects within 4 min |
| MQTT | backoff 5 s → 5 min with jitter → fallback broker after 3 failures → network escalation only if DNS also fails (a broker outage never resets the modem) |
| Data | RAM queue → flash queue (2.5 MB, oldest dropped first) → replay on reconnect |
| Location | GNSS → AGNSS assist → cell-tower LBS → last known (flash) + cell IDs in every message |
| Time | GNSS → network NITZ/CCLK → uptime (`up`) |
| CAN/OBD | passive listen → probe 500/250 k × 11/29-bit → bus-off recovery → re-detect → chip re-init |
| Firmware | task heartbeat → reboot; 30 s HW watchdog → panic + core dump; crash loop → safe mode (auto-exits after 1 h stable; brownouts from engine cranking do not count); bad OTA → bootloader rollback |

## Adding things
- **Setting**: field in `Config` (config.h) + one line in `ITEMS` (config.cpp). Instantly editable via `config set`.
- **OBD PID**: one line in `pid_table.cpp`.
- **Counter**: one `X(name)` in `counters.h`; appears in `health.cnt`.
- **Command**: handler + one line in `COMMANDS` (commands.cpp); works on console and MQTT.
- **Message type**: builder in `reporter.cpp`, call `bus::publish("suffix", json)`.
- **Manufacturer data** (door, fuel on some makes, …): no firmware change –
  `config set obd_custom 7E0:221234:30` sends UDS 22 1234 to ECU 0x7E0 every 30 s; the raw
  response appears in `obd.custom` for decoding on the server.

## Vehicle compatibility
OBD-II over CAN (ISO 15765-4) only, which covers every car sold in the US since 2008 and
in the EU from about 2008–2010 (Toyota, Honda, BMW, Mercedes, Nissan, Hyundai, VW, Ford, …),
11- and 29-bit, 250/500 kbit/s. Older K-line (ISO 9141 / KWP2000) and J1850 cars need
extra transceiver hardware this board does not have.

Standard OBD-II gives powertrain/emissions data (speed, RPM, temperatures, fuel level on
most cars, faults, VIN, odometer on newer cars). Doors, locks, tyre pressure, exact fuel
level on some makes are **manufacturer-specific** – use `obd_custom` requests per model.

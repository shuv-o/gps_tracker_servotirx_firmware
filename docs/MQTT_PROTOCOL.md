# Fleet Tracker – MQTT protocol

Transport: the A7672SA's **built-in MQTT client** (`AT+CMQTT*`). It works the same on
LTE and on 2G (GPRS/EDGE). All payloads are compact JSON (UTF-8).

Topic layout: `<topic_root>/<device_id>/<suffix>`

- `topic_root` – config `topic_root` (default `fleet`)
- `device_id` – modem IMEI (e.g. `860710056784732`)

Subscribe on the server to `fleet/+/#`.

## Common fields

| field | meaning |
|---|---|
| `ts` | UTC epoch seconds (from GNSS, else network time; `0` = clock unknown yet) |
| `up` | device uptime in seconds (always valid; use with `ts=0` messages) |

Messages produced while offline are stored in flash and **replayed oldest-first**
after reconnect, so `ts` can be in the past. Order by `ts`, not by arrival time.

## Device → server

| suffix | QoS | retained | when |
|---|---|---|---|
| `status` | 1 | yes | `{"online":true,...}` on connect; broker publishes LWT `{"online":false}` if the device drops |
| `info` | 1 | yes | on every connect: firmware, IMEI/ICCID, VIN, OBD protocol, supported PIDs |
| `loc` | 1 | no | every `rpt_move_s` moving / `rpt_park_s` parked, + every `rpt_dist_m` metres, + heading change `rpt_head_deg` |
| `obd` | 1 | no | every `rpt_obd_s` while ignition is on |
| `dtc` | 1 | no | whenever the fault code list changes |
| `health` | 1 | no | every `rpt_health_s` – device telemetry |
| `event` | 1 | no | state changes (see below) |
| `cmd/resp` | 1 | no | reply to a command |

### `loc`
```json
{"ts":1762339200,"up":5120,"src":"gnss","lat":23.810331,"lon":90.412521,"alt":12,
 "fix_ts":1762339199,"age":1,"spd":42.5,"crs":184,"hdop":0.9,"sat":11,"acc":5,
 "obd_spd":43,"mv":true,"ign":true,"odo":1234.56,
 "cell":{"mcc":"470","mnc":"02","tac":"0xC1EE","cid":33440526,"rat":"LTE","csq":31,"rsrp":-79}}
```
`src` = location source, best first:

| src | meaning |
|---|---|
| `gnss` | live satellite fix |
| `lbs` | modem cell-tower location (if the modem firmware supports `AT+CLBS`) |
| `last` | last known position (stale – check `fix_ts` / `age`) |
| `none` | nothing yet – use `cell` with a cell-ID database (OpenCellID, Google, Unwired Labs) |

`cell` is always included so the server can geolocate by tower when GNSS is unavailable.

### `obd`
```json
{"ts":...,"ign":true,"eng":true,"vin":"JTDBR32E720123456","mil":false,"dtc_n":0,
 "v":{"rpm":"812.00","speed":"0.00","coolant":"88.00","load":"21.18","fuel_level":"63.14",
      "ecu_volt":"14.12","maf":"3.21","throttle":"14.90","fuel_lph_est":"1.06"},
 "trip":{"km":"12.40","max_kmh":78,"idle_s":95,"fuel_l":"1.10","dur_s":1260},
 "custom":{"7E0:221234":"621234015A"}}
```
`v` contains only PIDs the vehicle supports and that were read recently. Keys and
units: `src/services/obd/pid_table.cpp` (rpm, speed km/h, coolant °C, load %, maf g/s,
fuel_level %, ecu_volt V, odometer km, fuel_rate L/h, oil_temp °C, ...).

### `dtc`
```json
{"ts":...,"mil":true,"stored":["P0301","P0420"],"pending":["P0171"],"permanent":[]}
```

### `health` (device telemetry)
```json
{"ts":...,"fw":"1.0.0","boot":17,"rst":"power_on","safe":false,"heap":201344,"heap_min":187220,
 "psram":8100000,"mcu_temp":"41.3","clk":"gnss","q_flash":0,"q_ram":0,"log_e":0,"log_w":3,
 "net":{"ph":"online","rat":"LTE","csq":31,"rsrp":-79,"rsrq":-11,"op":"Robi","band":"EUTRAN-BAND3",
        "fb2g":false,"rl":0,"vbat":"4.21","mtemp":31},
 "mqtt":{"conn":true,"broker":0,"up_s":3600,"fails":0},
 "gnss":{"on":true,"src":"gnss","sat":11,"hdop":"0.9","nofix_s":0,"ttff_s":34,"agnss":false,"lbs":true},
 "obd":{"st":"running","kbps":500,"detect":1,"busoff":0,"tec":0,"rec":0},
 "cnt":{"mqtt_connects":2,"mqtt_pub_ok":812,"rat_fallbacks":1,"gnss_fixes":3}}
```
`cnt` only lists non-zero counters (all names in `src/core/counters.h`).
`net.rl` = current recovery level (0 healthy … 5 power cycling), `fb2g` = forced 2G fallback.

### `event`
`{"ts":...,"up":...,"ev":"<name>","d":{...}}`

| ev | d |
|---|---|
| `boot` | fw, rst (reset reason), boot count, safe, ota_pending, crash (core-dump summary), stall |
| `ignition_on` / `ignition_off` | proto, ecus, vin |
| `trip_end` | start, dur_s, km, max_kmh, idle_s, fuel_l, harsh_accel, harsh_brake, over_rev |
| `harsh_accel` / `harsh_brake` | kmh_s, kmh |
| `moving` / `stopped` | – |
| `gnss_fix` / `gnss_lost` | src |
| `net_online` | rat, op, csq, ip, fb2g |
| `net_lost`, `net_recovery` | level, why |
| `rat_fallback` / `rat_restore` | LTE→GSM failover and back |
| `mqtt_lost` | – |
| `ota` | state start/progress/installed/failed, pct, err |
| `ota_confirmed` | new firmware passed its health check (rollback cancelled) |
| `dtc_clear`, `reboot` | – |

## Server → device: commands

Publish to `<root>/<id>/cmd` (QoS 1), either plain text or JSON:
```
status
{"id":"42","cmd":"config set rpt_move_s 5"}
```
Reply on `<root>/<id>/cmd/resp`:
```json
{"id":"42","cmd":"config set rpt_move_s 5","ok":true,"ts":...,"out":"rpt_move_s saved ..."}
```
The command set is the same as the USB console – send `help`. The most useful ones:

| command | |
|---|---|
| `status` | everything at a glance |
| `config list` / `config set <key> <value>` | change any setting remotely (saved in flash) |
| `report loc\|obd\|dtc\|health\|info` | send a message now |
| `logs 4000` | last 4 KB of the device log |
| `log 4` | raise log level (5 = full AT trace) |
| `at AT+CPSI?` | raw modem command |
| `rat auto\|lte\|gsm` | network mode |
| `modem recover 1..5` | force a recovery step |
| `gps cold\|hot\|agnss` | GNSS control |
| `dtc read` / `dtc clear confirm` | fault codes |
| `can sniff on` / `can rescan` | CAN bus debugging |
| `ota <url> <md5> [force] [version=x.y.z]` | firmware update |
| `safemode on\|off`, `reboot` | |

## OTA

1. Build: `pio run -e fleet_tracker` → `.pio/build/fleet_tracker/firmware.bin`
2. Host it on HTTP(S) **with Range support** (nginx, S3, most CDNs).
3. `md5 -q firmware.bin` (macOS) / `md5sum firmware.bin`
4. Send: `ota https://example.com/fw/1.0.1.bin <md5> version=1.0.1`
5. Watch `event` for `ota` progress → device reboots → `ota_confirmed` once it reconnects to MQTT.
   If the new image can't reach MQTT within `ota_confirm_s` (or crashes), the bootloader
   **rolls back automatically** to the previous firmware.

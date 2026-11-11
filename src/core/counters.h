#pragma once
// Device-wide event counters, reported in the "health" telemetry message.
// Add a counter = add one X(name) line. It is automatically serialised.
#include <Arduino.h>

#define DEVICE_COUNTERS(X) \
  X(modem_power_cycles) X(modem_hard_resets) X(modem_soft_resets) X(radio_cycles) \
  X(pdp_reattach) X(reg_failures) X(rat_fallbacks) X(rat_restores) \
  X(at_timeouts) X(at_errors) \
  X(mqtt_connects) X(mqtt_connect_fail) X(mqtt_conn_lost) X(mqtt_pub_ok) X(mqtt_pub_fail) \
  X(mqtt_rx) X(broker_failovers) \
  X(q_stored) X(q_replayed) X(q_dropped) X(msg_dropped) \
  X(gnss_fixes) X(gnss_fix_lost) X(gnss_restarts) X(agnss_ok) X(agnss_fail) X(lbs_ok) X(lbs_fail) \
  X(obd_requests) X(obd_timeouts) X(obd_neg_resp) X(obd_reinit) \
  X(can_rx) X(can_tx) X(can_tx_fail) X(can_bus_off) \
  X(ota_ok) X(ota_fail) X(cmds) X(task_stalls)

struct Counters {
#define X(n) uint32_t n;
  DEVICE_COUNTERS(X)
#undef X
};

extern Counters g_cnt;

#define CNT_INC(field) __atomic_fetch_add(&g_cnt.field, 1, __ATOMIC_RELAXED)
#define CNT_ADD(field, v) __atomic_fetch_add(&g_cnt.field, (v), __ATOMIC_RELAXED)

// Iterate: forEachCounter([](const char* name, uint32_t v){ ... });
template <typename F>
void forEachCounter(F f) {
#define X(n) f(#n, g_cnt.n);
  DEVICE_COUNTERS(X)
#undef X
}

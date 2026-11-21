#include "core/supervisor.h"
#include <Preferences.h>
#include <esp_core_dump.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include "core/config.h"
#include "core/counters.h"
#include "core/log.h"
#include "core/msgbus.h"

static const char* TAG = "SUP";

// Tell the Arduino core not to auto-confirm a freshly flashed OTA image:
// we confirm it ourselves once MQTT works (see markHealthy()).
extern "C" bool verifyRollbackLater() { return true; }

// Survives software/panic/watchdog resets (not power loss).
struct RtcInfo {
  uint32_t magic;
  uint32_t abnormalResets;
  bool     pendingAbnormal;      // we rebooted on purpose because something was stuck
  char     stalledTask[16];
};
RTC_NOINIT_ATTR static RtcInfo s_rtc;
static const uint32_t RTC_MAGIC = 0x5AFE0001;

struct TaskSlot {
  const char*  name;
  TaskHandle_t handle;
  uint32_t     maxSilence;
  volatile uint32_t lastBeat;
  const char*  note;
};
static TaskSlot s_tasks[10];
static int s_nTasks = 0;

static bool s_safe = false;
static bool s_abnormal = false;
static bool s_otaPending = false;
static uint32_t s_bootCount = 0;
static char s_reason[24];
static char s_crash[200];
static char s_stall[16];
static bool s_stableMarked = false;
static bool s_forcedSafe = false;

static const char* reasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power_on";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int_watchdog";
    case ESP_RST_TASK_WDT:  return "task_watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

static void readCoreDump() {
  s_crash[0] = 0;
  if (esp_core_dump_image_check() != ESP_OK) return;
  esp_core_dump_summary_t* sum = (esp_core_dump_summary_t*)malloc(sizeof(esp_core_dump_summary_t));
  if (!sum) return;
  if (esp_core_dump_get_summary(sum) == ESP_OK) {
    int n = snprintf(s_crash, sizeof s_crash, "task=%s pc=0x%08lx bt=", sum->exc_task,
                     (unsigned long)sum->exc_pc);
    for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 8 && n < (int)sizeof s_crash - 12; i++)
      n += snprintf(s_crash + n, sizeof s_crash - n, "%08lx ", (unsigned long)sum->exc_bt_info.bt[i]);
  }
  free(sum);
  esp_core_dump_image_erase();  // report once
}

namespace sup {

void begin() {
  esp_reset_reason_t rr = esp_reset_reason();
  strlcpy(s_reason, reasonStr(rr), sizeof s_reason);

  if (s_rtc.magic != RTC_MAGIC || rr == ESP_RST_POWERON) {
    memset(&s_rtc, 0, sizeof s_rtc);
    s_rtc.magic = RTC_MAGIC;
  }
  s_abnormal = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
               rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT || s_rtc.pendingAbnormal;
  // brownouts (engine cranking) are reported but are not firmware faults
  bool firmwareFault = s_abnormal && rr != ESP_RST_BROWNOUT;
  strlcpy(s_stall, s_rtc.stalledTask, sizeof s_stall);
  s_rtc.stalledTask[0] = 0;
  s_rtc.pendingAbnormal = false;
  if (firmwareFault) s_rtc.abnormalResets++;

  Preferences nv;
  nv.begin("sys", false);
  s_bootCount = nv.getUInt("boots", 0) + 1;
  nv.putUInt("boots", s_bootCount);
  bool forcedSafe = nv.getBool("safe", false);
  nv.end();

  s_forcedSafe = forcedSafe;
  s_safe = forcedSafe || s_rtc.abnormalResets >= (uint32_t)cfg().safe_resets;

  readCoreDump();

  const esp_partition_t* run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) s_otaPending = true;

  // Hardware task watchdog: 30 s, panic (-> reboot + core dump) on expiry
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdt = {.timeout_ms = 30000, .idle_core_mask = 0, .trigger_panic = true};
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
#else
  esp_task_wdt_init(30, true);  // IDF 4.4: re-configures the already running watchdog
#endif

  LOGI(TAG, "boot #%lu reason=%s abnormal=%d (streak %lu)%s%s", (unsigned long)s_bootCount, s_reason,
       s_abnormal, (unsigned long)s_rtc.abnormalResets, s_safe ? " SAFE MODE" : "",
       s_otaPending ? " OTA-PENDING-VERIFY" : "");
  if (s_crash[0]) LOGE(TAG, "last crash: %s", s_crash);
  if (s_stall[0]) LOGE(TAG, "rebooted because task '%s' stalled", s_stall);
}

static portMUX_TYPE s_regMux = portMUX_INITIALIZER_UNLOCKED;

int registerTask(const char* name, uint32_t maxSilenceMs, bool hwWatchdog) {
  portENTER_CRITICAL(&s_regMux);
  if (s_nTasks >= (int)(sizeof s_tasks / sizeof s_tasks[0])) { portEXIT_CRITICAL(&s_regMux); return -1; }
  int id = s_nTasks;
  s_tasks[id] = {name, xTaskGetCurrentTaskHandle(), maxSilenceMs, millis(), ""};
  s_nTasks++;
  portEXIT_CRITICAL(&s_regMux);
  if (hwWatchdog) esp_task_wdt_add(nullptr);
  return id;
}

void beat(int id) {
  if (id >= 0 && id < s_nTasks) s_tasks[id].lastBeat = millis();
  esp_task_wdt_reset();
}

void feed() {
  TaskHandle_t me = xTaskGetCurrentTaskHandle();
  for (int i = 0; i < s_nTasks; i++)
    if (s_tasks[i].handle == me) { s_tasks[i].lastBeat = millis(); break; }
  esp_task_wdt_reset();  // harmless error if this task is not subscribed
}

void setNote(int id, const char* note) {
  if (id >= 0 && id < s_nTasks) s_tasks[id].note = note;
}

void reboot(const char* why, bool abnormal) {
  LOGW(TAG, "reboot: %s", why);
  s_rtc.pendingAbnormal = abnormal;
  delay(300);
  Serial.flush();
  esp_restart();
}

void tick() {
  uint32_t now = millis();
  for (int i = 0; i < s_nTasks; i++) {
    TaskSlot& t = s_tasks[i];
    if (now - t.lastBeat > t.maxSilence) {
      CNT_INC(task_stalls);
      LOGE(TAG, "task '%s' stalled %lu ms (note: %s) -> reboot", t.name, (unsigned long)(now - t.lastBeat),
           t.note ? t.note : "");
      strlcpy(s_rtc.stalledTask, t.name, sizeof s_rtc.stalledTask);
      reboot("task stall", true);
    }
  }

  // 10 min without trouble -> forget the crash streak
  if (!s_stableMarked && now > 600000UL) {
    s_stableMarked = true;
    s_rtc.abnormalResets = 0;
    if (s_safe) LOGW(TAG, "stable for 10 min in safe mode");
  }
  // automatic safe mode ends after 1 h without trouble (manual 'safemode on' stays)
  if (s_safe && !s_forcedSafe && now > 3600000UL) {
    LOGW(TAG, "safe mode: 1 h stable -> rebooting into normal mode");
    s_rtc.abnormalResets = 0;
    reboot("leave safe mode");
  }

  // Unconfirmed OTA image past its deadline -> roll back
  if (s_otaPending && now > (uint32_t)cfg().ota_confirm_s * 1000UL) {
    LOGE(TAG, "new firmware not confirmed within %ld s -> ROLLBACK", (long)cfg().ota_confirm_s);
    delay(200);
    esp_ota_mark_app_invalid_rollback_and_reboot();
  }
}

bool safeMode() { return s_safe; }

void clearSafeMode() {
  Preferences nv;
  nv.begin("sys", false);
  nv.putBool("safe", false);
  nv.end();
  s_rtc.abnormalResets = 0;
  s_safe = false;
}

void markHealthy() {
  if (!s_otaPending) return;
  if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
    s_otaPending = false;
    CNT_INC(ota_ok);
    LOGI(TAG, "firmware image confirmed (rollback cancelled)");
    bus::event("ota_confirmed", "{}");
  }
}

bool otaPendingVerify() { return s_otaPending; }
const char* resetReason() { return s_reason; }
bool abnormalBoot() { return s_abnormal; }
uint32_t bootCount() { return s_bootCount; }
const char* lastCrash() { return s_crash; }
const char* lastStall() { return s_stall; }

void printTasks(Print& out) {
  out.printf("%-10s %8s %10s  %s\n", "task", "stackHWM", "lastBeat", "note");
  uint32_t now = millis();
  for (int i = 0; i < s_nTasks; i++) {
    TaskSlot& t = s_tasks[i];
    out.printf("%-10s %8u %8lums  %s\n", t.name, (unsigned)uxTaskGetStackHighWaterMark(t.handle),
               (unsigned long)(now - t.lastBeat), t.note ? t.note : "");
  }
  out.printf("heap free %u min %u | psram free %u\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
             (unsigned)ESP.getFreePsram());
}

}  // namespace sup

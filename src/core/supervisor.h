#pragma once
// Supervisor: keeps the device alive and diagnosable.
//  - Hardware task watchdog (30 s) + per-task logical heartbeats
//  - Reset reason, boot counter, last crash (core dump summary)
//  - Crash-loop detection -> SAFE MODE (only connectivity + OTA + commands run)
//  - OTA rollback: new firmware must call markHealthy() (MQTT connected)
//    within ota_confirm_s or the bootloader rolls back to the previous image.
#include <Arduino.h>

namespace sup {
void begin();                                     // call first in setup()
int  registerTask(const char* name, uint32_t maxSilenceMs, bool hwWatchdog = true);  // from inside the task
void beat(int id);                                // task is alive and making progress
void feed();                                      // keep-alive during long blocking waits
void setNote(int id, const char* note);           // breadcrumb, e.g. "mqtt connect"
void tick();                                      // run every ~1 s from the supervisor loop

bool safeMode();
void clearSafeMode();
void markHealthy();                               // confirms a pending OTA image
bool otaPendingVerify();

void reboot(const char* why, bool abnormal = false);
const char* resetReason();
bool abnormalBoot();
uint32_t bootCount();
const char* lastCrash();                          // "" if none
const char* lastStall();                          // task that stalled before last reboot
void printTasks(Print& out);
}  // namespace sup

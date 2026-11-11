#pragma once
// One command table for the USB console and remote MQTT commands.
//   console:  type "help"
//   MQTT:     publish to <root>/<id>/cmd  either plain text "status"
//             or JSON {"id":"42","cmd":"config set rpt_move_s 5"}
//             reply on <root>/<id>/cmd/resp {"id":"42","cmd":"...","ok":true,"out":"..."}
// Add a command: write a handler + one line in the table in commands.cpp.
#include <Arduino.h>

namespace cmd {
bool runLine(const char* line, Print& out);   // returns false for unknown command
void handleRemote(const char* payload);       // from MQTT (cellular task)
void startConsole();                          // USB serial console task
}  // namespace cmd

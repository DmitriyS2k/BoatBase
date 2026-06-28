#pragma once
#include <Arduino.h>

// ── Кольцевой лог событий ────────────────────────────────────────
// 50 записей × ~80 байт = ~4KB RAM — безопасно для ESP32
#define LOG_SIZE 50

struct LogEntry {
    uint32_t ms;        // millis() когда произошло
    char     msg[76];   // текст события
};

void logEvent(const char* fmt, ...);   // printf-style
void logGetJson(String &out);          // сериализация в JSON-массив

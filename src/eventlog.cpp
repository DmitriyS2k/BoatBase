#include "eventlog.h"
#include <stdarg.h>
#include <stdio.h>

static LogEntry logBuf[LOG_SIZE];
static int      logHead = 0;   // следующая позиция записи
static int      logCount = 0;  // сколько записей всего (макс LOG_SIZE)

void logEvent(const char* fmt, ...) {
    LogEntry &e = logBuf[logHead];
    e.ms = millis();
    va_list args;
    va_start(args, fmt);
    vsnprintf(e.msg, sizeof(e.msg), fmt, args);
    va_end(args);

    logHead = (logHead + 1) % LOG_SIZE;
    if (logCount < LOG_SIZE) logCount++;

    Serial.printf("[LOG %lu] %s\n", e.ms, e.msg);
}

// Возвращает JSON-массив от новых к старым
void logGetJson(String &out) {
    out = "[";
    bool first = true;
    // Идём от последней записи к первой
    for (int i = 1; i <= logCount; i++) {
        int idx = ((logHead - i) + LOG_SIZE) % LOG_SIZE;
        if (!first) out += ",";
        first = false;
        out += "{\"ms\":";
        out += logBuf[idx].ms;
        out += ",\"msg\":\"";
        // Экранируем кавычки
        const char *p = logBuf[idx].msg;
        while (*p) {
            if (*p == '"' || *p == '\\') out += '\\';
            out += *p++;
        }
        out += "\"}";
    }
    out += "]";
}

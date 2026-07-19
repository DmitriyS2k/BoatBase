#include "cruiselog.h"

#define CRUISELOG_DEFAULT_CAPACITY 1000
#define CRUISELOG_MAX_CAPACITY     2000

struct CruiseLogEntry {
    uint32_t t;               // millis()
    double   lat, lon;
    float    heading;
    float    lockedHeading;   // курс, зафиксированный при отпускании СН4
    float    err;             // ошибка курса относительно lockedHeading
    int16_t  ch4;              // сырой канал руля
    int16_t  manualSteer;      // ручное рулении (-100..100), 0 если СН4 в мёртвой зоне
    float    autoSteer;        // авто-коррекция удержания курса (когда СН4 отпущен)
    float    steerClamped;     // итоговая разница, применённая к моторам
    int16_t  motorLeft;
    int16_t  motorRight;
    int16_t  thrust;           // запрошенная базовая тяга (до центрирования)
};

static CruiseLogEntry *buf      = nullptr;
static int             capacity = 0;
static int             head     = 0;
static int             count    = 0;

static void cruiseLogEnsureAlloc() {
    if (!buf) {
        capacity = CRUISELOG_DEFAULT_CAPACITY;
        buf = new CruiseLogEntry[capacity];
    }
}

void cruiseLogSetCapacity(int newCapacity) {
    newCapacity = constrain(newCapacity, 100, CRUISELOG_MAX_CAPACITY);
    CruiseLogEntry *newBuf = new CruiseLogEntry[newCapacity];
    delete[] buf;
    buf      = newBuf;
    capacity = newCapacity;
    head     = 0;
    count    = 0;
}

void cruiseLogRecord(double lat, double lon, float heading, float lockedHeading, float err,
                      int ch4, int manualSteer, float autoSteer, float steerClamped,
                      int motorLeft, int motorRight, int thrust) {
    cruiseLogEnsureAlloc();
    CruiseLogEntry &e = buf[head];
    e.t             = millis();
    e.lat           = lat;
    e.lon           = lon;
    e.heading       = heading;
    e.lockedHeading = lockedHeading;
    e.err           = err;
    e.ch4           = (int16_t)ch4;
    e.manualSteer   = (int16_t)manualSteer;
    e.autoSteer     = autoSteer;
    e.steerClamped  = steerClamped;
    e.motorLeft     = (int16_t)motorLeft;
    e.motorRight    = (int16_t)motorRight;
    e.thrust        = (int16_t)thrust;

    head = (head + 1) % capacity;
    if (count < capacity) count++;
}

void cruiseLogClear() { head = 0; count = 0; }
int  cruiseLogCount()    { return count; }
int  cruiseLogCapacity() { cruiseLogEnsureAlloc(); return capacity; }

// ── Потоковая генерация CSV ─────────────────────────────────────
static int    fillIdx        = 0;
static int    fillSnapHead   = 0;
static int    fillSnapCount  = 0;
static bool   fillHeaderSent = false;
static String fillPending;

void cruiseLogFillReset() {
    fillIdx        = 0;
    fillSnapHead   = head;
    fillSnapCount  = count;
    fillHeaderSent = false;
    fillPending    = "";
}

size_t cruiseLogFillCsv(uint8_t *buffer, size_t maxLen, size_t /*index*/) {
    size_t written = 0;
    while (written < maxLen) {
        if (fillPending.length() == 0) {
            if (!fillHeaderSent) {
                fillPending = "t_ms,lat,lon,heading,lockedHeading,err,ch4,manualSteer,"
                              "autoSteer,steerClamped,motorL,motorR,thrust\n";
                fillHeaderSent = true;
            } else if (fillIdx < fillSnapCount) {
                int real = (fillSnapHead - fillSnapCount + fillIdx + capacity) % capacity;
                CruiseLogEntry &e = buf[real];
                char line[160];
                snprintf(line, sizeof(line),
                    "%lu,%.6f,%.6f,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%d,%d,%d\n",
                    (unsigned long)e.t, e.lat, e.lon, e.heading, e.lockedHeading, e.err,
                    (int)e.ch4, (int)e.manualSteer, e.autoSteer, e.steerClamped,
                    (int)e.motorLeft, (int)e.motorRight, (int)e.thrust);
                fillPending = line;
                fillIdx++;
            } else {
                break;  // всё отдано
            }
        }
        size_t toCopy = min(maxLen - written, (size_t)fillPending.length());
        memcpy(buffer + written, fillPending.c_str(), toCopy);
        written += toCopy;
        fillPending.remove(0, toCopy);
    }
    return written;
}

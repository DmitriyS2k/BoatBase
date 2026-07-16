#include "navlog.h"
#include "settings.h"

// Размер буфера настраивается на лету (см. navLogSetCapacity, /api/navlog/capacity).
// sizeof(NavLogEntry) ~72 байта: 1000 записей ~72КБ, 2000 ~144КБ в куче.
// При navInterval=200мс 1000 записей ~3.3 мин активной навигации.
#define NAVLOG_DEFAULT_CAPACITY 1000
#define NAVLOG_MAX_CAPACITY     2000

struct NavLogEntry {
    uint32_t t;              // millis()
    double   lat, lon;       // фактическая GPS-позиция
    float    heading;        // курс компаса
    float    targetHeading;  // сглаженный bearing на точку
    float    err;            // ошибка курса, поданная в PID (град, со знаком)
    float    pidRaw;         // сырой выход PID (до нелинейной коррекции)
    float    pidCurved;      // после pidCurve
    float    pidFinal;       // после сглаживания+maxDiff — реально применён к моторам
    int16_t  motorLeft;
    int16_t  motorRight;
    float    dist;           // до цели, м
    float    speedKmh;
    float    hdop;
    uint8_t  sats;
    uint8_t  wpTarget;
};

static NavLogEntry *buf      = nullptr;
static int          capacity = 0;
static int          head     = 0;   // следующий слот для записи
static int          count    = 0;   // сколько записей валидно (насыщается на capacity)

static void navLogEnsureAlloc() {
    if (!buf) {
        capacity = NAVLOG_DEFAULT_CAPACITY;
        buf = new NavLogEntry[capacity];
    }
}

// Пересоздаёт буфер под новый размер — старые записи теряются (это ОК,
// вызывается на воде вручную между заездами, не во время активного лога).
void navLogSetCapacity(int newCapacity) {
    newCapacity = constrain(newCapacity, 100, NAVLOG_MAX_CAPACITY);
    NavLogEntry *newBuf = new NavLogEntry[newCapacity];
    delete[] buf;
    buf      = newBuf;
    capacity = newCapacity;
    head     = 0;
    count    = 0;
}

void navLogRecord(double lat, double lon, float heading, float targetHeading, float err,
                   float pidRaw, float pidCurved, float pidFinal,
                   int motorLeft, int motorRight, float dist, float speedKmh,
                   int sats, float hdop, int wpTarget) {
    navLogEnsureAlloc();
    NavLogEntry &e = buf[head];
    e.t             = millis();
    e.lat           = lat;
    e.lon           = lon;
    e.heading       = heading;
    e.targetHeading = targetHeading;
    e.err           = err;
    e.pidRaw        = pidRaw;
    e.pidCurved     = pidCurved;
    e.pidFinal      = pidFinal;
    e.motorLeft     = (int16_t)motorLeft;
    e.motorRight    = (int16_t)motorRight;
    e.dist          = dist;
    e.speedKmh      = speedKmh;
    e.hdop          = hdop;
    e.sats          = (uint8_t)sats;
    e.wpTarget      = (uint8_t)wpTarget;

    head = (head + 1) % capacity;
    if (count < capacity) count++;
}

void navLogClear() { head = 0; count = 0; }
int  navLogCount()    { return count; }
int  navLogCapacity() { navLogEnsureAlloc(); return capacity; }

// ── Потоковая генерация CSV ─────────────────────────────────────
// Курсор — глобальное состояние, рассчитано на одно скачивание за раз
// (личный отладочный инструмент, не публичный сервис).
static int    fillIdx         = 0;
static int    fillSnapHead    = 0;
static int    fillSnapCount   = 0;
static bool   fillConfigSent  = false;
static bool   fillHeaderSent  = false;
static String fillPending;

// Снимок head/count на момент старта скачивания — чтобы новые записи,
// пришедшие пока файл качается, не сдвигали уже отдаваемые данные.
void navLogFillReset() {
    fillIdx        = 0;
    fillSnapHead    = head;
    fillSnapCount   = count;
    fillConfigSent  = false;
    fillHeaderSent  = false;
    fillPending     = "";
}

size_t navLogFillCsv(uint8_t *buffer, size_t maxLen, size_t /*index*/) {
    size_t written = 0;
    while (written < maxLen) {
        if (fillPending.length() == 0) {
            if (!fillConfigSent) {
                // Настройки на момент скачивания — чтобы CSV было понятно,
                // при каких параметрах записан этот заезд
                char cfgLine[200];
                snprintf(cfgLine, sizeof(cfgLine),
                    "# pidKp=%.2f pidKi=%.2f pidKd=%.2f pidCurve=%.2f maxDiff=%d "
                    "bearingAlpha=%.2f navInterval=%d turnSlowFloor=%.2f cruiseSpeed=%d\n",
                    cfg.pidKp, cfg.pidKi, cfg.pidKd, cfg.pidCurve, cfg.maxDiff,
                    cfg.bearingAlpha, cfg.navInterval, cfg.turnSlowFloor, cfg.cruiseSpeed);
                fillPending = cfgLine;
                fillConfigSent = true;
            } else if (!fillHeaderSent) {
                fillPending = "t_ms,lat,lon,heading,targetHeading,err,pidRaw,pidCurved,pidFinal,"
                              "motorL,motorR,dist,speedKmh,sats,hdop,wpTarget\n";
                fillHeaderSent = true;
            } else if (fillIdx < fillSnapCount) {
                // Записи хранятся по кругу — идём от самой старой к новой
                int real = (fillSnapHead - fillSnapCount + fillIdx + capacity) % capacity;
                NavLogEntry &e = buf[real];
                char line[160];
                snprintf(line, sizeof(line),
                    "%lu,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%d,%.1f,%d\n",
                    (unsigned long)e.t, e.lat, e.lon, e.heading, e.targetHeading, e.err,
                    e.pidRaw, e.pidCurved, e.pidFinal, (int)e.motorLeft, (int)e.motorRight,
                    e.dist, e.speedKmh, (int)e.sats, e.hdop, (int)e.wpTarget);
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

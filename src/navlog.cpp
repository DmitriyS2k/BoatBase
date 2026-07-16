#include "navlog.h"
#include "settings.h"

// ~1000 записей: при navInterval=200мс это ~3.3 мин активной навигации,
// при 500мс — ~8 мин. Хватает на один заплыв к точке и обратно.
// sizeof(NavLogEntry) ~72 байта → буфер ~72КБ RAM (свободно ~270КБ).
#define NAVLOG_SIZE 1000

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

static NavLogEntry buf[NAVLOG_SIZE];
static int head  = 0;   // следующий слот для записи
static int count = 0;   // сколько записей валидно (насыщается на NAVLOG_SIZE)

void navLogRecord(double lat, double lon, float heading, float targetHeading, float err,
                   float pidRaw, float pidCurved, float pidFinal,
                   int motorLeft, int motorRight, float dist, float speedKmh,
                   int sats, float hdop, int wpTarget) {
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

    head = (head + 1) % NAVLOG_SIZE;
    if (count < NAVLOG_SIZE) count++;
}

void navLogClear() { head = 0; count = 0; }
int  navLogCount()    { return count; }
int  navLogCapacity() { return NAVLOG_SIZE; }

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
                    "bearingAlpha=%.2f navInterval=%d cruiseSpeed=%d\n",
                    cfg.pidKp, cfg.pidKi, cfg.pidKd, cfg.pidCurve, cfg.maxDiff,
                    cfg.bearingAlpha, cfg.navInterval, cfg.cruiseSpeed);
                fillPending = cfgLine;
                fillConfigSent = true;
            } else if (!fillHeaderSent) {
                fillPending = "t_ms,lat,lon,heading,targetHeading,err,pidRaw,pidCurved,pidFinal,"
                              "motorL,motorR,dist,speedKmh,sats,hdop,wpTarget\n";
                fillHeaderSent = true;
            } else if (fillIdx < fillSnapCount) {
                // Записи хранятся по кругу — идём от самой старой к новой
                int real = (fillSnapHead - fillSnapCount + fillIdx + NAVLOG_SIZE) % NAVLOG_SIZE;
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

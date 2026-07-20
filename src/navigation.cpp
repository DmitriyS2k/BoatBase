#include <Arduino.h>
#include <math.h>
#include "state.h"
#include "settings.h"
#include "eventlog.h"
#include "navlog.h"
#include "cruiselog.h"

// ── Геодезия (Haversine) ────────────────────────────────────────
static float toRad(float deg) { return deg * M_PI / 180.0f; }

float geoDistance(double lat1, double lon1, double lat2, double lon2) {
    const float R = 6371000.0f;
    float dlat = toRad(lat2 - lat1);
    float dlon = toRad(lon2 - lon1);
    float a = sinf(dlat/2)*sinf(dlat/2) +
              cosf(toRad(lat1))*cosf(toRad(lat2))*sinf(dlon/2)*sinf(dlon/2);
    return R * 2.0f * atan2f(sqrtf(a), sqrtf(1-a));
}

float geoBearing(double lat1, double lon1, double lat2, double lon2) {
    float dlon = toRad(lon2 - lon1);
    float y = sinf(dlon) * cosf(toRad(lat2));
    float x = cosf(toRad(lat1))*sinf(toRad(lat2)) -
               sinf(toRad(lat1))*cosf(toRad(lat2))*cosf(dlon);
    float b = atan2f(y, x) * 180.0f / M_PI;
    while (b < 0)    b += 360.0f;
    while (b >= 360) b -= 360.0f;
    return b;
}

// ── Line-of-sight наведение (альтернатива прицеливанию прямо в точку) ──
// Обычный режим (geoBearing прямо в точку) при старте не с прямой линии
// старт→цель даёт плавную дугу вместо прямой (лодка постоянно перецеливается
// на конечную точку по мере движения). LOS вместо этого целится в "морковку" —
// точку на самой линии старт→цель, на losLookahead метров впереди проекции
// лодки на эту линию — так лодка сходится на прямую и дальше идёт по ней.
// Плоское приближение (метры) — точно для дистанций в сотни метров.
static float losBearing(double curLat, double curLon,
                          double startLat, double startLon,
                          double wpLat, double wpLon, float lookaheadM) {
    const float R = 6371000.0f;
    float latRad = toRad((float)startLat);
    float tx = toRad((float)(wpLon - startLon))  * R * cosf(latRad);
    float ty = toRad((float)(wpLat - startLat))  * R;
    float bx = toRad((float)(curLon - startLon)) * R * cosf(latRad);
    float by = toRad((float)(curLat - startLat)) * R;

    float lineLen = sqrtf(tx*tx + ty*ty);
    if (lineLen < 1.0f) {
        // старт и цель почти совпадают — целимся прямо в точку
        return geoBearing(curLat, curLon, wpLat, wpLon);
    }
    float dirX = tx / lineLen, dirY = ty / lineLen;
    float t = bx*dirX + by*dirY;                       // проекция лодки на линию
    t = constrain(t, 0.0f, lineLen);
    float carrotT = constrain(t + lookaheadM, 0.0f, lineLen);
    float cx = dirX * carrotT, cy = dirY * carrotT;     // "морковка" на линии

    float dx = cx - bx, dy = cy - by;
    if (fabsf(dx) < 0.01f && fabsf(dy) < 0.01f) {
        // лодка почти в самой "морковке" (например, уже у цели) — целимся в саму цель
        return geoBearing(curLat, curLon, wpLat, wpLon);
    }
    float b = atan2f(dx, dy) * 180.0f / M_PI;  // x=восток,y=север → компас (0=север)
    while (b < 0)    b += 360.0f;
    while (b >= 360) b -= 360.0f;
    return b;
}

// ── Вычисление минимального угла ошибки курса (из Katerok) ──────
// Возвращает -180..+180. Мёртвая зона ±2° — не реагируем на шум
static float headingError(float target, float current) {
    int err = (int)(target - current);
    int dz  = (int)cfg.compassDeadzone;
    if (err <= dz && err >= -dz) return 0;
    else if (err > 180)  err = err % 180 - 180;
    else if (err < -180) err = 180 + err % 180;
    return (float)err;
}

// ── Каскадная навигация (ветка cascade-nav) ─────────────────────
// PID из AUTO убран. Вместо него два контура:
//   внешний (раз в cfg.navInterval): дистанция, bearing на точку, замедления
//     → цель курса navTargetHeading и тяга navThrust;
//   внутренний (каждый вызов, ~200Гц из loop()): держит navTargetHeading тем же
//     P-регулятором, что и круиз (err × cruiseGain) — круиз на воде доказал,
//     что по компасу этот контур не рыскает.
// В петле управления нет ни I/D, ни сглаживания выхода — нет и запаздывания,
// которое в PID-варианте превращало демпфирование в раскачку.
static float smoothBearing = -1.0f;  // курс на точку, сглаженный bearingAlpha
static double navStartLat = 0.0, navStartLon = 0.0;  // позиция старта текущей ноги (для LOS)
static bool   navStartPending = true;                 // ещё не захватили старт этой ноги
static float    navTargetHeading = -1.0f;  // цель внутреннего контура; <0 = не задана
static int      navThrust        = 1500;   // тяга, посчитанная внешним контуром
static uint32_t navOuterMs       = 0;      // millis() последнего шага внешнего контура
static float    navLastDist      = 0.0f;   // дистанция с последнего внешнего шага (для лога)

// ── Учёт "тихих" пропусков навигации ────────────────────────────
// Ранний выход из navigationStep не пишет строку в navlog — в логе это
// выглядит как загадочная дыра (см. разбор navlog 5-17). Отмечаем начало
// пропуска (один раз за эпизод) и длительность при возобновлении.
static uint32_t navSkipSince = 0;

static void navSkipMark(const char *reason) {
    if (navSkipSince == 0) {
        navSkipSince = millis();
        logEvent("NAV skip: %s", reason);
    }
}

// Сброс состояния навигации при начале новой ноги
void navigationReset() {
    navTargetHeading = -1.0f;
    navThrust        = 1500;
    navOuterMs       = 0;
    navSkipSince     = 0;
    smoothBearing = -1.0f;  // забыть курс на предыдущую точку — иначе первые
                             // секунды новой ноги гонимся за старой целью
    navStartPending = true; // захватить новую стартовую точку для LOS при следующем шаге
}

// ── Структура выхода моторов ─────────────────────────────────────
struct MotorOut { int left, right; };

// ── Навигация к точке — каскад ──────────────────────────────────
// ВАЖНО: в отличие от PID-версии, вызывается на КАЖДОМ loop() (~200Гц),
// как круиз — внешний контур сам ограничивает себя cfg.navInterval.
MotorOut navigationStep(int speedLimit) {
    Waypoint &wp = boat.waypoints[boat.wpTarget];  // едем к wpTarget, не к wpSelected

    if (!wp.valid || !boat.gpsFix || boat.hdop > 5.0f) {
        navSkipMark(!wp.valid ? "WP invalid" : "no fix / hdop>5");
        return {1500, 1500};
    }

    uint32_t now = millis();
    bool outerRan = false;
    if (navTargetHeading < 0.0f || now - navOuterMs >= (uint32_t)cfg.navInterval) {
        navOuterMs = now;
        outerRan = true;

        float dist = geoDistance(boat.latitude, boat.longitude, wp.lat, wp.lon);
        boat.distToTarget = dist;
        navLastDist = dist;

        // Глюк GPS: точка дальше 500 м — стоп
        if (dist > 500.0f) {
            navSkipMark("dist>500m (GPS glitch?)");
            return {1500, 1500};
        }

        // Пропуск закончился — зафиксировать длительность дыры в логе
        if (navSkipSince != 0) {
            logEvent("NAV resumed after %.1fs skip", (millis() - navSkipSince) / 1000.0f);
            navSkipSince = 0;
        }

        // Приплыли
        if (dist < cfg.arrivalRadius) {
            boat.navigating    = false;
            navigationReset();
            logEvent("Arrived at WP%d!", boat.wpTarget);
            return {1500, 1500};
        }

        // Захват стартовой точки ноги для LOS — только на первом валидном шаге
        // после сброса (см. navigationReset), чтобы линия старт→цель не съезжала
        if (navStartPending) {
            navStartLat = boat.latitude;
            navStartLon = boat.longitude;
            navStartPending = false;
        }

        float rawBearing = (cfg.navMode == 1)
            ? losBearing(boat.latitude, boat.longitude, navStartLat, navStartLon,
                         wp.lat, wp.lon, cfg.losLookahead)
            : geoBearing(boat.latitude, boat.longitude, wp.lat, wp.lon);
        // Сглаживаем bearing (bearingAlpha) — GPS-прыжки на 1-3м не должны
        // дёргать цель внутреннего контура
        if (smoothBearing < 0.0f) {
            smoothBearing = rawBearing;  // первый цикл — без фильтра
        } else {
            // Учитываем переход через 0/360
            float diff = rawBearing - smoothBearing;
            if (diff > 180.0f)  diff -= 360.0f;
            if (diff < -180.0f) diff += 360.0f;
            smoothBearing += cfg.bearingAlpha * diff;
            if (smoothBearing < 0.0f)   smoothBearing += 360.0f;
            if (smoothBearing >= 360.0f) smoothBearing -= 360.0f;
        }
        navTargetHeading  = smoothBearing;
        boat.targetHeading = smoothBearing;

        // Тяга: лимит с крутилки ch6, замедление у точки и на резком повороте
        int topSpeed = min(cfg.cruiseSpeed, speedLimit);
        topSpeed = max(topSpeed, 1510);  // не ниже минимума движения

        int maxSpeed = topSpeed;
        if (dist < cfg.slowdownDist && cfg.slowdownDist > 0.1f) {
            float factor = dist / cfg.slowdownDist;
            int slowMin = min(cfg.slowdownSpeed, speedLimit);
            maxSpeed = (int)(slowMin + (topSpeed - slowMin) * factor);
        }

        // Замедление на резком повороте: режем поступательную тягу, чтобы
        // разворот шёл почти на месте, без заноса по инерции.
        float errAbs = fabsf(headingError(navTargetHeading, boat.heading));
        if (errAbs > 30.0f) {
            float turnFactor = constrain(1.0f - (errAbs - 30.0f) / 90.0f, cfg.turnSlowFloor, 1.0f);
            int turnSpeed = 1500 + (int)((maxSpeed - 1500) * turnFactor);
            maxSpeed = min(maxSpeed, turnSpeed);
        }
        navThrust = maxSpeed;
    }

    // ── Внутренний контур: математика 1-в-1 из cruiseStep ───────
    // err × cruiseGain, кламп ±maxDiff/2, центрирование тяги. Каждый вызов.
    float err = headingError(navTargetHeading, boat.heading);
    float autoSteer = constrain(err * cfg.cruiseGain, -100.0f, 100.0f);
    float steerClamped = constrain(autoSteer, -(float)cfg.maxDiff/2, (float)cfg.maxDiff/2);
    int maxSteer = (int)fabsf(steerClamped);
    int centeredThrust = constrain(navThrust, 1000, 2000 - maxSteer);
    int left  = constrain((int)(centeredThrust + steerClamped), 1000, 2000);
    int right = constrain((int)(centeredThrust - steerClamped), 1000, 2000);

    // Лог — на частоте внешнего контура, формат CSV прежний.
    // В колонках PID теперь: pidRaw = err×gain до клампов,
    // pidCurved = после клампов, pidFinal = фактическая полуразница моторов.
    if (outerRan) {
        navLogRecord(boat.latitude, boat.longitude, boat.heading, navTargetHeading, err,
                     autoSteer, steerClamped, (left - right) / 2.0f,
                     left, right, navLastDist, boat.speed,
                     boat.satellites, boat.hdop, boat.wpTarget);
    }

    return {left, right};
}

// ── Круиз-контроль (ручной режим с удержанием курса) ───────────
// ch3 > 1750 → активен, ch4 — руление.
//
// Раньше: ch4 крутил моторы напрямую (открытый контур, регулятор не смотрел
// на курс), а в момент отпускания резко включался P-регулятор на "курс на
// момент отпускания" — реальная инерция поворота (~0.3с) проносила нос мимо
// зафиксированного курса, пока регулятор спохватится (заброс до ~35°+
// постоянный уход вправо на неисправленном mechanical trim).
//
// Теперь: ch4 крутит cruiseTargetHeading — само число желаемого курса
// (cfg.cruiseSteerRate °/с при полном отклонении), а регулятор ВСЕГДА
// в замкнутом контуре держит эту цель, крутится она сейчас или стоит на
// месте. Мода "открытый контур" больше нет — заброса по инерции при
// отпускании стика в разы меньше, потому что регулятор ни на миг не
// переставал следить за курсом.
static float cruiseTargetHeading = -1.0f;
static uint32_t cruiseLastMs = 0;

MotorOut cruiseStep(int thrust, int ch4) {
    thrust = constrain(thrust, 1000, 2000);

    if (thrust < 1510) {
        cruiseTargetHeading = -1.0f;
        cruiseLastMs = 0;
        return {1500, 1500};
    }

    uint32_t now = millis();
    float dt = (cruiseLastMs == 0) ? 0.0f : (now - cruiseLastMs) / 1000.0f;
    if (dt > 0.5f) dt = 0.0f;  // защита от скачка после долгой паузы (первый шаг, лаги loop())
    cruiseLastMs = now;

    // ch4: 1000..2000 → -100..100, мёртвая зона ±8
    int steerRaw = map(constrain(ch4, 1000, 2000), 1000, 2000, -100, 100);
    int ch4Deflection = (abs(steerRaw) > 8) ? steerRaw : 0;

    if (cruiseTargetHeading < 0.0f) {
        // Первый шаг после включения круиза — стартуем с текущего курса
        cruiseTargetHeading = boat.heading;
    } else if (ch4Deflection != 0 && dt > 0.0f) {
        cruiseTargetHeading += (ch4Deflection / 100.0f) * cfg.cruiseSteerRate * dt;
        while (cruiseTargetHeading < 0.0f)   cruiseTargetHeading += 360.0f;
        while (cruiseTargetHeading >= 360.0f) cruiseTargetHeading -= 360.0f;
    }

    float err = headingError(cruiseTargetHeading, boat.heading);
    float autoSteer = constrain(err * cfg.cruiseGain, -100.0f, 100.0f);

    // Ограничиваем разницу моторов через cfg.maxDiff
    float steerClamped = constrain(autoSteer, -(float)cfg.maxDiff/2, (float)cfg.maxDiff/2);
    // Центрируем тягу чтобы коррекция была симметричной при высоком газе
    // Если thrust высокий, опускаем оба мотора чтобы было место для коррекции вверх
    int maxSteer = (int)fabsf(steerClamped);
    int centeredThrust = constrain(thrust, 1000, 2000 - maxSteer);
    int left  = constrain((int)(centeredThrust + steerClamped), 1000, 2000);
    int right = constrain((int)(centeredThrust - steerClamped), 1000, 2000);

    cruiseLogRecord(boat.latitude, boat.longitude, boat.heading, cruiseTargetHeading, err,
                     ch4, ch4Deflection, autoSteer, steerClamped,
                     left, right, thrust);

    return {left, right};
}

void cruiseReset() {
    cruiseTargetHeading = -1.0f;
    cruiseLastMs = 0;
}

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

// ── PID-регулятор (адаптирован из Katerok) ──────────────────────
// (текущий_курс, целевой_курс, kp, ki, kd, dt_sec, min, max)
static float pidIntegral   = 0.0f;
static float pidPrevErr    = 0.0f;
static float smoothPid     = 0.0f;
static float smoothBearing = -1.0f;  // курс на точку, сглаженный bearingAlpha
static double navStartLat = 0.0, navStartLon = 0.0;  // позиция старта текущей ноги (для LOS)
static bool   navStartPending = true;                 // ещё не захватили старт этой ноги

static int computePID(float currentHeading, float targetHeading,
                      float kp, float ki, float kd,
                      float dt, int minOut, int maxOut) {
    float err = headingError(targetHeading, currentHeading);

    // Интегральная составляющая с ограничением (anti-windup)
    pidIntegral = constrain(pidIntegral + err * dt * ki, (float)minOut, (float)maxOut);

    // Дифференциальная составляющая
    float D = (err - pidPrevErr) / dt;
    pidPrevErr = err;

    float out = err * kp + pidIntegral + D * kd;
    return (int)constrain(out, (float)minOut, (float)maxOut);
}

// Нелинейная коррекция выхода PID (pidCurve).
// 550-моторы слишком мощные для линейного PID: мелкий шум курса (пара градусов)
// даёт заметную разницу PWM → корабль дёргается влево-вправо.
// curve=1.0 — без изменений (линейно, как раньше).
// curve>1.0 — у центра (маленькие ошибки) выход подавляется сильнее, чем у краёв
// (большие ошибки) — гасит рысканье от шума, но не теряет силу для реальных отклонений.
// curve<1.0 — обратный эффект (резче у центра), обычно не нужен.
static float applyPidCurve(float val, float range, float curve) {
    if (range < 1.0f || curve <= 0.01f) return val;
    float norm = constrain(val / range, -1.0f, 1.0f);
    float sign = (norm < 0.0f) ? -1.0f : 1.0f;
    float curved = sign * powf(fabsf(norm), curve);
    return curved * range;
}

// Сброс PID при начале новой навигации
void navigationReset() {
    pidIntegral   = 0.0f;
    pidPrevErr    = 0.0f;
    smoothPid     = 0.0f;
    smoothBearing = -1.0f;  // забыть курс на предыдущую точку — иначе первые
                             // секунды новой ноги гонимся за старой целью
    navStartPending = true; // захватить новую стартовую точку для LOS при следующем шаге
}

// ── Структура выхода моторов ─────────────────────────────────────
struct MotorOut { int left, right; };

// ── Навигация к точке (алгоритм из Katerok + наш PWM) ──────────
// Период вызова: 100мс (из main.cpp — вызываем только раз в 100мс)
MotorOut navigationStep(int speedLimit) {
    Waypoint &wp = boat.waypoints[boat.wpTarget];  // едем к wpTarget, не к wpSelected

    if (!wp.valid || !boat.gpsFix || boat.hdop > 5.0f) {
        return {1500, 1500};
    }

    float dist = geoDistance(boat.latitude, boat.longitude, wp.lat, wp.lon);
    boat.distToTarget = dist;

    // Глюк GPS: точка дальше 500 м — стоп
    if (dist > 500.0f) {
        return {1500, 1500};
    }

    // Приплыли (< 1м — стоп как в Katerok, без ожидания нуля)
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
    // Сглаживаем bearing фильтром низких частот (alpha=0.1) чтобы убрать GPS шум
    // Без фильтра GPS прыжки на 1-3м меняют bearing на несколько градусов → рывки
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
    float bearing = smoothBearing;
    boat.targetHeading = bearing;
    float err = headingError(bearing, boat.heading);

    // Максимальная мощность: как в Katerok снижаем при приближении
    // Katerok: < 3м → скорость/2. Адаптируем к PWM 1500..1900
    // Максимальная скорость: минимум из настройки и лимита с крутилки ch6
    int topSpeed = min(cfg.cruiseSpeed, speedLimit);
    topSpeed = max(topSpeed, 1510);  // не ниже минимума движения

    int maxSpeed = topSpeed;
    if (dist < cfg.slowdownDist && cfg.slowdownDist > 0.1f) {
        float factor = dist / cfg.slowdownDist;
        // slowdownSpeed тоже ограничиваем лимитом
        int slowMin = min(cfg.slowdownSpeed, speedLimit);
        maxSpeed = (int)(slowMin + (topSpeed - slowMin) * factor);
    }

    // Замедление на резком повороте: если ошибка курса большая, лодка на
    // полном ходу входит в разворот как в занос — за счёт инерции проскакивает
    // нужный курс, пока цикл управления (navInterval) это заметит ("змейка").
    // Режем поступательную тягу пропорционально ошибке — разворот получается
    // почти на месте, без заноса. errAbs<=30° — без изменений, >=120° — до floor.
    float errAbs = fabsf(err);
    if (errAbs > 30.0f) {
        float turnFactor = constrain(1.0f - (errAbs - 30.0f) / 90.0f, cfg.turnSlowFloor, 1.0f);
        int turnSpeed = 1500 + (int)((maxSpeed - 1500) * turnFactor);
        maxSpeed = min(maxSpeed, turnSpeed);
    }

    // PID: dt соответствует navInterval
    float dt = cfg.navInterval / 1000.0f;
    int halfRange = maxSpeed - 1500;
    int pidRaw = computePID(boat.heading, bearing,
                            cfg.pidKp, cfg.pidKi, cfg.pidKd,
                            dt, -halfRange, halfRange);

    // Нелинейная коррекция — смягчает реакцию на мелкие ошибки курса
    float pidCurved = applyPidCurve((float)pidRaw, (float)halfRange, cfg.pidCurve);

    // Сглаживание выхода PID — убирает резкие скачки моторов
    // alpha=0.3: новое значение на 30%, старое на 70% → плавные изменения
    smoothPid = smoothPid * 0.7f + pidCurved * 0.3f;
    int pidOut = constrain((int)smoothPid, -halfRange, halfRange);
    pidOut = constrain(pidOut, -cfg.maxDiff, cfg.maxDiff);

    // pidOut > 0 → нужно повернуть налево → притормозить правый мотор
    // pidOut < 0 → нужно повернуть направо → притормозить левый мотор
    // Центрирование тяги — как в круизе:
    // оба мотора симметрично относительно maxSpeed
    // один прибавляет, другой убавляет на одинаковую величину
    int pidClamped = constrain(pidOut, -cfg.maxDiff, cfg.maxDiff);
    int absPid = abs(pidClamped);
    int centeredSpeed = constrain(maxSpeed, 1000, 2000 - absPid);
    int fast = centeredSpeed + absPid;
    int slow = centeredSpeed - absPid;

    // Реальный потолок КПД моторов (проверено на воде: ~1750-1800, не 2000) —
    // выше него добавка PWM почти не даёт тяги. Если "быстрый" борт упёрся
    // в потолок, излишек перекладываем на "медленный" — иначе часть заданной
    // разницы тяги впустую улетает выше потолка вместо реального поворота.
    if (fast > cfg.motorMaxPwm) {
        int excess = fast - cfg.motorMaxPwm;
        fast -= excess;
        slow -= excess;
    }

    int left  = (pidClamped >= 0) ? fast : slow;
    int right = (pidClamped >= 0) ? slow : fast;

    left  = constrain(left,  1000, 2000);
    right = constrain(right, 1000, 2000);

    // Лог для анализа/отладки — та же ошибка курса, что видел PID
    navLogRecord(boat.latitude, boat.longitude, boat.heading, bearing, err,
                 (float)pidRaw, pidCurved, (float)pidOut,
                 left, right, dist, boat.speed,
                 boat.satellites, boat.hdop, boat.wpTarget);

    return {left, right};
}

// ── Круиз-контроль (ручной режим с удержанием курса) ───────────
// ch3 > 1750 → активен, ch4 — ручное подруливание
static float cruiseLockedHeading = -1.0f;

MotorOut cruiseStep(int thrust, int ch4) {
    thrust = constrain(thrust, 1000, 2000);

    if (thrust < 1510) {
        cruiseLockedHeading = -1.0f;
        return {1500, 1500};
    }

    // ch4: 1000..2000 → -100..100, мёртвая зона ±8 (было ±15 — слишком большая)
    int steerRaw    = map(constrain(ch4, 1000, 2000), 1000, 2000, -100, 100);
    int manualSteer = (abs(steerRaw) > 8) ? steerRaw : 0;

    if (manualSteer != 0) {
        cruiseLockedHeading = boat.heading;
    } else if (cruiseLockedHeading < 0) {
        cruiseLockedHeading = boat.heading;
    }

    float err = headingError(cruiseLockedHeading, boat.heading);
    // Расширяем диапазон автокоррекции до ±100 (было ±50)
    float autoSteer = constrain(err * cfg.cruiseGain, -100.0f, 100.0f);

    float totalSteer = (manualSteer != 0) ? (float)manualSteer : autoSteer;
    totalSteer = constrain(totalSteer, -100.0f, 100.0f);

    // Ограничиваем разницу моторов через cfg.maxDiff
    float steerClamped = constrain(totalSteer, -(float)cfg.maxDiff/2, (float)cfg.maxDiff/2);
    // Центрируем тягу чтобы коррекция была симметричной при высоком газе
    // Если thrust высокий, опускаем оба мотора чтобы было место для коррекции вверх
    int maxSteer = (int)fabsf(steerClamped);
    int centeredThrust = constrain(thrust, 1000, 2000 - maxSteer);
    int left  = constrain((int)(centeredThrust + steerClamped), 1000, 2000);
    int right = constrain((int)(centeredThrust - steerClamped), 1000, 2000);

    cruiseLogRecord(boat.latitude, boat.longitude, boat.heading, cruiseLockedHeading, err,
                     ch4, manualSteer, autoSteer, steerClamped,
                     left, right, thrust);

    return {left, right};
}

void cruiseReset() {
    cruiseLockedHeading = -1.0f;
}

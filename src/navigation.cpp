#include <Arduino.h>
#include <math.h>
#include "state.h"
#include "settings.h"

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
static float pidIntegral = 0.0f;
static float pidPrevErr  = 0.0f;
static float smoothPid   = 0.0f;
static float smoothBearingG = -1.0f;

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

// Сброс PID при начале новой навигации
void navigationReset() {
    pidIntegral = 0.0f;
    pidPrevErr  = 0.0f;
    smoothPid = 0.0f;
    // smoothBearing сбросится в navigationStep при следующем старте
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
        boat.beepArrived.active  = true;
        boat.beepArrived.startMs = millis();
        navigationReset();
        Serial.printf("Arrived at WP %d!\n", boat.wpTarget);
        return {1500, 1500};
    }

    float rawBearing = geoBearing(boat.latitude, boat.longitude, wp.lat, wp.lon);
    // Сглаживаем bearing фильтром низких частот (alpha=0.1) чтобы убрать GPS шум
    // Без фильтра GPS прыжки на 1-3м меняют bearing на несколько градусов → рывки
    static float smoothBearing = -1.0f;
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

    // PID: dt соответствует navInterval
    float dt = cfg.navInterval / 1000.0f;
    int halfRange = maxSpeed - 1500;
    int pidRaw = computePID(boat.heading, bearing,
                            cfg.pidKp, cfg.pidKi, cfg.pidKd,
                            dt, -halfRange, halfRange);

    // Сглаживание выхода PID — убирает резкие скачки моторов
    // alpha=0.3: новое значение на 30%, старое на 70% → плавные изменения
    smoothPid = smoothPid * 0.7f + (float)pidRaw * 0.3f;
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
    int left  = constrain(centeredSpeed + pidClamped, 1000, 2000);
    int right = constrain(centeredSpeed - pidClamped, 1000, 2000);

    left  = constrain(left,  1000, 2000);
    right = constrain(right, 1000, 2000);

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

    return {left, right};
}

void cruiseReset() {
    cruiseLockedHeading = -1.0f;
}

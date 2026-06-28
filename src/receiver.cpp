#include <cstddef>   // fix NULL not declared in IBusBM.h
#include <IBusBM.h>
#include <HardwareSerial.h>
#include "state.h"

// ── Распределение UART ──────────────────────────────────────────
// UART0 (Serial0) : USB отладка + iBUS сенсор (GPIO32/33)
// UART1           : GPS модуль RX=GPIO16
// UART2           : iBUS каналы RX=GPIO4
//
// ── Сенсоры телеметрии (OpenI6X) ───────────────────────────────
//  1  RPM   — режим:      0=MANUAL  1=SAVE_WP  2=AUTO
//  2  RPM   — выбранная точка: 0..3
//  3  RPM   — спутники GPS (0..30)
//  4  RPM   — дистанция до точки (м): в AUTO→к цели, иначе→к дому
//  5  RPM   — скорость км/ч × 10  (т.е. 12.3 км/ч → 123)
//  6  TEMP  — звук A: сохранение точки / GPS нашёл ≥5 спутников
//  7  TEMP  — звук B: ошибка сохранения (нет GPS) — два быстрых пика
//  8  TEMP  — звук C: прибытие на точку
//
// Настройка тревог на пульте (OpenI6X → Telemetry → Sensor → Alarm):
//   Сенсор 6 Alarm: > 100°C  (value > 1400) — тональность A
//   Сенсор 7 Alarm: > 100°C  (value > 1400) — тональность B (другой звук)
//   Сенсор 8 Alarm: > 100°C  (value > 1400) — тональность C (другой звук)
//
// Значение в норме: 400 (= 0°C в формате TEMP: (celsius+40)*10)
// Значение тревоги: 1900 (= 150°C) — выше любого разумного порога
//
// «Два пика» (beepNoGps): чередуем 1900 / 400 / 1900 по ~220мс каждый.

#define IBUS_SERVO_RX_PIN  4
#define IBUS_SENS_RX_PIN   32
#define IBUS_SENS_TX_PIN   33
#define RC_TIMEOUT_MS      500

// TEMP формат: value = (celsius + 40) * 10
// 0°C (норма) → 400, 150°C (тревога) → 1900
#define TEMP_NORMAL  400
#define TEMP_ALARM   1900

// Длительности звуков
#define BEEP_SAVED_MS    800   // сохранение точки — короткий
#define BEEP_GPSFIX_MS   600   // GPS нашёл спутники — короткий
#define BEEP_ARRIVED_MS  1200  // прибытие — длинный
// «Два пика»: пик-пауза-пик, итого ~660мс
#define BEEP_NOGPS_PULSE_MS  220  // длина одного пика/паузы

IBusBM ibusServo;
IBusBM ibusSensor;
HardwareSerial IBusServoSerial(2);
HardwareSerial IBusSensorSerial(0);

static uint8_t sensMode;     // 1: режим
static uint8_t sensWp;       // 2: выбранная точка
static uint8_t sensSats;     // 3: спутники
static uint8_t sensDist;     // 4: дистанция
static uint8_t sensSpeed;    // 5: скорость
static uint8_t sensBeepA;    // 6: звук — успех (сохранение / GPS fix)
static uint8_t sensBeepB;    // 7: звук — ошибка (нет GPS при сохранении)
static uint8_t sensBeepC;    // 8: звук — прибытие

// ── Вспомогательная функция: обновить один TEMP-сенсор-звук ────
// Возвращает TEMP_ALARM пока beep активен, иначе TEMP_NORMAL.
// Автоматически сбрасывает beep.active по истечении durationMs.
static uint16_t beepValue(BeepState &b, uint32_t durationMs) {
    if (!b.active) return TEMP_NORMAL;
    if (millis() - b.startMs >= durationMs) {
        b.active = false;
        return TEMP_NORMAL;
    }
    return TEMP_ALARM;
}

// ── Два быстрых пика для sensBeepB (нет GPS) ───────────────────
// Фаза 0: пик  (0..220мс)   → ALARM
// Фаза 1: пауза(220..440мс) → NORMAL
// Фаза 2: пик  (440..660мс) → ALARM
// Фаза 3: конец(>660мс)     → NORMAL + сброс
static uint16_t beepDoubleValue(BeepState &b) {
    if (!b.active) return TEMP_NORMAL;
    uint32_t t = millis() - b.startMs;
    if (t >= BEEP_NOGPS_PULSE_MS * 3) {
        b.active = false;
        return TEMP_NORMAL;
    }
    uint32_t phase = t / BEEP_NOGPS_PULSE_MS;  // 0, 1, 2
    return (phase == 1) ? TEMP_NORMAL : TEMP_ALARM;  // фаза 1 — пауза
}

void receiverInit() {
    IBusServoSerial.begin(115200, SERIAL_8N1, IBUS_SERVO_RX_PIN, -1);
    ibusServo.begin(IBusServoSerial, IBUSBM_NOTIMER);

    IBusSensorSerial.begin(115200, SERIAL_8N1, IBUS_SENS_RX_PIN, IBUS_SENS_TX_PIN);
    ibusSensor.begin(IBusSensorSerial, IBUSBM_NOTIMER);

    // Переинициализируем GPS после сенсора — UART0 init может сбросить UART1
    delay(50);
    extern HardwareSerial GPSSerial;
    GPSSerial.end();
    delay(10);
    GPSSerial.begin(9600, SERIAL_8N1, 16, -1);

    // Регистрируем сенсоры в нужном порядке
    sensMode  = ibusSensor.addSensor(IBUSS_RPM);   // 1: режим
    sensWp    = ibusSensor.addSensor(IBUSS_RPM);   // 2: точка
    sensSats  = ibusSensor.addSensor(IBUSS_RPM);   // 3: спутники
    sensDist  = ibusSensor.addSensor(IBUSS_RPM);   // 4: дистанция
    sensSpeed = ibusSensor.addSensor(IBUSS_RPM);   // 5: скорость
    sensBeepA = ibusSensor.addSensor(IBUSS_TEMP);  // 6: звук A
    sensBeepB = ibusSensor.addSensor(IBUSS_TEMP);  // 7: звук B
    sensBeepC = ibusSensor.addSensor(IBUSS_TEMP);  // 8: звук C
}

void receiverUpdate() {
    ibusSensor.loop();

    // ── Каналы управления ───────────────────────────────────────
    static uint8_t lastCnt = 0;
    uint8_t curCnt = ibusServo.cnt_rec;
    bool newPacket = (curCnt != lastCnt);

    if (newPacket) {
        lastCnt = curCnt;
        boat.lastRcMs = millis();
        for (int i = 0; i < 14; i++) {
            uint16_t v = ibusServo.readChannel(i);
            if (i < 6) {
                // OpenI6X: аналоговые каналы в 16-бит → маппируем в 1000-2000
                uint16_t minV, maxV;
                if (v > 53000)      { minV = 54236; maxV = 55260; }  // тип B
                else if (v > 30000) { minV = 50140; maxV = 51164; }  // тип A
                else if (v > 800)   { minV = 21468; maxV = 22492; }  // тип C
                else                { boat.ch[i] = v; continue; }
                int mapped = 1000 + (int)((long)(constrain(v, minV, maxV) - minV) * 1000 / (maxV - minV));
                boat.ch[i] = (uint16_t)constrain(mapped, 1000, 2000);
            } else if (i < 10) {
                if (v >= 800 && v <= 2200)
                    boat.ch[i] = v;
            }
        }
    }

    boat.rcConnected = (millis() - boat.lastRcMs) < RC_TIMEOUT_MS;

    // ── Сенсор 1: режим ─────────────────────────────────────────
    ibusSensor.setSensorMeasurement(sensMode, (uint16_t)boat.mode);

    // ── Сенсор 2: выбранная точка ────────────────────────────────
    ibusSensor.setSensorMeasurement(sensWp, (uint16_t)boat.wpSelected);

    // ── Сенсор 3: спутники ───────────────────────────────────────
    ibusSensor.setSensorMeasurement(sensSats, (uint16_t)boat.satellites);

    // ── Сенсор 4: дистанция ──────────────────────────────────────
    // AUTO (едем) → до цели (wpTarget)
    // Иначе       → до дома (WP0), если есть фикс и точка сохранена
    uint16_t dist = 0;
    if (boat.mode == MODE_AUTO && boat.navigating) {
        dist = (uint16_t)constrain((int)boat.distToTarget, 0, 9999);
    } else if (boat.waypoints[0].valid && boat.gpsFix) {
        double lat1 = boat.latitude,  lon1 = boat.longitude;
        double lat2 = boat.waypoints[0].lat, lon2 = boat.waypoints[0].lon;
        float dlat = (float)((lat2 - lat1) * 0.01745329);
        float dlon = (float)((lon2 - lon1) * 0.01745329);
        float a = sinf(dlat/2)*sinf(dlat/2) +
                  cosf((float)(lat1*0.01745329))*cosf((float)(lat2*0.01745329))*
                  sinf(dlon/2)*sinf(dlon/2);
        dist = (uint16_t)constrain((int)(6371000.0f * 2.0f * atan2f(sqrtf(a), sqrtf(1-a))), 0, 9999);
    }
    ibusSensor.setSensorMeasurement(sensDist, dist);

    // ── Сенсор 5: скорость км/ч × 10 ────────────────────────────
    // Умножаем на 10 чтобы показывать одно знаковое место (12.3 → 123)
    // OpenI6X RPM сенсор: делитель настраивается на пульте, ставь /10
    ibusSensor.setSensorMeasurement(sensSpeed, (uint16_t)(boat.speed * 10.0f));

    // ── Сенсор 6: звук A — успех (сохранение точки / GPS fix) ───
    // beepSaved и beepGpsFix разделяют один сенсор:
    // оба "хороших" события — один и тот же звук на пульте.
    // Приоритет: beepSaved > beepGpsFix
    uint16_t valA = TEMP_NORMAL;
    if (boat.beepSaved.active) {
        valA = beepValue(boat.beepSaved, BEEP_SAVED_MS);
    } else if (boat.beepGpsFix.active) {
        valA = beepValue(boat.beepGpsFix, BEEP_GPSFIX_MS);
    }
    ibusSensor.setSensorMeasurement(sensBeepA, valA);

    // ── Сенсор 7: звук B — ошибка (нет GPS при сохранении) ──────
    ibusSensor.setSensorMeasurement(sensBeepB, beepDoubleValue(boat.beepNoGps));

    // ── Сенсор 8: звук C — прибытие на точку ────────────────────
    ibusSensor.setSensorMeasurement(sensBeepC, beepValue(boat.beepArrived, BEEP_ARRIVED_MS));
}

int rcNorm(int ch_idx) {
    int v = constrain(boat.ch[ch_idx], 1000, 2000);
    return map(v, 1000, 2000, -100, 100);
}

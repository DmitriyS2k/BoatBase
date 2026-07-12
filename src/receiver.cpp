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
static uint8_t sensEvent;    // 5: событие (0=тихо, 1=сохранено/GPS, 2=нет GPS, 3=прибытие)

// ── Вспомогательная функция: обновить один TEMP-сенсор-звук ────
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

    // IBusBM нумерует первый сенсор с индекса 0, но пульт показывает inst начиная с 1.
    // Instance 0 пульт игнорирует — добавляем заглушку чтобы сдвинуть нумерацию.
    ibusSensor.addSensor(IBUSS_RPM);               // 0: заглушка (inst0, пульт не видит)
    sensMode  = ibusSensor.addSensor(IBUSS_RPM);   // 1: режим    (inst1)
    sensWp    = ibusSensor.addSensor(IBUSS_RPM);   // 2: точка    (inst2)
    sensSats  = ibusSensor.addSensor(IBUSS_RPM);   // 3: спутники (inst3)
    sensDist  = ibusSensor.addSensor(IBUSS_RPM);   // 4: дистанция(inst4)
    sensEvent = ibusSensor.addSensor(IBUSS_RPM);   // 5: событие  (inst5)
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
    // ── Сенсор 6: событие (RPM) ──────────────────────────────────
    // 0 = тихо, 1 = сохранено/GPS fix, 2 = нет GPS, 3 = прибытие
    // Приоритет: прибытие > ошибка > успех
    // Сбрасываем флаги после истечения времени
    static uint16_t eventVal = 0;
    static uint32_t eventEnd = 0;
    uint32_t now = millis();

    if (now < eventEnd) {
        // событие ещё активно — держим значение
    } else if (boat.beepArrived.active) {
        eventVal = 3;
        eventEnd = now + 3000;  // 3 сек — пульт точно успеет опросить
        boat.beepArrived.active = false;
    } else if (boat.beepNoGps.active) {
        eventVal = 2;
        eventEnd = now + 3000;
        boat.beepNoGps.active = false;
    } else if (boat.beepSaved.active) {
        eventVal = 1;
        eventEnd = now + 3000;
        boat.beepSaved.active = false;
    } else if (boat.beepGpsFix.active) {
        eventVal = 1;
        eventEnd = now + 3000;
        boat.beepGpsFix.active = false;
    } else {
        eventVal = 0;
    }
    ibusSensor.setSensorMeasurement(sensEvent, eventVal);

}

int rcNorm(int ch_idx) {
    int v = constrain(boat.ch[ch_idx], 1000, 2000);
    return map(v, 1000, 2000, -100, 100);
}

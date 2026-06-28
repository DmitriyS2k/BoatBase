#include <Wire.h>
#include "state.h"
#include "settings.h"

#define QMC5883P_ADDR 0x2C

TwoWire I2CCOMPASS(1);

static float filteredHeading = 0.0f;
static bool  firstRead = true;

static int16_t calXOffset = 0;
static int16_t calYOffset = 0;

static int16_t calMinX =  32767, calMaxX = -32768;
static int16_t calMinY =  32767, calMaxY = -32768;
static uint32_t calStartMs = 0;
static int      calPointCount = 0;
#define CAL_DURATION_MS  15000

// Сырые значения для диагностики (доступны снаружи)
int16_t compassRawX = 0;
int16_t compassRawY = 0;
int16_t compassRawZ = 0;

void compassInit() {
    I2CCOMPASS.begin(21, 22);
    I2CCOMPASS.setTimeOut(10);  // 10мс таймаут I2C — не блокируем loop надолго

    I2CCOMPASS.beginTransmission(QMC5883P_ADDR);
    I2CCOMPASS.write(0x0A);
    I2CCOMPASS.write(0xCF);
    I2CCOMPASS.endTransmission();
    delay(10);

    I2CCOMPASS.beginTransmission(QMC5883P_ADDR);
    I2CCOMPASS.write(0x0B);
    I2CCOMPASS.write(0x08);
    I2CCOMPASS.endTransmission();
    delay(100);

    calXOffset = (int16_t)cfg.compassXOffset;
    calYOffset = (int16_t)cfg.compassYOffset;
}

static float wrapAngle(float a) {
    while (a < 0)    a += 360;
    while (a >= 360) a -= 360;
    return a;
}

void compassCalibStart() {
    calMinX =  32767; calMaxX = -32768;
    calMinY =  32767; calMaxY = -32768;
    calPointCount = 0;
    calStartMs = millis();
    boat.calibrating = true;
    Serial.println("Compass calibration started — rotate the boat!");
}

static void compassCalibFinish() {
    boat.calibrating = false;
    if (calPointCount < 20) {
        Serial.println("Calibration FAILED — too few points");
        return;
    }
    calXOffset = (calMaxX + calMinX) / 2;
    calYOffset = (calMaxY + calMinY) / 2;
    cfg.compassXOffset = calXOffset;
    cfg.compassYOffset = calYOffset;
    settingsSave();
    firstRead = true;
    Serial.printf("Calibration OK: XOff=%d YOff=%d pts=%d\n",
                  calXOffset, calYOffset, calPointCount);
}

int compassCalibProgress() {
    if (!boat.calibrating) return 0;
    uint32_t elapsed = millis() - calStartMs;
    uint32_t pct = elapsed * 100 / CAL_DURATION_MS;
    return (int)(pct > 100 ? 100 : pct);
}

// ── Формула угла: зависит от ориентации чипа ───────────────────
// cfg.compassAxis задаёт вариант:
//   0 = atan2( cy,  cx)   — стандарт, X→восток  Y→север  (чип маркировкой вверх, X вперёд)
//   1 = atan2( cx,  cy)   — X→север  Y→восток              (чип повёрнут 90°)
//   2 = atan2(-cy,  cx)   — X→восток  Y→юг      (чип перевёрнут Y)
//   3 = atan2( cy, -cx)   — X→запад  Y→север    (чип перевёрнут X)
//   4 = atan2(-cx, -cy)   — X→юг     Y→запад    (чип повёрнут 180°)
//   5 = atan2(-cx,  cy)   — X→север  Y→запад
//   6 = atan2( cx, -cy)   — X→юг     Y→восток
//   7 = atan2(-cy, -cx)   — X→запад  Y→юг
// Подбирается через страницу "Компас" в веб-интерфейсе.
static float calcHeading(float cx, float cy) {
    float angle;
    switch ((int)cfg.compassAxis) {
        case 1: angle = atan2f( cx,  cy); break;
        case 2: angle = atan2f(-cy,  cx); break;
        case 3: angle = atan2f( cy, -cx); break;
        case 4: angle = atan2f(-cx, -cy); break;
        case 5: angle = atan2f(-cx,  cy); break;
        case 6: angle = atan2f( cx, -cy); break;
        case 7: angle = atan2f(-cy, -cx); break;
        default: angle = atan2f( cy,  cx); break;  // 0
    }
    // Переводим из математического угла (CCW от востока)
    // в навигационный (CW от севера)
    float heading = 90.0f - angle * 180.0f / M_PI;
    return wrapAngle(heading + cfg.compassDecl);
}

void compassUpdate() {
    // Ограничиваем частоту — каждые 20мс (50Гц достаточно для компаса)
    // Блокирующий I2C при каждом loop() вызывает пропуски пакетов iBUS
    static uint32_t lastCompassMs = 0;
    if (millis() - lastCompassMs < 20) return;
    lastCompassMs = millis();

    I2CCOMPASS.beginTransmission(QMC5883P_ADDR);
    I2CCOMPASS.write(0x09);
    I2CCOMPASS.endTransmission(false);
    if (I2CCOMPASS.requestFrom(QMC5883P_ADDR, 1) != 1) return;
    uint8_t status = I2CCOMPASS.read();
    if (!(status & 0x01)) return;

    I2CCOMPASS.beginTransmission(QMC5883P_ADDR);
    I2CCOMPASS.write(0x01);
    I2CCOMPASS.endTransmission(false);
    if (I2CCOMPASS.requestFrom(QMC5883P_ADDR, 6) != 6) return;

    compassRawX = (int16_t)(I2CCOMPASS.read() | (I2CCOMPASS.read() << 8));
    compassRawY = (int16_t)(I2CCOMPASS.read() | (I2CCOMPASS.read() << 8));
    compassRawZ = (int16_t)(I2CCOMPASS.read() | (I2CCOMPASS.read() << 8));

    if (boat.calibrating) {
        if (compassRawX < calMinX) { calMinX = compassRawX; calPointCount++; }
        if (compassRawX > calMaxX) { calMaxX = compassRawX; calPointCount++; }
        if (compassRawY < calMinY) { calMinY = compassRawY; calPointCount++; }
        if (compassRawY > calMaxY) { calMaxY = compassRawY; calPointCount++; }
        if (millis() - calStartMs >= CAL_DURATION_MS) compassCalibFinish();
    }

    float cx = (float)(compassRawX - calXOffset);
    float cy = (float)(compassRawY - calYOffset);

    float raw = calcHeading(cx, cy);

    if (firstRead) {
        filteredHeading = raw;
        firstRead = false;
    } else {
        float diff = raw - filteredHeading;
        if (diff >  180) diff -= 360;
        if (diff < -180) diff += 360;
        filteredHeading = wrapAngle(filteredHeading + 0.15f * diff);
    }

    boat.heading = filteredHeading;
}

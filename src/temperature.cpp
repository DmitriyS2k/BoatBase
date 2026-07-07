#include <OneWire.h>
#include <DallasTemperature.h>
#include "state.h"

#define TEMP_PIN        27
#define TEMP_INTERVAL   2000  // читаем каждые 2 секунды

static OneWire           oneWire(TEMP_PIN);
static DallasTemperature sensors(&oneWire);

void temperatureInit() {
    sensors.begin();
    sensors.setResolution(11);       // 11 бит — точность 0.125°C, конвертация ~375мс
    sensors.setWaitForConversion(false);  // неблокирующий режим
    sensors.requestTemperatures();   // первый запрос
}

void temperatureUpdate() {
    static uint32_t lastRequest = 0;
    static uint32_t lastRead    = 0;
    uint32_t now = millis();

    // Читаем результат через 400мс после запроса
    if (now - lastRequest >= 400 && now - lastRead >= 400) {
        float t = sensors.getTempCByIndex(0);
        if (t != DEVICE_DISCONNECTED_C && t > -50.0f && t < 125.0f) {
            boat.motorTemp = t;
        }
        lastRead = now;
    }

    // Новый запрос каждые 2 секунды
    if (now - lastRequest >= TEMP_INTERVAL) {
        sensors.requestTemperatures();
        lastRequest = now;
    }
}

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "state.h"
#include "gps.h"

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

void gpsInit() {
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

static void gpsReinit() {
    GPSSerial.end();
    delay(100);
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[GPS] Watchdog: reinit UART");
}

void gpsReset() {
    // NEO-6M — чип u-blox, $PMTK (MediaTek) он не понимает, поэтому нужен
    // бинарный UBX-CFG-RST (класс 0x06, ID 0x04):
    //   navBbrMask=0xFFFF — стереть эфемериды/альманах/позицию/время (холодный старт)
    //   resetMode=0x02    — мягкий рестарт только GNSS-движка, без сброса UART/чипа
    static const uint8_t UBX_COLD_START[] = {
        0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61
    };
    GPSSerial.write(UBX_COLD_START, sizeof(UBX_COLD_START));
    Serial.println("[GPS] UBX cold reset sent (NEO-6M)");
}

void gpsUpdate() {
    while (GPSSerial.available())
        gps.encode(GPSSerial.read());

    // ── Watchdog: если символов нет 5 секунд — переинициализируем ──
    static uint32_t lastCheck   = 0;
    static uint32_t lastChars   = 0;
    static uint32_t noDataSince = 0;

    if (millis() - lastCheck >= 1000) {
        uint32_t chars = gps.charsProcessed();
        if (chars != lastChars) {
            // Данные идут — всё хорошо
            boat.gpsActive = true;
            noDataSince    = millis();
            lastChars      = chars;
        } else {
            boat.gpsActive = false;
            // Watchdog reinit убран — GPSSerial.end() конфликтует с iBUS сенсором
            // Если GPS завис — перезагрузи корабль вручную
        }
        lastCheck = millis();
    }

    // ВАЖНО: isValid() значит «хоть раз распарсилось с момента загрузки»,
    // а НЕ «данные свежие». Без проверки age() модуль, единожды поймавший
    // фикс, будет вечно показывать те же координаты/спутники даже если
    // сигнал давно пропал или модуль завис/отключился.
    #define GPS_MAX_AGE_MS 2000
    bool satFresh = gps.satellites.isValid() && gps.satellites.age() < GPS_MAX_AGE_MS;
    bool hdopFresh = gps.hdop.isValid()      && gps.hdop.age()      < GPS_MAX_AGE_MS;
    bool locFresh  = gps.location.isValid()  && gps.location.age()  < GPS_MAX_AGE_MS;

    boat.satellites = satFresh  ? gps.satellites.value() : 0;
    boat.hdop       = hdopFresh ? gps.hdop.hdop()         : 99.0f;

    bool goodFix = locFresh
                && boat.hdop < 5.0f
                && boat.satellites >= 3;

    boat.gpsFix = goodFix;

    if (goodFix) {
        boat.latitude  = gps.location.lat();
        boat.longitude = gps.location.lng();
    }

    if (goodFix && gps.speed.isValid()) {
        float spd = gps.speed.kmph();
        if (spd < 30.0f)
            boat.speed = boat.speed * 0.7f + spd * 0.3f;
    } else {
        boat.speed = 0.0f;
    }

    boat.gpsHeading = (goodFix && gps.course.isValid())
                    ? gps.course.deg() : boat.heading;
}

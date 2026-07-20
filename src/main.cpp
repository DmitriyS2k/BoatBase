#include <Arduino.h>
#include "state.h"
#include "settings.h"
#include "gps.h"
#include "compass.h"
#include "receiver.h"
#include "motors.h"
#include "navigation.h"
#include "temperature.h"
#include "web.h"
#include "eventlog.h"

static int wpFromCh5(uint16_t v) {
    if (v <= 1502) return 0;   // Дом:    1500-1502 (и ниже)
    if (v <= 1506) return 1;   // Точка1: 1503-1506
    if (v <= 1511) return 2;   // Точка2: 1507-1511
    return 3;                  // Точка3: 1512-1517 (и выше)
}

static BoatMode modeFromCh9(uint16_t v) {
    if (v < 1250) return MODE_MANUAL;
    if (v < 1750) return MODE_SAVE_WP;
    return MODE_AUTO;
}

static BoatMode prevMode       = MODE_MANUAL;
static int      prevWpSel      = -1;
static bool     prevRc         = false;
static bool     prevGpsFix     = false;
static bool     rcEverConnected = false;  // пульт хотя бы раз подключался

// Стики ch1/ch2 в нейтрали (пульт не трогают)
static bool rcStickNeutral() {
    return (boat.ch[0] > 1400 && boat.ch[0] < 1600)
        && (boat.ch[1] > 1400 && boat.ch[1] < 1600);
}

void setup() {
    Serial.begin(115200);
    settingsLoad();
    gpsInit();          // GPS первым на UART1
    compassInit();
    motorsInit();
    motorsStop();
    temperatureInit();
    receiverInit();     // Сенсор последним — после GPS
    webInit();
    logEvent("System started");
    Serial.println("Boat ready!");
}

void loop() {
    // Ограничиваем loop до 200Гц (5мс) — стабильный период для iBUS сенсора
    // и WiFi стека. На управление не влияет — ESC всё равно медленнее.
    static uint32_t loopTimer = 0;
    while (millis() - loopTimer < 5) { vTaskDelay(1); }
    loopTimer = millis();

    gpsUpdate();
    compassUpdate();
    receiverUpdate();
    temperatureUpdate();
    webUpdate();

    // ── Логируем изменение связи с пультом ──────────────────────
    if (boat.rcConnected != prevRc) {
        prevRc = boat.rcConnected;
        logEvent(boat.rcConnected ? "RC connected" : "RC LOST");
    }
    if (boat.rcConnected) rcEverConnected = true;

    // ── Логируем появление/потерю GPS фикса ─────────────────────
    if (boat.gpsFix != prevGpsFix) {
        prevGpsFix = boat.gpsFix;
        logEvent(boat.gpsFix ? "GPS fix OK (%d sats)" : "GPS fix LOST (%d sats)",
                 boat.satellites);
    }

    // ── Потеря пульта ────────────────────────────────────────────
    if (!boat.rcConnected) {
        // Если пульт ни разу не подключался — просто стоим и ждём
        // Авто-возврат только если пульт БЫЛ подключён и потерялся
        if (!rcEverConnected) {
            motorsStop();
            boat.navigating = false;
            boat.mode = MODE_MANUAL;
            return;
        }

        if (boat.gpsFix && boat.satellites >= cfg.minSatellites
            && boat.waypoints[0].valid) {
            boat.mode    = MODE_AUTO;
            boat.wpTarget = 0;
            if (!boat.navigating) {
                boat.navigating = true;
                navigationReset();
                logEvent("RC lost — auto-return HOME");
            }
            // Каскад: внутренний контур держит курс на каждом loop (как круиз),
            // внешний сам ограничен navInterval внутри navigationStep
            MotorOut m = navigationStep(boat.speedLimitPwm);
            motorsWrite(m.left, m.right);
        } else {
            if (boat.navigating || boat.mode != MODE_MANUAL) {
                logEvent("RC lost — STOP (no GPS/WP)");
            }
            motorsStop();
            boat.navigating = false;
            boat.mode = MODE_MANUAL;
        }
        return;
    }

    // ── Режим по ch9 ─────────────────────────────────────────────
    BoatMode newMode = modeFromCh9(boat.ch[8]);
    if (newMode != prevMode) {
        const char* names[] = {"MANUAL","SAVE_WP","AUTO"};
        logEvent("Mode → %s", names[(int)newMode]);
        if (newMode == MODE_MANUAL) {
            motorsStop();
            boat.navigating = false;
            cruiseReset();
            navigationReset();
        }
        prevMode = newMode;
    }
    boat.mode = newMode;

    // ── Лимит скорости по крутилке ch6 (индекс 5) ───────────────
    // 1000..2000 → 50%..100% от cfg.cruiseSpeed
    // Минимум 50% чтобы корабль всегда двигался в AUTO
    {
        int ch6 = boat.ch[5];
        float pct = 0.5f + 0.5f * (float)(constrain(ch6, 1000, 2000) - 1000) / 1000.0f;
        boat.speedLimitPwm = (int)(1500 + (cfg.cruiseSpeed - 1500) * pct);
    }
    int newWp = wpFromCh5(boat.ch[4]);
    if (newWp != prevWpSel) {
        prevWpSel = newWp;
        boat.wpSelected = newWp;
        const char* wpNames[] = {"Home","WP1","WP2","WP3"};
        logEvent("WP selected: %s", wpNames[newWp]);
    }
    boat.wpSelected = newWp;

    // ══════════════════════════════════════════════════════════════
    if (boat.mode == MODE_MANUAL) {
        // Таймаут веб-джойстика: если 300мс нет новых команд — сбрасываем
        if (boat.webJoyActive && (millis() - boat.webJoyLastMs > 300)) {
            boat.webJoyActive = false;
            boat.webJoyLeft   = 1500;
            boat.webJoyRight  = 1500;
        }

        int ch3 = boat.ch[2];
        int ch4 = boat.ch[3];

        // CH3 — стик без пружины, без реверса, диапазон 1000-2000
        // Нижние 10% (1000-1100) = выключен круиз
        // Остальные 90% (1100-2000) = круиз включён, скорость пропорциональна положению
        // Маппируем 1100-2000 → минимальная тяга (1520) до максимальной (cfg.cruiseSpeed)
        if (ch3 > 1100) {
            // Вычисляем скорость из положения стика
            int cruiseThrust = map(constrain(ch3, 1100, 2000), 1100, 2000, 1520, cfg.cruiseSpeed);
            MotorOut m = cruiseStep(cruiseThrust, ch4);
            motorsWrite(m.left, m.right);
        } else if (boat.webJoyActive && rcStickNeutral()) {
            // Веб-джойстик активен И пульт в нейтрали — управляет телефон
            motorsWrite(boat.webJoyLeft, boat.webJoyRight);
        } else {
            cruiseReset();
            // Экспоненциальный фильтр на стики — сглаживает резкое отпускание.
            // Без фильтра: отпустил стик с 1900 → ESP32 ещё ~300-500мс видит
            // промежуточные значения пока стик физически возвращается в центр.
            // С фильтром 0.4: каждый цикл (loop ~5мс) значение быстро тянется к цели.
            // Мёртвая зона ±60: как только стик рядом с нейтралью — стоп.
            static float filtL = 1500, filtR = 1500;
            float rawL = (float)boat.ch[0];
            float rawR = (float)boat.ch[1];
            filtL = filtL + 0.4f * (rawL - filtL);
            filtR = filtR + 0.4f * (rawR - filtR);
            int l = (int)filtL;
            int r = (int)filtR;
            if (l > 1440 && l < 1560) l = 1500;
            if (r > 1440 && r < 1560) r = 1500;
            motorsWrite(l, r);
        }
        return;
    }

    if (boat.mode == MODE_SAVE_WP) {
        motorsStop();
        // CH4 > 1800 → включить WiFi, CH4 < 1200 → выключить
        {
            int ch4 = boat.ch[3];
            if (ch4 > 1800 && !getWifiEnabled()) setWifiEnabled(true);
            if (ch4 < 1200 && getWifiEnabled())  setWifiEnabled(false);
        }
        static uint32_t saveDelay = 0;
        // Сохраняем только если GPS качественный: фикс + HDOP < 3 + минимум спутников
        bool gpsGood = boat.gpsFix
                    && boat.hdop < 3.0f
                    && boat.satellites >= cfg.minSatellites;
        if (boat.ch[0] >= 1600 && gpsGood) {
            if (millis() - saveDelay > 600) {
                saveDelay = millis();
                Waypoint &wp = boat.waypoints[boat.wpSelected];
                wp.lat   = boat.latitude;
                wp.lon   = boat.longitude;
                wp.valid = true;
                waypointSave(boat.wpSelected);
                logEvent("WP%d saved: %.5f, %.5f (hdop=%.1f sat=%d)",
                         boat.wpSelected, wp.lat, wp.lon,
                         boat.hdop, boat.satellites);
            }
        } else if (boat.ch[0] >= 1600 && !gpsGood) {
            // Пробуем сохранить но GPS плохой — логируем и пищим два раза
            static uint32_t warnDelay = 0;
            if (millis() - warnDelay > 2000) {
                warnDelay = millis();
                logEvent("WP save rejected: hdop=%.1f sat=%d fix=%d",
                         boat.hdop, boat.satellites, boat.gpsFix);
            }
        }
        return;
    }

    if (boat.mode == MODE_AUTO) {
        bool gpsOk = boat.gpsFix
                  && boat.hdop < 5.0f
                  && (boat.satellites >= cfg.minSatellites);
        if (!gpsOk) {
            if (boat.navigating) logEvent("AUTO stopped — no GPS");
            motorsStop();
            boat.navigating = false;
            return;
        }

        // ch2 <= 1100 → GO HOME (точка 0), независимо от ch5
        static bool prevGoHome = false;
        bool goHome = (boat.ch[1] <= 1400) && boat.waypoints[0].valid;
        if (goHome && !prevGoHome) {
            boat.wpTarget   = 0;
            boat.navigating = true;
            navigationReset();
            logEvent("GO HOME (WP0)");
        }
        prevGoHome = goHome;

        // ch2 >= 1900 → GO TO выбранной точки (ch5)
        static bool prevGoWp = false;
        bool goWp = (boat.ch[1] >= 1600) && boat.waypoints[boat.wpSelected].valid;
        if (goWp && !prevGoWp) {
            float dist = geoDistance(
                boat.latitude, boat.longitude,
                boat.waypoints[boat.wpSelected].lat,
                boat.waypoints[boat.wpSelected].lon);
            if (dist <= 500.0f) {
                boat.wpTarget   = boat.wpSelected;
                boat.navigating = true;
                navigationReset();
                logEvent("GO WP%d (%.0fm)", boat.wpSelected, dist);
            } else {
                logEvent("WP%d too far (%.0fm), ignored", boat.wpSelected, dist);
            }
        }
        prevGoWp = goWp;

        // Расстояние до выбранной точки ch5 (даже если не едем туда)
        // обновляем в state чтобы показать в веб
        if (boat.waypoints[boat.wpSelected].valid) {
            boat.distToSelected = geoDistance(
                boat.latitude, boat.longitude,
                boat.waypoints[boat.wpSelected].lat,
                boat.waypoints[boat.wpSelected].lon);
        } else {
            boat.distToSelected = 0;
        }

        if (boat.navigating) {
            // Каскад: внутренний контур держит курс на каждом loop (как круиз),
            // внешний сам ограничен navInterval внутри navigationStep
            MotorOut m = navigationStep(boat.speedLimitPwm);
            motorsWrite(m.left, m.right);
        } else {
            motorsStop();
        }
    }
}

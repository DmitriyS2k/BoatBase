#pragma once
#include <Arduino.h>

enum BoatMode {
    MODE_MANUAL    = 0,
    MODE_SAVE_WP   = 1,
    MODE_AUTO      = 2,
};

struct Waypoint {
    double lat   = 0.0;
    double lon   = 0.0;
    bool   valid = false;
};

struct BoatState {
    // GPS
    bool     gpsActive   = false;
    bool     gpsFix      = false;
    int      satellites  = 0;
    float    hdop        = 99.0f;
    double   latitude    = 0.0;
    double   longitude   = 0.0;
    float    speed       = 0.0f;
    float    gpsHeading  = 0.0f;

    // Компас
    float    heading     = 0.0f;
    bool     calibrating = false;

    // Приёмник
    uint16_t ch[10]      = {1500,1500,1000,1500,1500,1500,1500,1500,1500,1500};
    bool     rcConnected = false;
    uint32_t lastRcMs    = 0;

    // Режим
    BoatMode mode        = MODE_MANUAL;
    int      wpSelected  = 0;
    Waypoint waypoints[4];

    // Навигация
    float    targetHeading  = 0.0f;
    float    distToTarget   = 0.0f;   // до wpTarget (текущая цель)
    float    distToSelected = 0.0f;   // до wpSelected (выбрана ch5)
    bool     navigating     = false;
    int      wpTarget       = 0;

    // Лимит скорости AUTO по крутилке ch6
    int      speedLimitPwm = 1900;

    // Веб-джойстик
    bool     webJoyActive  = false;
    uint32_t webJoyLastMs  = 0;
    int      webJoyLeft    = 1500;
    int      webJoyRight   = 1500;

    // Моторы
    int      motorLeft   = 1500;
    int      motorRight  = 1500;

    float    battery     = 0.0f;
    float    motorTemp   = -127.0f;  // температура мотора °C, -127 = датчик не найден

};

extern BoatState boat;

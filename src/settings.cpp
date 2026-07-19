#include "settings.h"
#include "state.h"
#include <Preferences.h>

BoatSettings cfg;
static Preferences prefs;

void settingsLoad() {
    prefs.begin("boat", true);
    cfg.pidKp          = prefs.getFloat("pidKp",    3.0f);
    cfg.pidKi          = prefs.getFloat("pidKi",    1.5f);
    cfg.pidKd          = prefs.getFloat("pidKd",    0.5f);
    cfg.pidCurve       = prefs.getFloat("pidCurve", 1.0f);
    cfg.arrivalRadius  = prefs.getFloat("aRadius",  3.0f);
    cfg.cruiseSpeed    = prefs.getInt  ("cSpeed",   1650);
    cfg.minSatellites  = prefs.getInt  ("minSat",   5);
    cfg.slowdownDist   = prefs.getFloat("slowDist", 5.0f);
    cfg.slowdownSpeed  = prefs.getInt  ("slowSpd",  1550);
    cfg.trimLeft       = prefs.getInt  ("trimL",    0);
    cfg.trimRight      = prefs.getInt  ("trimR",    0);
    cfg.cruiseGain     = prefs.getFloat("cruGain",  0.8f);
    cfg.maxDiff        = prefs.getInt("maxDiff",    150);
    cfg.bearingAlpha   = prefs.getFloat("brAlpha",  0.15f);
    cfg.navInterval    = prefs.getInt("navInt",     200);
    cfg.turnSlowFloor  = prefs.getFloat("turnSlowFl", 0.4f);
    cfg.motorMaxPwm    = prefs.getInt("motorMaxPwm", 1800);
    cfg.navMode        = prefs.getInt("navMode",     0);
    cfg.losLookahead   = prefs.getFloat("losLook",   10.0f);
    cfg.gpsRateHz      = prefs.getInt("gpsRateHz",   2);
    cfg.compassDecl    = prefs.getFloat("cDecl",    0.0f);
    cfg.compassXOffset = prefs.getFloat("cXOff",    0.0f);
    cfg.compassYOffset = prefs.getFloat("cYOff",    0.0f);
    cfg.compassAxis    = prefs.getInt  ("cAxis",    1);
    cfg.compassDeadzone= prefs.getFloat("cDead",    2.0f);

    // Точки маршрута
    for (int i = 0; i < 4; i++) {
        char keyV[8], keyLat[10], keyLon[10];
        snprintf(keyV,   sizeof(keyV),   "wpV%d",   i);
        snprintf(keyLat, sizeof(keyLat), "wpLat%d", i);
        snprintf(keyLon, sizeof(keyLon), "wpLon%d", i);
        boat.waypoints[i].valid = prefs.getBool  (keyV,   false);
        boat.waypoints[i].lat   = prefs.getDouble(keyLat, 0.0);
        boat.waypoints[i].lon   = prefs.getDouble(keyLon, 0.0);
    }
    prefs.end();
}

void settingsSave() {
    prefs.begin("boat", false);
    prefs.putFloat("pidKp",   cfg.pidKp);
    prefs.putFloat("pidKi",   cfg.pidKi);
    prefs.putFloat("pidKd",   cfg.pidKd);
    prefs.putFloat("pidCurve",cfg.pidCurve);
    prefs.putFloat("aRadius", cfg.arrivalRadius);
    prefs.putInt  ("cSpeed",  cfg.cruiseSpeed);
    prefs.putInt  ("minSat",  cfg.minSatellites);
    prefs.putFloat("slowDist",cfg.slowdownDist);
    prefs.putInt  ("slowSpd", cfg.slowdownSpeed);
    prefs.putInt  ("trimL",   cfg.trimLeft);
    prefs.putInt  ("trimR",   cfg.trimRight);
    prefs.putFloat("cruGain", cfg.cruiseGain);
    prefs.putInt("maxDiff",   cfg.maxDiff);
    prefs.putFloat("brAlpha", cfg.bearingAlpha);
    prefs.putInt("navInt",    cfg.navInterval);
    prefs.putFloat("turnSlowFl", cfg.turnSlowFloor);
    prefs.putInt("motorMaxPwm", cfg.motorMaxPwm);
    prefs.putInt("navMode",     cfg.navMode);
    prefs.putFloat("losLook",   cfg.losLookahead);
    prefs.putInt("gpsRateHz",   cfg.gpsRateHz);
    prefs.putFloat("cDecl",   cfg.compassDecl);
    prefs.putFloat("cXOff",   cfg.compassXOffset);
    prefs.putFloat("cYOff",   cfg.compassYOffset);
    prefs.putInt  ("cAxis",   cfg.compassAxis);
    prefs.putFloat("cDead",   cfg.compassDeadzone);
    prefs.end();
}

// Вызывается из main.cpp при сохранении точки
void waypointSave(int idx) {
    if (idx < 0 || idx > 3) return;
    prefs.begin("boat", false);
    char keyV[8], keyLat[10], keyLon[10];
    snprintf(keyV,   sizeof(keyV),   "wpV%d",   idx);
    snprintf(keyLat, sizeof(keyLat), "wpLat%d", idx);
    snprintf(keyLon, sizeof(keyLon), "wpLon%d", idx);
    prefs.putBool  (keyV,   boat.waypoints[idx].valid);
    prefs.putDouble(keyLat, boat.waypoints[idx].lat);
    prefs.putDouble(keyLon, boat.waypoints[idx].lon);
    prefs.end();
}

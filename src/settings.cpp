#include "settings.h"
#include "state.h"
#include <Preferences.h>

BoatSettings cfg;
static Preferences prefs;

void settingsLoad() {
    prefs.begin("boat", true);
    cfg.pidKp          = prefs.getFloat("pidKp",    7.9f);
    cfg.pidKi          = prefs.getFloat("pidKi",    7.6f);
    cfg.pidKd          = prefs.getFloat("pidKd",    0.2f);
    cfg.arrivalRadius  = prefs.getFloat("aRadius",  3.0f);
    cfg.cruiseSpeed    = prefs.getInt  ("cSpeed",   1650);
    cfg.minSatellites  = prefs.getInt  ("minSat",   5);
    cfg.slowdownDist   = prefs.getFloat("slowDist", 5.0f);
    cfg.slowdownSpeed  = prefs.getInt  ("slowSpd",  1550);
    cfg.trimLeft       = prefs.getInt  ("trimL",    0);
    cfg.trimRight      = prefs.getInt  ("trimR",    0);
    cfg.cruiseGain     = prefs.getFloat("cruGain",  2.0f);
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
    prefs.putFloat("aRadius", cfg.arrivalRadius);
    prefs.putInt  ("cSpeed",  cfg.cruiseSpeed);
    prefs.putInt  ("minSat",  cfg.minSatellites);
    prefs.putFloat("slowDist",cfg.slowdownDist);
    prefs.putInt  ("slowSpd", cfg.slowdownSpeed);
    prefs.putInt  ("trimL",   cfg.trimLeft);
    prefs.putInt  ("trimR",   cfg.trimRight);
    prefs.putFloat("cruGain", cfg.cruiseGain);
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

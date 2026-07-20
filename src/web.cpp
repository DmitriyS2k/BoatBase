#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Update.h>
#include "state.h"
#include "settings.h"
#include "compass.h"
#include "motors.h"
#include "eventlog.h"
#include "navlog.h"
#include "cruiselog.h"

AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

// ── WebSocket push ──────────────────────────────────────────────
void wsPush() {
    if (!ws.count()) return;

    JsonDocument doc;
    doc["sat"]       = boat.satellites;
    doc["hdop"]      = serialized(String(boat.hdop, 1));
    doc["fix"]       = boat.gpsFix;
    doc["gpsActive"] = boat.gpsActive;
    doc["heading"]   = (int)boat.heading;
    doc["speed"]     = serialized(String(boat.speed, 1));
    doc["battery"]   = serialized(String(boat.battery, 2));
    doc["mode"]      = (int)boat.mode;
    doc["rc"]        = boat.rcConnected;
    doc["wpSel"]     = boat.wpSelected;
    doc["wpTarget"]  = boat.wpTarget;
    doc["nav"]       = boat.navigating;
    doc["dist"]      = (int)boat.distToTarget;
    doc["distSel"]   = (int)boat.distToSelected;
    doc["targetHdg"] = (int)boat.targetHeading;
    doc["lat"]       = serialized(String(boat.latitude,  6));
    doc["lon"]       = serialized(String(boat.longitude, 6));
    doc["ml"]        = boat.motorLeft;
    doc["mr"]        = boat.motorRight;
    doc["speedLim"]  = boat.speedLimitPwm;
    doc["calib"]     = boat.calibrating;
    doc["calibPct"]  = compassCalibProgress();
    doc["rawX"]      = compassRawX;
    doc["rawY"]      = compassRawY;
    doc["axisMode"]  = cfg.compassAxis;
    doc["motorTemp"]  = serialized(String(boat.motorTemp, 1));

    JsonArray chs = doc["chs"].to<JsonArray>();
    for (int i = 0; i < 10; i++) chs.add(boat.ch[i]);

    JsonArray wpa = doc["wp"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
        JsonObject w = wpa.add<JsonObject>();
        w["v"]   = boat.waypoints[i].valid;
        w["lat"] = serialized(String(boat.waypoints[i].lat, 6));
        w["lon"] = serialized(String(boat.waypoints[i].lon, 6));
    }

    String json;
    serializeJson(doc, json);
    ws.textAll(json);
}

static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType t,
                      void*, uint8_t*, size_t) { (void)t; }

// ── GET /api/settings ───────────────────────────────────────────
static void handleGetSettings(AsyncWebServerRequest *req) {
    JsonDocument doc;
    doc["pidKp"]           = cfg.pidKp;
    doc["pidKi"]           = cfg.pidKi;
    doc["pidKd"]           = cfg.pidKd;
    doc["pidCurve"]        = cfg.pidCurve;
    doc["arrivalRadius"]   = cfg.arrivalRadius;
    doc["cruiseSpeed"]     = cfg.cruiseSpeed;
    doc["minSatellites"]   = cfg.minSatellites;
    doc["tempAlarm"]       = cfg.tempAlarm;
    doc["tempLimitPwm"]    = cfg.tempLimitPwm;
    doc["slowdownDist"]    = cfg.slowdownDist;
    doc["slowdownSpeed"]   = cfg.slowdownSpeed;
    doc["trimLeft"]        = cfg.trimLeft;
    doc["trimRight"]       = cfg.trimRight;
    doc["cruiseGain"]      = cfg.cruiseGain;
    doc["cruiseSteerRate"] = cfg.cruiseSteerRate;
    doc["maxDiff"]         = cfg.maxDiff;
    doc["bearingAlpha"]    = cfg.bearingAlpha;
    doc["navInterval"]     = cfg.navInterval;
    doc["turnSlowFloor"]   = cfg.turnSlowFloor;
    doc["motorMaxPwm"]     = cfg.motorMaxPwm;
    doc["navMode"]         = cfg.navMode;
    doc["losLookahead"]    = cfg.losLookahead;
    doc["gpsRateHz"]       = cfg.gpsRateHz;
    doc["compassDecl"]     = cfg.compassDecl;
    doc["compassAxis"]     = cfg.compassAxis;
    doc["compassDeadzone"] = cfg.compassDeadzone;
    String json;
    serializeJson(doc, json);
    req->send(200, "application/json", json);
}

// ── POST /api/settings ──────────────────────────────────────────
static void handlePostSettings(AsyncWebServerRequest *req, uint8_t *data,
                                size_t len, size_t, size_t) {
    JsonDocument doc;
    if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
        req->send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
    }
    if (doc["pidKp"].is<float>())           cfg.pidKp           = doc["pidKp"];
    if (doc["pidKi"].is<float>())           cfg.pidKi           = doc["pidKi"];
    if (doc["pidKd"].is<float>())           cfg.pidKd           = doc["pidKd"];
    if (doc["pidCurve"].is<float>())        cfg.pidCurve        = doc["pidCurve"];
    if (doc["arrivalRadius"].is<float>())   cfg.arrivalRadius   = doc["arrivalRadius"];
    if (doc["cruiseSpeed"].is<int>())       cfg.cruiseSpeed     = doc["cruiseSpeed"];
    if (doc["minSatellites"].is<int>())     cfg.minSatellites   = doc["minSatellites"];
    if (doc["tempAlarm"].is<float>())       cfg.tempAlarm       = doc["tempAlarm"];
    if (doc["tempLimitPwm"].is<int>())      cfg.tempLimitPwm    = doc["tempLimitPwm"];
    if (doc["slowdownDist"].is<float>())    cfg.slowdownDist    = doc["slowdownDist"];
    if (doc["slowdownSpeed"].is<int>())     cfg.slowdownSpeed   = doc["slowdownSpeed"];
    if (doc["trimLeft"].is<int>())          cfg.trimLeft        = doc["trimLeft"];
    if (doc["trimRight"].is<int>())         cfg.trimRight       = doc["trimRight"];
    if (doc["cruiseGain"].is<float>())      cfg.cruiseGain      = doc["cruiseGain"];
    if (doc["cruiseSteerRate"].is<float>()) cfg.cruiseSteerRate = doc["cruiseSteerRate"];
    if (doc["maxDiff"].is<int>())           cfg.maxDiff         = doc["maxDiff"];
    if (doc["bearingAlpha"].is<float>())    cfg.bearingAlpha    = doc["bearingAlpha"];
    if (doc["navInterval"].is<int>())       cfg.navInterval     = doc["navInterval"];
    if (doc["turnSlowFloor"].is<float>())   cfg.turnSlowFloor   = doc["turnSlowFloor"];
    if (doc["motorMaxPwm"].is<int>())       cfg.motorMaxPwm     = doc["motorMaxPwm"];
    if (doc["navMode"].is<int>())           cfg.navMode         = doc["navMode"];
    if (doc["losLookahead"].is<float>())    cfg.losLookahead    = doc["losLookahead"];
    if (doc["compassDecl"].is<float>())     cfg.compassDecl     = doc["compassDecl"];
    if (doc["compassAxis"].is<int>())       cfg.compassAxis     = doc["compassAxis"];
    if (doc["compassDeadzone"].is<float>()) cfg.compassDeadzone = doc["compassDeadzone"];
    settingsSave();
    req->send(200, "application/json", "{\"ok\":true}");
}

// ── POST /api/calibrate ─────────────────────────────────────────
static void handleCalibrate(AsyncWebServerRequest *req) {
    compassCalibStart();
    req->send(200, "application/json", "{\"ok\":true,\"duration\":15}");
}

// ── GET /api/log ────────────────────────────────────────────────
static void handleGetLog(AsyncWebServerRequest *req) {
    String json;
    logGetJson(json);
    req->send(200, "application/json", json);
}

// ── POST /api/joystick — ручное управление из веб ───────────────
// Приоритет у пульта: если RC подключён и стики отклонены — игнорируем веб
// "Стики в нейтрали" = ch1 и ch2 оба в диапазоне 1400..1600
static bool rcStickNeutral() {
    int ch1 = boat.ch[0];
    int ch2 = boat.ch[1];
    return (ch1 > 1400 && ch1 < 1600) && (ch2 > 1400 && ch2 < 1600);
}

static void handleJoystick(AsyncWebServerRequest *req, uint8_t *data,
                            size_t len, size_t, size_t) {
    // Пульт подключён и стики отклонены — пульт имеет приоритет
    if (boat.rcConnected && !rcStickNeutral()) {
        req->send(200, "application/json", "{\"ok\":false,\"reason\":\"rc_active\"}");
        return;
    }
    JsonDocument doc;
    if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
        req->send(400, "application/json", "{\"error\":\"bad json\"}");
        return;
    }
    int l = constrain((int)(doc["l"] | 1500), 1000, 2000);
    int r = constrain((int)(doc["r"] | 1500), 1000, 2000);

    // Сохраняем в state — main.cpp применит в нужный момент цикла
    boat.webJoyLeft   = l;
    boat.webJoyRight  = r;
    boat.webJoyLastMs = millis();
    // Считаем активным если не стоп
    boat.webJoyActive = (l != 1500 || r != 1500);

    req->send(200, "application/json", "{\"ok\":true}");
}

void webInit() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("BoatAutopilot", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    LittleFS.begin(true);

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/api/settings",  HTTP_GET,  handleGetSettings);
    server.on("/api/settings",  HTTP_POST,
        [](AsyncWebServerRequest *req){}, nullptr, handlePostSettings);
    server.on("/api/calibrate", HTTP_POST, handleCalibrate);
    server.on("/api/gpsreset", HTTP_POST, [](AsyncWebServerRequest *req) {
        extern void gpsReset();
        gpsReset();
        req->send(200, "text/plain", "OK");
    });
    // Частота GPS — применяется сразу к модулю, без перезагрузки корабля
    server.on("/api/gpsrate", HTTP_POST,
        [](AsyncWebServerRequest *req){}, nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
                req->send(400, "application/json", "{\"error\":\"bad json\"}");
                return;
            }
            int hz = doc["hz"] | 2;
            cfg.gpsRateHz = hz;
            settingsSave();
            extern void gpsSetRate(int hz);
            gpsSetRate(hz);
            req->send(200, "application/json", "{\"ok\":true}");
        });
    server.on("/api/log",       HTTP_GET,  handleGetLog);

    // ── Лог навигации AUTO (для анализа/отладки на компьютере) ──
    server.on("/api/navlog/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["count"]    = navLogCount();
        doc["capacity"] = navLogCapacity();
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });
    server.on("/api/navlog/clear", HTTP_POST, [](AsyncWebServerRequest *req) {
        navLogClear();
        req->send(200, "text/plain", "OK");
    });
    server.on("/api/navlog/capacity", HTTP_POST,
        [](AsyncWebServerRequest *req){}, nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
                req->send(400, "application/json", "{\"error\":\"bad json\"}");
                return;
            }
            int cap = doc["capacity"] | 1000;
            navLogSetCapacity(cap);
            req->send(200, "application/json", "{\"ok\":true}");
        });
    server.on("/api/navlog.csv", HTTP_GET, [](AsyncWebServerRequest *req) {
        navLogFillReset();
        AsyncWebServerResponse *resp = req->beginChunkedResponse("text/csv", navLogFillCsv);
        resp->addHeader("Content-Disposition", "attachment; filename=navlog.csv");
        req->send(resp);
    });

    // ── Лог круиз-режима (отдельно от navlog) ────────────────────
    server.on("/api/cruiselog/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["count"]    = cruiseLogCount();
        doc["capacity"] = cruiseLogCapacity();
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });
    server.on("/api/cruiselog/clear", HTTP_POST, [](AsyncWebServerRequest *req) {
        cruiseLogClear();
        req->send(200, "text/plain", "OK");
    });
    server.on("/api/cruiselog/capacity", HTTP_POST,
        [](AsyncWebServerRequest *req){}, nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
            JsonDocument doc;
            if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
                req->send(400, "application/json", "{\"error\":\"bad json\"}");
                return;
            }
            int cap = doc["capacity"] | 1000;
            cruiseLogSetCapacity(cap);
            req->send(200, "application/json", "{\"ok\":true}");
        });
    server.on("/api/cruiselog.csv", HTTP_GET, [](AsyncWebServerRequest *req) {
        cruiseLogFillReset();
        AsyncWebServerResponse *resp = req->beginChunkedResponse("text/csv", cruiseLogFillCsv);
        resp->addHeader("Content-Disposition", "attachment; filename=cruiselog.csv");
        req->send(resp);
    });

    server.on("/api/joystick",  HTTP_POST,
        [](AsyncWebServerRequest *req){}, nullptr, handleJoystick);

    // ── OTA прошивка через веб ──────────────────────────────────
    server.on("/update", HTTP_POST,
        [](AsyncWebServerRequest *req) {
            bool ok = !Update.hasError();
            AsyncWebServerResponse *resp = req->beginResponse(
                200, "text/plain", ok ? "OK" : "FAIL");
            resp->addHeader("Connection", "close");
            req->send(resp);
            if (ok) {
                delay(500);
                ESP.restart();
            }
        },
        [](AsyncWebServerRequest *req, String filename,
           size_t index, uint8_t *data, size_t len, bool final) {
            if (!index) {
                Serial.printf("[OTA] Start: %s\n", filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                    Update.printError(Serial);
                }
            }
            if (Update.write(data, len) != len) {
                Update.printError(Serial);
            }
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] Done: %u bytes\n", index + len);
                } else {
                    Update.printError(Serial);
                }
            }
        }
    );

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.begin();
}

static bool wifiEnabled = true;  // WiFi включён по умолчанию

void setWifiEnabled(bool enable) {
    if (enable == wifiEnabled) return;
    wifiEnabled = enable;
    if (enable) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BoatAutopilot", "12345678");
        Serial.println("[WiFi] AP started");
    } else {
        ws.closeAll();
        ws.cleanupClients();
        WiFi.softAPdisconnect(true);
        Serial.println("[WiFi] AP stopped");
    }
}

bool getWifiEnabled() { return wifiEnabled; }

void webUpdate() {
    static uint32_t t = 0;
    if (millis() - t >= 200) {
        t = millis();
        wsPush();
        ws.cleanupClients();
    }
}

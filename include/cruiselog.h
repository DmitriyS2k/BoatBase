#pragma once
#include <Arduino.h>

// ── Лог круиз-режима — кольцевой буфер в RAM, отдельно от navlog ──
// Пишется на каждом шаге cruiseStep(). Другие поля, чем у navlog:
// нет точки назначения/дистанции, зато есть целевой курс круиза (lockedHeading —
// теперь это cruiseTargetHeading, крутится от сн4, а не фиксируется единожды) и СН4.

void cruiseLogRecord(double lat, double lon, float heading, float lockedHeading, float err,
                      int ch4, int manualSteer, float autoSteer, float steerClamped,
                      int motorLeft, int motorRight, int thrust);

void cruiseLogClear();
int  cruiseLogCount();
int  cruiseLogCapacity();
void cruiseLogSetCapacity(int newCapacity);  // 100..2000, пересоздаёт буфер (лог очищается)

void   cruiseLogFillReset();
size_t cruiseLogFillCsv(uint8_t *buffer, size_t maxLen, size_t index);

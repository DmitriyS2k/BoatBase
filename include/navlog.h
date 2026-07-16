#pragma once
#include <Arduino.h>

// ── Лог навигации AUTO — кольцевой буфер в RAM для анализа/отладки ──
// Пишется на каждом шаге navigationStep() пока корабль реально едет.
// Скачивается как CSV через веб (/api/navlog.csv), старые записи
// перезаписываются новыми при переполнении.

void navLogRecord(double lat, double lon, float heading, float targetHeading, float err,
                   float pidRaw, float pidCurved, float pidFinal,
                   int motorLeft, int motorRight, float dist, float speedKmh,
                   int sats, float hdop, int wpTarget);

void navLogClear();
int  navLogCount();
int  navLogCapacity();
void navLogSetCapacity(int newCapacity);  // 100..2000, пересоздаёт буфер (лог очищается)

// Потоковая генерация CSV (для AsyncWebServer beginChunkedResponse).
// navLogFillReset() вызывать один раз в начале каждого скачивания.
void   navLogFillReset();
size_t navLogFillCsv(uint8_t *buffer, size_t maxLen, size_t index);

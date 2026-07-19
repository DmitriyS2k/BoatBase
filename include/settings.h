#pragma once
#include <Arduino.h>

struct BoatSettings {
    // Навигация (PID автопилот — взято из Katerok)
    float pidKp         = 3.0f;   // P: исправляет ошибку сейчас
    float pidKi         = 1.5f;   // I: накопленная ошибка
    float pidKd         = 0.5f;   // D: производная (демпфирование)
    float pidCurve      = 1.0f;   // нелинейность выхода PID: 1.0=линейно, >1=мягче у центра/жёстче у краёв
    float arrivalRadius = 3.0f;   // метры — считать «прибыл»
    int   cruiseSpeed   = 1650;   // PWM тяга в авто-режиме (1500..1900)
    int   minSatellites = 5;      // минимум спутников
    float tempAlarm     = 60.0f;   // температура перегрева мотора °C
    float slowdownDist  = 5.0f;   // метры — начало замедления перед точкой
    int   slowdownSpeed = 1550;   // PWM минимальная скорость в зоне замедления

    // Trim моторов (компенсация механического разброса ESC/винтов)
    // Добавляется к каждому мотору перед финальной записью, диапазон -200..+200
    int   trimLeft      = 0;
    int   trimRight     = 0;

    // Круиз (ручной с удержанием курса)
    float cruiseGain    = 0.8f;
    int   maxDiff       = 150;    // макс разница PWM между моторами при коррекции (1..400)
    float bearingAlpha  = 0.15f;  // сглаживание направления на точку (0.05=плавно..1.0=без фильтра)
    int   navInterval   = 200;    // период коррекции курса в AUTO (мс): меньше=чаще=рывки, больше=плавнее
    float turnSlowFloor = 0.4f;   // мин. доля тяги при резком повороте (0.2..1.0, 1.0=выкл замедление на повороте)
    int   motorMaxPwm   = 1800;   // реальный потолок КПД моторов (PWM выше почти не даёт тяги, проверено на воде)
    int   navMode       = 0;      // наведение в AUTO: 0=прямо в точку (старое), 1=line-of-sight (по прямой старт→цель)
    float losLookahead  = 10.0f;  // м — насколько вперёд по линии целиться в LOS-режиме (меньше=резче сходится к линии)

    // Компас
    float compassDecl      = 0.0f;  // магнитное склонение
    float compassXOffset   = 0.0f;  // hard-iron X
    float compassYOffset   = 0.0f;  // hard-iron Y
    int   compassAxis      = 1;     // формула осей 0..7
    float compassDeadzone  = 2.0f;  // мёртвая зона ±° (игнор шума, 2-5°)
};

extern BoatSettings cfg;

void settingsLoad();
void settingsSave();
void waypointSave(int idx);  // сохранить одну точку в NVS

#pragma once
#include <Arduino.h>

struct BoatSettings {
    // Навигация (PID автопилот — взято из Katerok)
    float pidKp         = 7.9f;   // P: исправляет ошибку сейчас
    float pidKi         = 7.6f;   // I: накопленная ошибка
    float pidKd         = 0.2f;   // D: производная (демпфирование)
    float arrivalRadius = 3.0f;   // метры — считать «прибыл»
    int   cruiseSpeed   = 1650;   // PWM тяга в авто-режиме (1500..1900)
    int   minSatellites = 5;      // минимум спутников
    float slowdownDist  = 5.0f;   // метры — начало замедления перед точкой
    int   slowdownSpeed = 1550;   // PWM минимальная скорость в зоне замедления

    // Trim моторов (компенсация механического разброса ESC/винтов)
    // Добавляется к каждому мотору перед финальной записью, диапазон -200..+200
    int   trimLeft      = 0;
    int   trimRight     = 0;

    // Круиз (ручной с удержанием курса)
    float cruiseGain    = 1.5f;

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

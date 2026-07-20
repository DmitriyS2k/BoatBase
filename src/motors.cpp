#include <Arduino.h>
#include "state.h"
#include "settings.h"

#define PIN_ESC_LEFT  25
#define PIN_ESC_RIGHT 26

#define CH_LEFT  0
#define CH_RIGHT 1

#define PWM_FREQ 50
#define PWM_RES  16

static uint32_t usToDuty(int us) {
    us = constrain(us, 1000, 2000);
    return (uint32_t)((uint64_t)us * 65535 / 20000);
}

void motorsInit() {
    ledcSetup(CH_LEFT,  PWM_FREQ, PWM_RES);
    ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_ESC_LEFT,  CH_LEFT);
    ledcAttachPin(PIN_ESC_RIGHT, CH_RIGHT);
    ledcWrite(CH_LEFT,  usToDuty(1500));
    ledcWrite(CH_RIGHT, usToDuty(1500));
}

void motorsWrite(int left, int right) {
    // Применяем trim: только когда мотор реально работает (> нейтрали),
    // иначе trim мешал бы тормозить/реверсить
    if (left  > 1510) left  += cfg.trimLeft;
    if (right > 1510) right += cfg.trimRight;

    // Термозащита: мотор перегрелся — режем газ сверху во всех режимах
    // (AUTO, круиз, ручное, веб-джойстик — все идут через эту функцию).
    // После trim, чтобы итоговый сигнал на ESC гарантированно не превышал потолок.
    if (boat.motorTemp > cfg.tempAlarm) {
        left  = min(left,  cfg.tempLimitPwm);
        right = min(right, cfg.tempLimitPwm);
    }

    boat.motorLeft  = constrain(left,  1000, 2000);
    boat.motorRight = constrain(right, 1000, 2000);
    ledcWrite(CH_LEFT,  usToDuty(boat.motorLeft));
    ledcWrite(CH_RIGHT, usToDuty(boat.motorRight));
}

void motorsStop() {
    motorsWrite(1500, 1500);
}

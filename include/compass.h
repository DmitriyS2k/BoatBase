#pragma once
#include <stdint.h>

void    compassInit();
void    compassUpdate();
void    compassCalibStart();
int     compassCalibProgress(); // 0..100 %

extern int16_t compassRawX;
extern int16_t compassRawY;
extern int16_t compassRawZ;

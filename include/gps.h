#pragma once

#define GPS_RX_PIN 16   // GPS TX → сюда
#define GPS_TX_PIN 17   // сюда → GPS RX (нужен для команд, напр. gpsReset())

void gpsInit();
void gpsUpdate();
void gpsReset();

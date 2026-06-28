#pragma once

struct MotorOut { int left, right; };

float geoDistance(double lat1, double lon1, double lat2, double lon2);
float geoBearing (double lat1, double lon1, double lat2, double lon2);

MotorOut navigationStep(int speedLimit);  // speedLimit: PWM 1510..1900
void     navigationReset();
MotorOut cruiseStep(int ch3, int ch4);
void     cruiseReset();

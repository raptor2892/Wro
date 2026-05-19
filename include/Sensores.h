#ifndef Sensores_H
#define Sensores_H

#include <Adafruit_BNO08x.h>

extern Adafruit_BNO08x bno08x;
extern float yaw_inicial;

bool iniciarBNO();
float leerYaw();

#endif // sensores_H
#ifndef sensores_H
#define sensores_H

#include <Adafruit_BNO08x.h>

#define lectura1 2
#define lectura2 3
#define lectura3 4
#define lectura4 5
#define lectura5 6
#define lectura6 7
#define lectura7 8
#define lectura8 9

#define ENC1_A 36
#define ENC1_B 39
#define ENC2_A 34
#define ENC2_B 35

extern bool linea_ancha;
extern Adafruit_BNO08x bno08x;
extern float yaw_inicial;

extern volatile long encoder1_count;
extern volatile long encoder2_count;

bool verificarLineaAncha();
void iniciarBNO();
float leerYaw();

void iniciarEncoders();
long leerEncoder1();
long leerEncoder2();

#endif // sensores_H
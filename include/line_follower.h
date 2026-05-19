#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>

void beginLineFollower();
void updateLineFollower();
void avanzarConLinea();
void detenerLinea();
void detectarMosaico();

// Procesador de cadenas complejas de velocidad
void parsePIDCommand(String input);

#endif // LINE_FOLLOWER_H
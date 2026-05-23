#include <Arduino.h>
#include "line_follower.h"
#include "motores.h"
#include "Sensores.h"

void setup() {
    // Configura pines, inicializa BNO, arranca Servo y frena motores por seguridad
    beginLineFollower(); // Avanza 3 segundos a full, luego lento hasta encontrar intersección y para
}

void loop() {
    leerYaw(); // Mantiene el buffer serial leyendo los comandos de error PID y marcas de intersección // Avanza 3 segundos a full, luego lento hasta encontrar intersección y para
    avanzar(200);
    girarGrados(90);
    avanzartiempo(3000);
    // Mantiene el buffer serial leyendo los comandos de error PID y marcas de intersección
    // Avanza 3 segundos a full, luego lento hasta encontrar intersección y para

}
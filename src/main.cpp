#include <Arduino.h>
#include "line_follower.h"
#include "motores.h"
#include "Sensores.h"

void setup() {
    // 1. Inicializa las comunicaciones, pines de motores, servos y el sensor IMU BNO08x
    beginLineFollower();
    
    // Pequeño delay de estabilización después de arrancar los sistemas
    delay(500);
    
    // 2. Envía la señal por el puerto serial hacia el script de Python para arrancar la cámara
    Serial.println(">>> Iniciando seguidor de linea...");
    avanzarConLinea(); 
}

void loop() {
    // Escucha el puerto serial de forma continua, procesa comandos PID o caracteres de paro 'S'
    updateLineFollower();
    
    // Mantenemos el loop libre de delays para no perder ningún frame enviado por Python
}
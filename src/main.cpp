#include <Arduino.h>
#include "line_follower.h"
#include "motores.h"
#include "Sensores.h"

void setup() {
    // 1. Inicializar todo el hardware (Configura pines de motores, Serial, I2C)
    beginLineFollower();
    
    Serial.println("\n=========================================");
    Serial.println("[MAIN] Iniciando Prueba Secuencial de Navegación");
    Serial.println("=========================================\n");
    
    // Espera de 3 segundos para que puedas colocar el robot en la pista con calma
    delay(3000);

    // ── PASO 1: Avanzar hasta pasar exactamente 2 cruces a la derecha ────────
    Serial.println("[MAIN] Fase 1/3: Avanzando con PID central. Buscando 2 marcas a la DERECHA...");
    
    // Esta función activará los motores a BASE_SPEED y mantendrá el PID vivo.
    // No pasará a la siguiente línea de código hasta que Python cuente 2 marcas derechas.
    avanzarConLineaHasta(0, 2); 
    
    // Al salir de la función anterior, los motores ya se detuvieron automáticamente.
    Serial.println("[MAIN] ¡Fase 1 Completada! Robot detenido en la segunda marca.");
    delay(500); // Pausa de medio segundo para estabilizar el chasis


    // ── PASO 2: Giro de 90 grados a la derecha usando el BNO08x ──────────────
    Serial.println("[MAIN] Fase 2/3: Ejecutando giro de 90° a la derecha...");
    
    // Limpiamos las marcas en memoria para que el residuo visual del cruce 
    // donde estamos parados no confunda al robot al arrancar la nueva línea.
    clearMarkers(); 
    
    // Arrancamos el giro (Llama a tu función nativa de motores)
    derecha(); 
    
    // [AQUÍ] Debes integrar la lectura real de tu giroscopio BNO08x.
    // Un ejemplo de cómo se vería tu control por grados de forma lineal sería:
    // float anguloInicial = obtenerAnguloBNO();
    // while(abs(obtenerAnguloBNO() - anguloInicial) < 90.0) {
    //     yield(); // Mantiene el procesador estable
    // }
    
    delay(450); // <-- RETRASO TEMPORAL DE PRUEBA (Sustitúyelo por tu lazo del BNO08x)
    
    detenerMotores();
    Serial.println("[MAIN] ¡Fase 2 Completada! Giro finalizado.");
    delay(500); // Pausa de estabilidad antes de volver a activar la cámara


    // ── PASO 3: Avanzar por la nueva calle hasta el siguiente cruce ──────────
    Serial.println("[MAIN] Fase 3/3: Retomando PID... Avanzando hasta topar con CUALQUIER cruce.");
    
    // Avanza usando el PID central hasta que detecte una intersección en T, en cruz,
    // o una escuadra (sin importar si la marca física está a la izquierda o derecha).
    avanzarConLineaHastaCruce(1);
    
    // ── FIN DE LA PRUEBA ─────────────────────────────────────────────────────
    detenerLinea(); // Le envía a Python el comando para apagar el modo de seguimiento
    Serial.println("[MAIN] =========================================");
    Serial.println("[MAIN] ¡Prueba de navegación completada con éxito!");
    Serial.println("[MAIN] =========================================");
}

void loop() {
    // Como tu navegación ahora es puramente lineal y procedimental, 
    // el loop se queda completamente vacío. La rutina corre una sola vez en el arranque.
    yield(); 
}
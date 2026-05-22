#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>

// ── Funciones de Inicialización y Ciclo Principal ───────────────────────────
void beginLineFollower();
void updateLineFollower();

// ── Parser de Comandos de Visión ──────────────────────────────────────────────
void parseErrorCommand(String input);
void parseMarkCommand(String input); // <- Nueva firma para procesar MARK:L y MARK:R

// ── API Pública: Comandos desde el ESP32 hacia Python ────────────────────────
void avanzarConLinea();
void detenerLinea();
void detectarMosaico();

// ── NUEVO: API Pública para consultar las Zonas Rojas desde tu Laberinto ────
/**
 * Devuelve true si la cámara de Python detectó una línea en el extremo izquierdo.
 */
bool checkLeftMarker();

/**
 * Devuelve true si la cámara de Python detectó una línea en el extremo derecho.
 */
bool checkRightMarker();

/**
 * Resetea las banderas de marcas detectadas. 
 * Lámala inmediatamente después de registrar la intersección en tu matriz 
 * o justo antes de iniciar un giro de 90°.
 */
void clearMarkers();

#endif // LINE_FOLLOWER_H

void avanzarConLineaHasta(int crucesIzq, int crucesDer);
void avanzarConLineaHastaCruce(int totalCruces);
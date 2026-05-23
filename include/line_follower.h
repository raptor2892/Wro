#pragma once
#include <Arduino.h>
#include <functional>

void beginLineFollower();
void updateLineFollower();

void avanzarConLinea();
void detenerLinea();
void detectarMosaico();

bool checkLeftMarker();
bool checkRightMarker();
void clearMarkers();

void avanzarConLineaHasta(int crucesIzq, int crucesDer);
void avanzarConLineaHastaCruce(int totalCruces);
void avanzarTiempo(unsigned long tiempoMs);

void avanza(int velocidad);
void derecha();
void izquierda();
void reversa(int velocidad);
void detenerMotores();
void girarGrados(float grados);
void ajustarMotores(int velocidadIzq, int velocidadDer);
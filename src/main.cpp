#include <Arduino.h>
#include "motores.h"
#include "Sensores.h"
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Configurar pines de motores como salida
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Configurar pines de sensores como entrada
    pinMode(lectura1, INPUT);
    pinMode(lectura2, INPUT);
    pinMode(lectura3, INPUT);
    pinMode(lectura4, INPUT);
    pinMode(lectura5, INPUT);
    pinMode(lectura6, INPUT);
    pinMode(lectura7, INPUT);
    pinMode(lectura8, INPUT);

    iniciarBNO();
    inicializarServo();
    iniciarEncoders();
    Serial.println("Sistema inicializado");
}

void loop() {
    // Imprimir valores de encoders
    Serial.print("Encoder1: ");
    Serial.print(leerEncoder1());
    Serial.print(" Encoder2: ");
    Serial.println(leerEncoder2());
    
    delay(100); // Delay para no saturar el serial
}


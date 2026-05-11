#include <Arduino.h>
#include <math.h>
#include "Sensores.h"
#include <Wire.h>

#define lectura1 2
#define lectura2 3
#define lectura3 4
#define lectura4 5
#define lectura5 6
#define lectura6 7
#define lectura7 8
#define lectura8 9

bool linea_ancha = false;
Adafruit_BNO08x bno08x(-1); // Reset pin only for current library version
float yaw_inicial = 0.0;

volatile long encoder1_count = 0;
volatile long encoder2_count = 0;

void IRAM_ATTR encoder1_ISR() {
    if (digitalRead(ENC1_B) == HIGH) {
        encoder1_count++;
    } else {
        encoder1_count--;
    }
}

void IRAM_ATTR encoder2_ISR() {
    if (digitalRead(ENC2_B) == HIGH) {
        encoder2_count++;
    } else {
        encoder2_count--;
    }
}

bool verificarLineaAncha() {
    if (digitalRead(lectura1) == HIGH && digitalRead(lectura2) == HIGH &&
        digitalRead(lectura3) == HIGH && digitalRead(lectura4) == HIGH &&
        digitalRead(lectura5) == HIGH && digitalRead(lectura6) == HIGH &&
        digitalRead(lectura7) == HIGH && digitalRead(lectura8) == HIGH) {
        linea_ancha = true;
        return true;
    } else {
        linea_ancha = false;
        return false;
    }
}

void iniciarBNO() {
    if (!bno08x.begin_I2C()) {
        Serial.println("Error al inicializar BNO08x");
        while (1);
    }
    Serial.println("BNO08x inicializado");

    // Configurar reportes
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Error al habilitar reporte de rotación");
    }

    // Capturar yaw inicial
    delay(100);
    yaw_inicial = leerYaw();
}

float leerYaw() {
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            // Convertir quaternion a yaw (en grados)
            float qr = sensorValue.un.gameRotationVector.real;
            float qi = sensorValue.un.gameRotationVector.i;
            float qj = sensorValue.un.gameRotationVector.j;
            float qk = sensorValue.un.gameRotationVector.k;

            float yaw = atan2(2.0 * (qj * qk + qi * qr), 1.0 - 2.0 * (qi * qi + qj * qj));
            return yaw * 180.0 / PI; // Convertir a grados
        }
    }
    return 0.0; // Error
}

void iniciarEncoders() {
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2_ISR, RISING);
}

long leerEncoder1() {
    return encoder1_count;
}

long leerEncoder2() {
    return encoder2_count;
}
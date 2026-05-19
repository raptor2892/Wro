#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Sensores.h"

bool linea_ancha = false;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
float yaw_inicial = 0.0;

uint8_t scanI2C() {
    Serial.println("Escaneando bus I2C...");
    uint8_t found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("  Dispositivo encontrado en 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found = addr;
            break;
        }
    }

    if (!found) {
        Serial.println("  Ningún dispositivo encontrado.");
    }

    return found;
}

void setReports() {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 50000)) {
        Serial.println("WARNING: no se pudo habilitar SH2_ROTATION_VECTOR");
    }
    if (!bno08x.enableReport(SH2_ACCELEROMETER, 50000)) {
        Serial.println("WARNING: no se pudo habilitar SH2_ACCELEROMETER");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000)) {
        Serial.println("WARNING: no se pudo habilitar SH2_GYROSCOPE_CALIBRATED");
    }
}

void quaternionToEuler(float qr, float qi, float qj, float qk,
                       float &roll, float &pitch, float &yaw) {
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    roll  = atan2(2.0 * (qr * qi + qj * qk), 1.0 - 2.0 * (sqi + sqj)) * 180.0 / PI;
    pitch = asin(2.0 * (qr * qj - qk * qi)) * 180.0 / PI;
    yaw   = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (sqj + sqk)) * 180.0 / PI;
}

bool iniciarBNO() {
    Serial.println("Iniciando BNO085...");
    Wire.end();
    delay(100);
    Wire.begin(21, 22);
    delay(200);
    Serial.println("Bus I2C iniciado en SDA=21, SCL=22");

    uint8_t addr = scanI2C();
    if (!addr) {
        Serial.println("ERROR: No se encontró ningún dispositivo I2C.");
        return false;
    }

    Serial.print("Iniciando BNO085 en 0x");
    Serial.println(addr, HEX);

    if (!bno08x.begin_I2C(addr)) {
        Serial.println("ERROR: No se pudo iniciar el BNO085 en esa dirección.");
        return false;
    }

    Serial.println("BNO085 OK.");
    setReports();
    Serial.println("Reportes habilitados.");

    delay(100);
    yaw_inicial = leerYaw();
    return true;
}

float leerYaw() {
    unsigned long deadline = millis() + 50;
    while (millis() < deadline) {
        if (!bno08x.getSensorEvent(&sensorValue)) {
            continue;
        }

        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float roll, pitch, yaw;
            quaternionToEuler(
                sensorValue.un.rotationVector.real,
                sensorValue.un.rotationVector.i,
                sensorValue.un.rotationVector.j,
                sensorValue.un.rotationVector.k,
                roll, pitch, yaw);

            if (yaw < 0) yaw += 360.0;
            yaw_inicial = yaw;
            return yaw;
        }
    }

    return yaw_inicial;
}

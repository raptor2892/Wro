#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Sensores.h"

bool linea_ancha = false;

Adafruit_BNO08x   bno08x(-1);
sh2_SensorValue_t sensorValue;
float             yaw_inicial  = 0.0f;
float             ultimoYaw    = 0.0f;  // <-- último yaw válido sin bloquear

// ── Escaneo I2C ───────────────────────────────────────────────────────────────
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

    if (!found) Serial.println("  Ningun dispositivo encontrado.");
    return found;
}

// ── Habilitar reportes del BNO085 ─────────────────────────────────────────────
void setReports() {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {  // 10ms = 100Hz
        Serial.println("WARNING: no se pudo habilitar SH2_ROTATION_VECTOR");
    }
    if (!bno08x.enableReport(SH2_ACCELEROMETER, 50000)) {
        Serial.println("WARNING: no se pudo habilitar SH2_ACCELEROMETER");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000)) {
        Serial.println("WARNING: no se pudo habilitar SH2_GYROSCOPE_CALIBRATED");
    }
}

// ── Conversión quaternión → Euler ─────────────────────────────────────────────
void quaternionToEuler(float qr, float qi, float qj, float qk,
                       float &roll, float &pitch, float &yaw) {
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    roll  = atan2f(2.0f * (qr * qi + qj * qk), 1.0f - 2.0f * (sqi + sqj)) * 180.0f / PI;
    pitch = asinf (2.0f * (qr * qj - qk * qi))                             * 180.0f / PI;
    yaw   = atan2f(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (sqj + sqk)) * 180.0f / PI;
}

// ── Inicialización ────────────────────────────────────────────────────────────
bool iniciarBNO() {
    Serial.println("Iniciando BNO085...");
    Wire.end();
    delay(100);
    Wire.begin(21, 22);
    delay(200);
    Serial.println("Bus I2C iniciado en SDA=21, SCL=22");

    uint8_t addr = scanI2C();
    if (!addr) {
        Serial.println("ERROR: No se encontro ningun dispositivo I2C.");
        return false;
    }

    Serial.print("Iniciando BNO085 en 0x");
    Serial.println(addr, HEX);

    if (!bno08x.begin_I2C(addr)) {
        Serial.println("ERROR: No se pudo iniciar el BNO085 en esa direccion.");
        return false;
    }

    Serial.println("BNO085 OK.");
    setReports();
    Serial.println("Reportes habilitados.");

    delay(100);
    ultimoYaw  = leerYaw();
    yaw_inicial = ultimoYaw;
    return true;
}

// ── Lectura de Yaw NO BLOQUEANTE ──────────────────────────────────────────────
// Si hay dato nuevo lo procesa y actualiza ultimoYaw.
// Si no hay dato, regresa el último valor válido al instante.
float leerYaw() {
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float roll, pitch, yaw;
            quaternionToEuler(
                sensorValue.un.rotationVector.real,
                sensorValue.un.rotationVector.i,
                sensorValue.un.rotationVector.j,
                sensorValue.un.rotationVector.k,
                roll, pitch, yaw);

            if (yaw < 0.0f) yaw += 360.0f;
            ultimoYaw = yaw;
        }
    }
    return ultimoYaw;
}
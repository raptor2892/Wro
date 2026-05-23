#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Sensores.h"

bool linea_ancha = false;

Adafruit_BNO08x   bno08x(-1);
sh2_SensorValue_t sensorValue;
float             yaw_inicial = 0.0f;

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

    if (!found) {
        Serial.println("  Ningun dispositivo encontrado.");
    }

    return found;
}

// ── Habilitar reportes del BNO085 ─────────────────────────────────────────────
void setReports() {
    // 50 000 µs = 20 Hz — suficiente para corrección de heading
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
    yaw_inicial = leerYaw();
    return true;
}

// ── Lectura de Yaw ────────────────────────────────────────────────────────────
//
//  Timeout reducido a 15 ms (vs 50 ms original) para no bloquear el loop PID.
//  Si no llega un dato fresco en ese tiempo, devuelve el último yaw conocido
//  (yaw_inicial) para que el corrector no aplique una corrección espuria.
//
float leerYaw() {
    unsigned long deadline = millis() + 15;   // ← 15 ms máximo de espera

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

            // Normalizar a [0, 360)
            if (yaw < 0.0f) yaw += 360.0f;
            return yaw;
        }
    }

    // Sin dato fresco: devuelve el último yaw conocido
    return yaw_inicial;
}
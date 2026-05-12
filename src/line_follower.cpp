#include "line_follower.h"

#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>

#include "motores.h"
#include "Sensores.h"

namespace {
constexpr unsigned long SERIAL_BAUD_RATE = 9600;
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;

char lastCommand = 'S';
unsigned long lastCommandTime = 0;

void configurePins() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(lectura1, INPUT);
    pinMode(lectura2, INPUT);
    pinMode(lectura3, INPUT);
    pinMode(lectura4, INPUT);
    pinMode(lectura5, INPUT);
    pinMode(lectura6, INPUT);
    pinMode(lectura7, INPUT);
    pinMode(lectura8, INPUT);
}

void stopMotors() {
    detenerMotores();
}

void executeCommand(char command) {
    switch (command) {
        case 'A':
            avanzar();
            break;
        case 'D':
            girarDerecha();
            break;
        case 'I':
            girarIzquierda();
            break;
        case 'R':
            retroceder();
            break;
        case 'S':
            stopMotors();
            break;
        default:
            return;
    }

    lastCommand = command;
    lastCommandTime = millis();
    Serial.print("ACK:");
    Serial.println(command);
}
}

void beginLineFollower() {
    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();

    configurePins();

    iniciarBNO();
    inicializarServo();
    iniciarEncoders();
    stopMotors();

    Serial.println("System ready");
}

void updateLineFollower() {
    while (Serial.available() > 0) {
        char incoming = static_cast<char>(Serial.read());

        if (incoming == '\n' || incoming == '\r') {
            continue;
        }

        executeCommand(static_cast<char>(toupper(incoming)));
    }

    if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS && lastCommand != 'S') {
        stopMotors();
        lastCommand = 'S';
    }
}
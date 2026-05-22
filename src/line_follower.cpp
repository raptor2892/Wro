#include "line_follower.h"
<<<<<<< HEAD
=======

>>>>>>> a9d26d1bb75b58abf3071deb52fe26dd5f5cc306
#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>

#include "motores.h"
#include "Sensores.h"

<<<<<<< HEAD
// ── Configuración ─────────────────────────────────────────────────────────────
static constexpr unsigned long SERIAL_BAUD_RATE  = 115200;
static constexpr unsigned long TIMEOUT_SIMPLE_MS = 300;  // A/D/I/R
static constexpr unsigned long TIMEOUT_PID_MS    = 50;  // PID (depende del framerate)

// ── Estado interno ────────────────────────────────────────────────────────────
static char          lastCommand     = 'S';
static unsigned long lastCommandTime = 0;
static bool          pidMode         = false;

// ── Helpers privados ──────────────────────────────────────────────────────────
static void configurePins() {
=======
namespace {
constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;

char lastCommand = 'S';
unsigned long lastCommandTime = 0;

void configurePins() {
>>>>>>> a9d26d1bb75b58abf3071deb52fe26dd5f5cc306
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
<<<<<<< HEAD
}

static void stopMotors() {
    detenerMotores();
}

static void sendPiCommand(const char* command) {
    Serial.println(command);
}

// ── Comandos simples: A / D / I / R / S ──────────────────────────────────────
static void executeCommand(char command) {
    pidMode = false;  // cualquier comando manual cancela el modo PID

    switch (command) {
        case 'A': avanza(180);  break;
        case 'D': derecha();    break;
        case 'I': izquierda();  break;
        case 'R': reversa(150); break;
        case 'S': stopMotors(); break;
        default:  return;       // carácter desconocido → ignorar sin ACK
    }

    lastCommand     = command;
    lastCommandTime = millis();

    Serial.print("ACK:");
    Serial.println(command);
}

// ── Parser PID ────────────────────────────────────────────────────────────────
// Formato esperado: "PID:velIzq,velDer"   Ej: "PID:220,195"
// ── Parser PID ────────────────────────────────────────────────────────────────
void parsePIDCommand(String input) {
    String datos = input.substring(4);  // quita "PID:"
    int comaIndex = datos.indexOf(',');

    if (comaIndex == -1) return;  // formato inválido → ignorar

    int velIzquierda = datos.substring(0, comaIndex).toInt();
    int velDerecha   = datos.substring(comaIndex + 1).toInt();

    // Limitamos los valores entre -255 y 255 manualmente por seguridad
    if (velIzquierda > 255) velIzquierda = 255;
    if (velIzquierda < -255) velIzquierda = -255;
    if (velDerecha > 255) velDerecha = 255;
    if (velDerecha < -255) velDerecha = -255;

    // Enviamos las velocidades
    ajustarMotores(velDerecha, velIzquierda);

    lastCommand     = 'P';
    lastCommandTime = millis();
    pidMode         = true;
}


// ── API pública: comandos hacia Python ───────────────────────────────────────
void avanzarConLinea() {
    sendPiCommand("FOLLOW_LINE");
}

void detenerLinea() {
    sendPiCommand("STOP_LINE");
    stopMotors();
    lastCommand = 'S';
    pidMode     = false;
}

void detectarMosaico() {
    sendPiCommand("DETECT_MOSAIC");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void beginLineFollower() {
    Serial.begin(SERIAL_BAUD_RATE);
    configurePins();

    if (!iniciarBNO()) {
        Serial.println("ERROR: BNO08x no inicializado");
        return;
    }

    inicializarServo();
    stopMotors();
    Serial.println("System ready");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void updateLineFollower() {

    // Leer y despachar todos los comandos pendientes en el buffer serial
    while (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) continue;

        if (input.startsWith("PID:")) {
            parsePIDCommand(input);
        } else if (input.length() == 1) {
            executeCommand(static_cast<char>(toupper(input[0])));
        }
        // Cualquier otro formato se ignora silenciosamente
    }

    // Watchdog: si Python deja de responder, frena el robot
    unsigned long timeout = pidMode ? TIMEOUT_PID_MS : TIMEOUT_SIMPLE_MS;
    if (lastCommand != 'S' && (millis() - lastCommandTime > timeout)) {
        stopMotors();
        lastCommand = 'S';
        pidMode     = false;
        Serial.println("TIMEOUT: motores detenidos");
    }
}
=======

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
>>>>>>> a9d26d1bb75b58abf3071deb52fe26dd5f5cc306

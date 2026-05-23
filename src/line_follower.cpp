#include "line_follower.h"
#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>

#include "motores.h"
#include "Sensores.h"

// ── Configuración ─────────────────────────────────────────────────────────────
static constexpr unsigned long SERIAL_BAUD_RATE  = 115200;
static constexpr unsigned long TIMEOUT_SIMPLE_MS = 300;
static constexpr unsigned long TIMEOUT_PID_MS    = 150;

// ── Configuración corrector de heading (BNO085 - Timón Principal) ─────────────
static constexpr float HEADING_KP = 3.0f;
static constexpr float HEADING_KD = 1.5f;

static int           BASE_SPEED = 200;   // modificable en runtime
static constexpr int MAX_SPEED  = 220;
static constexpr int MIN_SPEED  = 120;
float yaw_recta = 0.0f; // Ahora este valor será dinámico y aprenderá de la cámara

// ── Estado interno ────────────────────────────────────────────────────────────
static char          lastCommand     = 'S';
static unsigned long lastCommandTime = 0;
static bool          pidMode         = false;

// PID / Historial de línea (Cámara)
static float         integralError = 0.0f;
static float         prevError     = 0.0f;
static unsigned long prevTimePid   = 0;

// Corrector de heading (BNO)
static float         prevHeadingError = 0.0f;

// ── Estado de Intersecciones ──────────────────────────────────────────────────
static bool          leftMarkerDetected  = false;
static bool          rightMarkerDetected = false;
static bool          markDetected        = false;
static unsigned long lastLeftMarkerTime  = 0;
static unsigned long lastRightMarkerTime = 0;



// ── Helpers privados ──────────────────────────────────────────────────────────
static void configurePins() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
}

static void stopMotors()                   { detenerMotores(); }
static void sendPiCommand(const char* cmd) { Serial.println(cmd); }

// ── Comandos simples: A / D / I / R / S ──────────────────────────────────────
static void executeCommand(char command) {
    if (pidMode) {
        integralError    = 0.0f;
        prevError        = 0.0f;
        prevHeadingError = 0.0f;
        pidMode          = false;
    }

    switch (command) {
        case 'A': avanza(180);  break;
        case 'D': derecha();    break;
        case 'I': izquierda();  break;
        case 'R': reversa(150); break;
        case 'S': stopMotors(); break;
        default:  return;
    }

    lastCommand     = command;
    lastCommandTime = millis();
    Serial.print("ACK:");
    Serial.println(command);
}

// ── OPCIÓN A: MANTENER BNO EN RECTAS Y CÁMARA COMO SALVAVIDAS SUAVE ───────────
// ── VARIABLES GLOBALES A COLOCAR AL INICIO ────────────────────────────────────

static float prev_yaw_bno = 0.0f; // Guardará el ángulo físico anterior del BNO

// ── CONTROL SINTONIZADO ANTI-OSCILACIÓN CREADO POR TEC ────────────────────────
bool en_linea_recta = false; // Indica si estamos en modo crucero recto (BNO bloqueado)
void parseErrorCommand(String input) {
    String datos     = input.substring(6);
    float  errorLinea = datos.toFloat();

    unsigned long now = millis();
    float dt = (now - prevTimePid) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;

    // Umbral de tolerancia: 15 píxeles
    if (abs(errorLinea) <= 25.0f) {
        
        // ── MUNDO 1: CRUCERO RECTO (PURO BNO085) ─────────────────────────────
        // Si acabamos de entrar al centro, congelamos el ángulo actual como NUEVA META
        if (!en_linea_recta) {
            yaw_recta = leerYaw();
            en_linea_recta = true;
            prevHeadingError = 0.0f;
        }

        float headingError = leerYaw() - yaw_recta;
        if (headingError >  180.0f) headingError -= 360.0f;
        if (headingError < -180.0f) headingError += 360.0f;

        float headingDerivative = (headingError - prevHeadingError) / dt;
        // Un PD muy simple para mantener el chasis congelado en esa línea
        float corrHeading = (4.0f * headingError) + (0.3f * headingDerivative);
        prevHeadingError = headingError;

        int velIzquierda = constrain((int)(BASE_SPEED + corrHeading), MIN_SPEED, MAX_SPEED);
        int velDerecha   = constrain((int)(BASE_SPEED - corrHeading), MIN_SPEED, MAX_SPEED);
        ajustarMotores(velDerecha, velIzquierda);

    } else {
        
        // ── MUNDO 2: RECUPERACIÓN DE EMERGENCIA (PURA CÁMARA) ─────────────────
        // Rompemos el bloqueo del BNO porque nos salimos de la zona segura
        en_linea_recta = false; 

        float derivative = (errorLinea - prevError) / dt;
        // PID puramente visual, ignoramos el BNO por completo para no confundir motores
        float corrLinea = (0.006f * errorLinea) + (0.04f * derivative);
        prevError = errorLinea;

        int velIzquierda = constrain((int)(BASE_SPEED + corrLinea), MIN_SPEED, MAX_SPEED);
        int velDerecha   = constrain((int)(BASE_SPEED - corrLinea), MIN_SPEED, MAX_SPEED);
        ajustarMotores(velDerecha, velIzquierda);
    }

    prevTimePid     = now;
    lastCommand     = 'P';
    lastCommandTime = millis();
    pidMode         = true;
}

// ── Parser para Marcas ────────────────────────────────────────────────────────
void parseMarkCommand(String input) {
    unsigned long now = millis();

    if (input.equals("MARK:L")) {
        leftMarkerDetected = true;
        markDetected       = true;
        lastLeftMarkerTime = now;
        Serial.println("ACK:MARK_L");
    }
    else if (input.equals("MARK:R")) {
        rightMarkerDetected = true;
        markDetected        = true;
        lastRightMarkerTime = now;
        Serial.println("ACK:MARK_R");
    }
    else if (input.equals("MARK")) {
        markDetected = true;
        Serial.println("ACK:MARK");
    }
}

// ── API pública ───────────────────────────────────────────────────────────────

void avanzarConLinea() {
    yaw_recta = leerYaw(); // Sincroniza rumbo antes de pedir el flujo visual
    sendPiCommand("FOLLOW_LINE");
}

void detenerLinea() {
    sendPiCommand("STOP_LINE");
    stopMotors();
    lastCommand      = 'S';
    pidMode          = false;
    prevHeadingError = 0.0f;
}

void detectarMosaico() { sendPiCommand("DETECT_MOSAIC"); }

bool checkLeftMarker()  { return leftMarkerDetected; }
bool checkRightMarker() { return rightMarkerDetected; }

void clearMarkers() {
    leftMarkerDetected  = false;
    rightMarkerDetected = false;
    markDetected        = false;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void beginLineFollower() {
    Serial.begin(SERIAL_BAUD_RATE);
    configurePins();

    if (!iniciarBNO()) {
        Serial.println("WARN: BNO085 no disponible, continuando sin IMU");
    }

    inicializarServo();
    stopMotors();
    Serial.println("System ready");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void updateLineFollower() {
    while (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) continue;

        if      (input.startsWith("ERROR:")) parseErrorCommand(input);
        else if (input.startsWith("MARK:"))  parseMarkCommand(input);
        else if (input.equals("MARK"))        parseMarkCommand(input);
        else if (input.length() == 1)         executeCommand(static_cast<char>(toupper(input[0])));
    }

    unsigned long timeout = pidMode ? TIMEOUT_PID_MS : TIMEOUT_SIMPLE_MS;
    if (lastCommand != 'S' && (millis() - lastCommandTime > timeout)) {
        stopMotors();
        lastCommand      = 'S';
        pidMode          = false;
        prevHeadingError = 0.0f;
        Serial.println("TIMEOUT: motores detenidos por falta de datos");
    }
}

// ── Navegación ────────────────────────────────────────────────────────────────
void avanzarConLineaHasta(int crucesIzq, int crucesDer) {
    int contadorIzq = 0;
    int contadorDer = 0;
    clearMarkers();
    avanzarConLinea();   

    while ((crucesIzq > 0 && contadorIzq < crucesIzq) ||
           (crucesDer > 0 && contadorDer < crucesDer)) {
        updateLineFollower();
        if (crucesIzq > 0 && checkLeftMarker()) {
            contadorIzq++;
            Serial.print("[NAV] Marca Izquierda #");
            Serial.println(contadorIzq);
            clearMarkers();
        }
        if (crucesDer > 0 && checkRightMarker()) {
            contadorDer++;
            Serial.print("[NAV] Marca Derecha #");
            Serial.println(contadorDer);
            clearMarkers();
        }
        yield();
    }

    stopMotors();
    Serial.println("[NAV] Objetivo de cruces alcanzado. Motores en STOP.");
}

void avanzarConLineaHastaCruce(int totalCruces) {
    int contadorCruces = 0;
    clearMarkers();
    avanzarConLinea();

    while (contadorCruces < totalCruces) {
        updateLineFollower();
        if (checkLeftMarker() || checkRightMarker()) {
            contadorCruces++;
            Serial.print("[NAV] Interseccion #");
            Serial.println(contadorCruces);
            clearMarkers();
        }
        yield();
    }

    stopMotors();
    Serial.println("[NAV] Cruce(s) detectado(s). Motores en STOP.");
}

void avanzarTiempo(unsigned long tiempoMs) {
    clearMarkers();
    markDetected = false;
    avanzarConLinea();

    // Fase 1: velocidad normal por el tiempo indicado
    unsigned long inicio = millis();
    while (millis() - inicio < tiempoMs) {
        updateLineFollower();
        yield();
    }

    // Fase 2: baja velocidad y espera 1 MARK
    BASE_SPEED = 100;
    Serial.println("[NAV] Buscando interseccion a baja velocidad...");

    while (!markDetected) {
        updateLineFollower();
        yield();
    }
    markDetected = false;
    Serial.println("[NAV] MARK detectado, frenando...");

    // Fase 3: avanza un poco mas lento y se detiene
    BASE_SPEED = 60;
    unsigned long pausa = millis();
    while (millis() - pausa < 200) {
        updateLineFollower();
        yield();
    }

    BASE_SPEED       = 240;
    markDetected     = false;
    prevHeadingError = 0.0f;
    clearMarkers();
    stopMotors();
    Serial.println("[NAV] Detenido sobre la interseccion.");
}
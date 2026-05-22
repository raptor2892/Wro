#include "line_follower.h"
#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>

#include "motores.h"
#include "Sensores.h"

// ── Configuración ─────────────────────────────────────────────────────────────
static constexpr unsigned long SERIAL_BAUD_RATE  = 115200;
static constexpr unsigned long TIMEOUT_SIMPLE_MS = 300;  // A/D/I/R
static constexpr unsigned long TIMEOUT_PID_MS    = 150;  // Tolerancia para recepción del video

// ── Configuración PID Optimizado para Rectas de Competencia ───────────────────
static constexpr float PID_KP     = 0.15;  // Súbelo un poco para que responda con más autoridad
static constexpr float PID_KI     = 0.00;  // Mantener en 0
static constexpr float PID_KD     = 0.05; // Amortiguación suave para el ruido del sensor

static constexpr int BASE_SPEED   = 240;
static constexpr int MAX_SPEED    = 255;
static constexpr int MIN_SPEED    = 210;   // Piso alto para no perder inercia lineal

// ── Estado interno ────────────────────────────────────────────────────────────
static char          lastCommand     = 'S';
static unsigned long lastCommandTime = 0;
static bool          pidMode         = false;

// Variables de estado para el cálculo PID
static float         integralError   = 0.0;
static float         prevError       = 0.0;
static unsigned long prevTimePid     = 0;

// ── Estado de Intersecciones de las Zonas Rojas ───────────────────────────────
static bool          leftMarkerDetected  = false;
static bool          rightMarkerDetected = false;
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

static void stopMotors() {
    detenerMotores();
}

static void sendPiCommand(const char* command) {
    Serial.println(command);
}

// ── Comandos simples: A / D / I / R / S ──────────────────────────────────────
static void executeCommand(char command) {
    if (pidMode) {
        integralError = 0.0;
        prevError = 0.0;
        pidMode = false;
    }

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

// ── Parser y Lógica PID ───────────────────────────────────────────────────────
void parseErrorCommand(String input) {
    String datos = input.substring(6);  // quita "ERROR:"
    float error = datos.toFloat();

    // ── FILTRO ZONA MUERTA (Deadband): Elimina el ruido en línea recta ──
    if (abs(error) <= 10.0) {
        error = 0.0;
    }

    unsigned long now = millis();
    
    if (!pidMode) {
        prevTimePid = now;
        integralError = 0.0;
        prevError = error;
    }

    float dt = (now - prevTimePid) / 1000.0; // Delta time en segundos
    if (dt <= 0.0) dt = 0.001;               // Evitar división por cero

    // Cálculo PID (Con Ki en 0.00, integralError se mantiene neutral)
    integralError += error * dt;
    
    // Anti-windup
    if (integralError > 500.0) integralError = 500.0;
    if (integralError < -500.0) integralError = -500.0;

    float derivative = (error - prevError) / dt;
    float correction = (PID_KP * error) + (PID_KI * integralError) + (PID_KD * derivative);

    prevError = error;
    prevTimePid = now;

    // Calcular las velocidades con la corrección
    int velIzquierda = BASE_SPEED + correction;
    int velDerecha   = BASE_SPEED - correction;

    // Constreñir usando los límites de competencia establecidos
    velIzquierda = constrain(velIzquierda, MIN_SPEED, MAX_SPEED);
    velDerecha   = constrain(velDerecha, MIN_SPEED, MAX_SPEED);

    // Enviamos las velocidades a los motores
    ajustarMotores(velIzquierda, velDerecha);

    lastCommand     = 'P'; 
    lastCommandTime = millis();
    pidMode         = true;
}

// ── Parser para Marcas de Intersección ────────────────────────────────────────
void parseMarkCommand(String input) {
    unsigned long now = millis();
    
    if (input.equals("MARK:L")) {
        leftMarkerDetected = true;
        lastLeftMarkerTime = now;
        Serial.println("ACK:MARK_L"); 
    } 
    else if (input.equals("MARK:R")) {
        rightMarkerDetected = true;
        lastRightMarkerTime = now;
        Serial.println("ACK:MARK_R");
    }
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

// ── API pública: Getters ─────────────────────────────────────────────────────
bool checkLeftMarker() { return leftMarkerDetected; }
bool checkRightMarker() { return rightMarkerDetected; }

void clearMarkers() {
    leftMarkerDetected = false;
    rightMarkerDetected = false;
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
    // Leer y despachar todos los comandos pendientes en el buffer serial de golpe
    while (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) continue;

        if (input.startsWith("ERROR:")) {
            parseErrorCommand(input);
        } 
        else if (input.startsWith("MARK:")) {
            parseMarkCommand(input);
        } 
        else if (input.length() == 1) {
            executeCommand(static_cast<char>(toupper(input[0])));
        }
    }

    // Watchdog: si Python deja de responder, frena el robot de seguridad
    unsigned long timeout = pidMode ? TIMEOUT_PID_MS : TIMEOUT_SIMPLE_MS;
    if (lastCommand != 'S' && (millis() - lastCommandTime > timeout)) {
        stopMotors();
        lastCommand = 'S';
        pidMode     = false;
        Serial.println("TIMEOUT: motores detenidos por falta de datos");
    }
}

// ── AVANZAR HASTA DETECTAR "N" CRUCES ESPECÍFICOS (Izquierda o Derecha) ──────
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

// ── AVANZAR HASTA DETECTAR "N" CRUCES CUALESQUIERA (Sin bloquear el flujo) ───
void avanzarConLineaHastaCruce(int totalCruces) {
    int contadorCruces = 0;
    
    clearMarkers();
    avanzarConLinea();

    while (contadorCruces < totalCruces) {
        updateLineFollower();

        if (checkLeftMarker() || checkRightMarker()) {
            contadorCruces++;
            Serial.print("[NAV] Intersección detectada #");
            Serial.println(contadorCruces);
            
            // ── OPTIMIZADO: Al limpiar los flags por flanco en Python, 
            // ya no necesitamos demoras lentas de milisegundos aquí.
            clearMarkers();
        }
        
        yield();
    }

    stopMotors();
    Serial.println("[NAV] Cruce(s) detectado(s). Motores en STOP.");
}
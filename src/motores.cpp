#include "motores.h"
#include "Sensores.h"
#include <ESP32Servo.h>

Servo servoMotor;

static int velActualIzq = 0;
static int velActualDer = 0;

// ===================== INICIALIZACIÓN =====================
void iniciarMotores() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  Serial.println("Motores OK");
}

// ===================== CONTROL BAJO NIVEL =====================
void motorIzq(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);

  if (velocidad > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (velocidad < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  analogWrite(PWMA, abs(velocidad));
  velActualIzq = velocidad;
}

void motorDer(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);

  if (velocidad > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (velocidad < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }

  analogWrite(PWMB, abs(velocidad));
  velActualDer = velocidad;
}

void parar() {
  motorIzq(0);
  motorDer(0);
}

// ===================== HELPERS =====================

// Diferencia angular mínima entre dos ángulos en [0,360)
// Devuelve valor en (-180, 180]: positivo = girar CW, negativo = girar CCW
static float angularDiff(float objetivo, float actual) {
  float diff = objetivo - actual;
  while (diff >  180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

// Filtro EMA (Exponential Moving Average) para suavizar el yaw
// alpha bajo = más suavizado, alpha alto = más respuesta
static float filtrarYaw(float yawNuevo, float yawPrev, float alpha = 0.25f) {
  return alpha * yawNuevo + (1.0f - alpha) * yawPrev;
}

// ===================== GIRO CON ÁNGULO — PD MEJORADO =====================
//
//  Mejoras respecto a la versión anterior:
//
//  1. Wrap-around correcto: todo en [0,360), angularDiff() para el error.
//  2. Filtro EMA en el yaw antes de calcular el derivativo, elimina spikes de ruido.
//  3. Zona muerta en el derivativo: si |derror/dt| < DEAD_BAND no se aplica Kd.
//  4. Criterio de salida por ventana de estabilidad: el robot debe mantenerse
//     dentro de la tolerancia por N ciclos consecutivos antes de detenerse,
//     en lugar de contar cruces de cero (que era frágil con oscilaciones).
//  5. Timeout absoluto de seguridad para no quedarse en bucle infinito.
//  6. Rampa de arranque: evita el golpe de corriente inicial en el peso.
//  7. VEL_MIN sólo se aplica cuando |error| > TOLERANCIA (no se pelea contra
//     la fricción cuando ya casi llegó).
//
void girarGrados(float grados) {
  if (abs(grados) < 0.5f) return;   // giro insignificante, ignorar

  Serial.print("[girar PD+] ");
  Serial.print(grados, 1);
  Serial.println(" deg");

  // ── Constantes (ajusta en pista) ─────────────────────────────────────────
  constexpr float kd_giro        = 3.0f;   // ganancia proporcional
  constexpr float kp_giro           = 5.5f;   // ganancia derivativa
  constexpr float EMA_ALPHA    = 0.30f;  // suavizado del yaw (0.1=mucho, 0.5=poco)
  constexpr float TOLERANCIA   = 1.2f;   // grados de error aceptable
  constexpr float DEAD_BAND_D  = 15.0f;  // °/s mínimos para aplicar Kd (filtro de ruido)
  constexpr int   VEL_MAX      = 255;
  constexpr int   VEL_MIN      = 40;      // mínimo para vencer fricción
  constexpr int   RAMPA_MS     = 80;     // tiempo de rampa de arranque
  constexpr int   CICLOS_OK    = 8;      // ciclos consecutivos dentro de tolerancia para salir
  constexpr unsigned long TIMEOUT_MS = 3000; // timeout de seguridad

  // ── Estado inicial ────────────────────────────────────────────────────────
  float yawRaw      = leerYaw();          // [0, 360)
  float yawFiltrado = yawRaw;
  float yawObjetivo = yawRaw + grados;
  while (yawObjetivo >= 360.0f) yawObjetivo -= 360.0f;
  while (yawObjetivo <    0.0f) yawObjetivo += 360.0f;

  float error          = angularDiff(yawObjetivo, yawFiltrado);
  float errorAnterior  = error;
  float derivAnterior  = 0.0f;

  int   ciclosEstable  = 0;
  unsigned long tAnterior = micros();
  unsigned long tInicio   = millis();

  // ── Rampa de arranque ─────────────────────────────────────────────────────
  // Sube suavemente de 0 a VEL_MIN en RAMPA_MS para no sacudir el peso.
  {
    int dir = (error > 0) ? 1 : -1;
    unsigned long tRampa = millis();
    while (millis() - tRampa < (unsigned long)RAMPA_MS) {
      float t   = (float)(millis() - tRampa) / RAMPA_MS;
      int   vel = (int)(t * VEL_MIN);
      motorIzq(-dir * vel);
      motorDer( dir * vel);
      delay(4);
    }
  }

  // ── Bucle PD principal ────────────────────────────────────────────────────
  while (ciclosEstable < CICLOS_OK) {

    // Timeout de seguridad
    if (millis() - tInicio > TIMEOUT_MS) {
      Serial.println(">>> [girar PD+] TIMEOUT — forzando parada");
      break;
    }

    // dt en segundos (usando micros para precisión)
    unsigned long tActual = micros();
    float dt = (tActual - tAnterior) / 1e6f;
    if (dt <= 0.0f) dt = 0.005f;
    tAnterior = tActual;

    // Leer y filtrar yaw
    yawRaw      = leerYaw();
    yawFiltrado = filtrarYaw(yawRaw, yawFiltrado, EMA_ALPHA);

    // Error angular con wrap-around correcto
    error = angularDiff(yawObjetivo, yawFiltrado);

    // ── Derivativo con zona muerta ────────────────────────────────────────
    float deriv = (error - errorAnterior) / dt;

    // Suavizar el derivativo con EMA también
    deriv = 0.4f * deriv + 0.6f * derivAnterior;

    // Zona muerta: ignorar derivativo si el cambio de ángulo es sólo ruido
    float derivActivo = (abs(deriv) > DEAD_BAND_D) ? deriv : 0.0f;

    float salidaPD = kd_giro * error + kp_giro * derivActivo;

    errorAnterior  = error;
    derivAnterior  = deriv;

    // ── Velocidad ─────────────────────────────────────────────────────────
    int vel = (int)abs(salidaPD);
    vel = constrain(vel, 0, VEL_MAX);

    // VEL_MIN sólo si todavía estamos lejos de la meta
    if (vel > 0 && vel < VEL_MIN && abs(error) > TOLERANCIA) {
      vel = VEL_MIN;
    }

    // ── Dirección ─────────────────────────────────────────────────────────
    // salidaPD positiva → error positivo → necesita girar CW
    int dir = (salidaPD > 0) ? 1 : -1;

    motorIzq(-dir * vel);
    motorDer( dir * vel);

    // ── Criterio de salida por estabilidad ───────────────────────────────
    if (abs(error) <= TOLERANCIA) {
      ciclosEstable++;
    } else {
      ciclosEstable = 0;   // cualquier salida de tolerancia reinicia el contador
    }

    // Debug opcional (descomenta si necesitas tuning)
    // Serial.printf("err=%.2f yaw=%.2f vel=%d\n", error, yawFiltrado, vel * dir);

    delay(6);   // ~160 Hz
  }

  // ── Freno electrónico ─────────────────────────────────────────────────────
  parar();

  Serial.print(">>> Giro PD+ completado — error final: ");
  Serial.print(angularDiff(yawObjetivo, leerYaw()), 2);
  Serial.println(" deg");
}

// ===================== SERVO =====================
void inicializarServo() {
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90);
  delay(500);
}

void mover_servos(int angulo) {
  angulo = constrain(angulo, 0, 180);
  servoMotor.write(angulo);
  delay(500);
}

// ===================== MOVIMIENTO SIMPLE =====================
void reversa() {
  reversa(150);
}

void reversa(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}

void avanza() {
  avanza(255);
}

void avanza(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}

void izquierda() {
  izquierda(150);
}

void izquierda(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}

void derecha() {
  derecha(150);
}

void derecha(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}

void avanzar(int tiempo) {
  avanza();
  delay(tiempo);
  parar();
}

void retroceder(int tiempo) {
  reversa();
  delay(tiempo);
  parar();
}

void girarIzquierda() {
  izquierda();
}

void girarDerecha() {
  derecha();
}

void detenerMotores() {
  parar();
}

// ===================== CONTROL CON PWM =====================
void ajustarMotores(int velocidadIzq, int velocidadDer) {
  motorIzq(velocidadIzq);
  motorDer(velocidadDer);
}
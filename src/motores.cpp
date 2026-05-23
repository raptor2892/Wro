#include "motores.h"
#include "Sensores.h"
#include <Arduino.h>
#include "line_follower.h"
#include <ESP32Servo.h>

Servo servoMotor;

static int velActualIzq = 0;
static int velActualDer = 0;

// ===================== INICIALIZACIÓN =====================
void iniciarMotores() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  Serial.println("Motores OK");
}

// ===================== CONTROL BAJO NIVEL =====================
void motorIzq(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, velocidad);
  } else if (velocidad < 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -velocidad);
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
  velActualIzq = velocidad;
}

void motorDer(int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (velocidad > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, velocidad);
  } else if (velocidad < 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -velocidad);
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
  velActualDer = velocidad;
}

void parar() {
  motorIzq(0);
  motorDer(0);
}

// ===================== HELPERS =====================
static float angularDiff(float objetivo, float actual) {
  float diff = objetivo - actual;
  while (diff >  180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

void girarGrados(float grados) {
  if (abs(grados) < 0.5f) return;

  Serial.print("[girar] "); Serial.print(grados); Serial.println("deg");

  constexpr int   VEL_GIRO     = 170;
  constexpr int   VEL_MIN      = 100;
  constexpr float ZONA_ARRANQUE = 20.0f;
  constexpr float ZONA_FRENADO  = 50.0f;
  constexpr float TOLERANCIA    = 1.5f;

  float yawInicial = NAN;
  while (isnan(yawInicial)) { yawInicial = leerYaw(); delay(10); }

  float objetivo = yawInicial + grados;
  if (objetivo >= 360) objetivo -= 360;
  if (objetivo <    0) objetivo += 360;

  float totalGrados = abs(grados);
  int   dirIzq      = (grados > 0) ?  1 : -1;
  int   dirDer      = (grados > 0) ? -1 :  1;

  while (true) {
    float yawActual = leerYaw();

    float restante  = abs(angularDiff(objetivo, yawActual));
    float recorrido = totalGrados - restante;

    int vel;
    if (recorrido < ZONA_ARRANQUE)
      vel = map((int)recorrido, 0, (int)ZONA_ARRANQUE, VEL_MIN, VEL_GIRO);
    else if (restante < ZONA_FRENADO)
      vel = map((int)restante,  0, (int)ZONA_FRENADO,  VEL_MIN, VEL_GIRO);
    else
      vel = VEL_GIRO;

    vel = constrain(vel, VEL_MIN, VEL_GIRO);

    motorIzq(dirIzq * vel);
    motorDer(dirDer * vel);

    if (restante <= TOLERANCIA) { Serial.println(">>> Giro OK"); break; }
    delay(10);
  }

  parar();
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
void reversa() { reversa(150); }
void reversa(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorIzq(-pwm);
  motorDer(-pwm);
}

void avanza() { avanza(255); }
void avanza(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorIzq(pwm);
  motorDer(pwm);
}

void izquierda() { izquierda(150); }
void izquierda(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorIzq(-pwm);
  motorDer( pwm);
}

void derecha() { derecha(150); }
void derecha(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorIzq( pwm);
  motorDer(-pwm);
}

void avanzar(int tiempo)    { avanza();  delay(tiempo); parar(); }
void retroceder(int tiempo) { reversa(); delay(tiempo); parar(); }
void girarIzquierda()       { izquierda(); }
void girarDerecha()         { derecha(); }
void detenerMotores()       { parar(); }

// ===================== CONTROL CON PWM =====================
void ajustarMotores(int velocidadIzq, int velocidadDer) {
  motorIzq(velocidadIzq);
  motorDer(velocidadDer);
}
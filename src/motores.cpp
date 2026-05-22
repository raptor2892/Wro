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

// ===================== GIRO CON ÁNGULO OPTIMIZADO =====================
// ===================== GIRO CON REVERSA DINÁMICA SI SE PASA =====================
void girarGrados(float grados) {
  Serial.print("[girar] ");
  Serial.print(grados);
  Serial.println("deg");
  
  float yaw_inicial = leerYaw();
  float yaw_actual = yaw_inicial;
  float yaw_objetivo = yaw_inicial + grados;

  // Normalizar ángulos entre -180 y 180
  while (yaw_objetivo > 180) yaw_objetivo -= 360;
  while (yaw_objetivo < -180) yaw_objetivo += 360;

  float diferencia = yaw_objetivo - yaw_actual;
  if (diferencia > 180) diferencia -= 360;
  if (diferencia < -180) diferencia += 360;

  float totalGrados = abs(grados);
  
  // Ajustes de dinámica
  float ZONA_FRENADO = totalGrados * 0.40; 
  float TOLERANCIA = 1.0; // Tolerancia más estricta ya que ahora puede regresar
  
  int VEL_MAX = 150;      
  int VEL_MIN = 45;       

  // Usamos un contador de seguridad para evitar que oscile infinitamente si la fricción es rara
  int intentosEstabilizacion = 0;

  while (abs(diferencia) > TOLERANCIA && intentosEstabilizacion < 100) {
    float restante = abs(diferencia);
    int vel = VEL_MAX;
    
    // 1. DIRECCIÓN DINÁMICA: Se recalcula en cada iteración
    // Si la diferencia es positiva, debe girar en un sentido; si es negativa (se pasó), en el otro.
    int dirIzq = (diferencia > 0) ? -1 : 1;
    int dirDer = (diferencia > 0) ? 1 : -1;

    // 2. CONTROL DE VELOCIDAD
    if (restante <= ZONA_FRENADO) {
      vel = map(restante * 100, 0, ZONA_FRENADO * 100, VEL_MIN, VEL_MAX);
    } else {
      vel = VEL_MAX;
    }
    vel = constrain(vel, VEL_MIN, VEL_MAX);
    
    // Si cambió de sentido para corregir, aumentamos el contador de estabilización
    if ((grados > 0 && diferencia < 0) || (grados < 0 && diferencia > 0)) {
      intentosEstabilizacion++;
      // Reducimos un poco la velocidad de corrección para evitar que vuelva a pasarse de largo
      vel = constrain(vel, VEL_MIN, VEL_MAX - 20); 
    }

    motorIzq(dirIzq * vel);
    motorDer(dirDer * vel);

    delay(5); 
    
    yaw_actual = leerYaw();
    diferencia = yaw_objetivo - yaw_actual;

    if (diferencia > 180) diferencia -= 360;
    if (diferencia < -180) diferencia += 360;
  }

  // --- FRENO ACTIVO FINAL ---
  // Al salir del bucle (porque entró en la tolerancia), aplicamos un contra-pulso corto
  // basándonos en la última dirección detectada para clavarlo en el sitio.
  int ultimoDirIzq = (diferencia > 0) ? -1 : 1;
  int ultimoDirDer = (diferencia > 0) ? 1 : -1;
  
  motorIzq(-ultimoDirIzq * VEL_MIN);
  motorDer(-ultimoDirDer * VEL_MIN);
  delay(25); 

  parar();
  Serial.println(">>> Giro corregido y completado");
}
// ===================== SERVO =====================
void inicializarServo() {
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90); // Posición inicial neutra
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
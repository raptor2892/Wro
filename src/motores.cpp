#include "motores.h"
#include "Sensores.h"
#include <Arduino.h>
#include <ESP32Servo.h>

Servo servoMotor;

// Variables PID
float errorAnterior = 0;
float integral = 0;
unsigned long tiempoAnterior = 0;

void avanzar() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 255);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 255);
}

void retroceder(){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 255);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 255);
}

void girarIzquierda(){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 255);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 255);
}

void girarDerecha(){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 255);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 255);
}

void detenerMotores() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
}

void avanzar_hasta_linea(){
    while (!verificarLineaAncha()) {
        float posicion = calcularPosicionLinea();
        controlarPID(posicion);
        delay(10); // Pequeño delay para estabilidad
    }
    // Detener motores al llegar
    ajustarMotores(0, 0);
}

void girarGrados(float grados) {
    float yaw_actual = leerYaw();
    float yaw_objetivo = yaw_inicial + grados;

    // Normalizar ángulos entre -180 y 180
    while (yaw_objetivo > 180) yaw_objetivo -= 360;
    while (yaw_objetivo < -180) yaw_objetivo += 360;

    float diferencia = yaw_objetivo - yaw_actual;

    // Elegir dirección de giro (la más corta)
    if (diferencia > 180) diferencia -= 360;
    if (diferencia < -180) diferencia += 360;

    // Girar en la dirección correcta
    if (diferencia > 0) {
        // Girar derecha
        girarDerecha();
    } else {
        // Girar izquierda
        girarIzquierda();
    }

    // Esperar hasta alcanzar el ángulo
    while (abs(diferencia) > 2.0) { // Tolerancia de 2 grados
        delay(10);
        yaw_actual = leerYaw();
        diferencia = yaw_objetivo - yaw_actual;

        if (diferencia > 180) diferencia -= 360;
        if (diferencia < -180) diferencia += 360;
    }

    // Detener motores
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
}

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

// Función para calcular la posición de la línea basada en sensores
float calcularPosicionLinea() {
    int pesos[8] = {-3, -2, -1, 0, 1, 2, 3, 4}; // Pesos para cada sensor, centrado en 0 para sensor 4
    float numerador = 0;
    float denominador = 0;
    
    for (int i = 0; i < 8; i++) {
        int lectura = digitalRead(lectura1 + i); // lectura1 es el primer sensor
        numerador += lectura * pesos[i];
        denominador += lectura;
    }
    
    if (denominador == 0) return 0; // Sin línea detectada
    return numerador / denominador;
}

// Función PID para controlar el robot
void controlarPID(float posicion) {
    unsigned long tiempoActual = millis();
    float deltaT = (tiempoActual - tiempoAnterior) / 1000.0; // en segundos
    tiempoAnterior = tiempoActual;
    
    float error = 0 - posicion; // Error: 0 es el centro deseado
    
    integral += error * deltaT;
    float derivada = (error - errorAnterior) / deltaT;
    errorAnterior = error;
    
    float output = KP * error + KI * integral + KD * derivada;
    
    // Limitar output
    output = constrain(output, -VELOCIDAD_BASE, VELOCIDAD_BASE);
    
    int velocidadIzq = VELOCIDAD_BASE - output;
    int velocidadDer = VELOCIDAD_BASE + output;
    
    ajustarMotores(velocidadIzq, velocidadDer);
}

// Función para ajustar motores con PWM
void ajustarMotores(int velocidadIzq, int velocidadDer) {
    // Motor izquierdo (A)
    if (velocidadIzq > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, velocidadIzq);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, -velocidadIzq);
    }
    
    // Motor derecho (B)
    if (velocidadDer > 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, velocidadDer);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, -velocidadDer);
    }
}
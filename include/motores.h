#ifndef MOTORES_H
#define MOTORES_H

#define AIN1 12
#define AIN2 13
#define PWMA 14
#define BIN1 15
#define BIN2 2
#define PWMB 5
#define SERVO_PIN 25

// Constantes PID
#define KP 1.0
#define KI 0.0
#define KD 0.1
#define VELOCIDAD_BASE 150

void avanzar();
void retroceder();
void girarIzquierda();
void girarDerecha();
void detenerMotores();
void avanzar_hasta_linea();

void girarGrados(float grados);
void inicializarServo();
void mover_servos(int angulo);

// Funciones PID
float calcularPosicionLinea();
void controlarPID(float posicion);
void ajustarMotores(int velocidadIzq, int velocidadDer);

#endif // MOTORES_H

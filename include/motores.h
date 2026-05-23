#ifndef MOTORES_H
#define MOTORES_H

#define AIN1 12
#define AIN2 13
#define PWMA 14
#define BIN1 26
#define BIN2 33
#define PWMB 25
#define SERVO_PIN 27


// Constantes PID
#define KP 1.0
#define KI 0.0
#define KD 0.1
#define VELOCIDAD_BASE 50

void avanzar(int tiempo);
void retroceder(int tiempo);
void avanza();
void avanza(int pwm);
void reversa();
void reversa(int pwm);
void izquierda();
void izquierda(int pwm);
void derecha();
void derecha(int pwm);
void girarIzquierda();
void girarDerecha();
void detenerMotores();

void girarGrados(float grados);
void inicializarServo();
void mover_servos(int angulo);
void iniciarMotores();

// Funciones PID
void ajustarMotores(int velocidadIzq, int velocidadDer);

#endif // MOTORES_H

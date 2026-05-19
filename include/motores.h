#ifndef MOTORES_H
#define MOTORES_H

#define AIN1 12
#define AIN2 13
#define PWMA 14
#define BIN1 2
#define BIN2 15
#define PWMB 5
#define SERVO_PIN 25

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
<<<<<<< HEAD
=======
void avanzar_hasta_linea();
>>>>>>> a9d26d1bb75b58abf3071deb52fe26dd5f5cc306

void girarGrados(float grados);
void inicializarServo();
void mover_servos(int angulo);

// Funciones PID
void ajustarMotores(int velocidadIzq, int velocidadDer);

#endif // MOTORES_H

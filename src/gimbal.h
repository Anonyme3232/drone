#pragma once
#include <ESP32Servo.h>

// Inicializa los servos en los pines indicados
void servos_init(int pin_pitch);

// Mueve el servo de pitch a un ángulo (0-180)
void servo_set_pitch(float angulo);

// Centra ambos servos (90°)
void servos_center();
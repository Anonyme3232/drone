#include <Arduino.h>
#include "mando.h"

#define JOYX 16
#define JOYY 15

const float max_angle = 30.0f; // Ángulo máximo para el joystick

void setupJoystick() {
    // Configuración de los pines del joystick si es necesario
    analogReadResolution(12); // Configura la resolución de lectura a 12 bits (0-4095)
}

float readJoystick(int pin) {
    float raw = analogRead(pin);
    float centered = raw - 2048.0f; // Centrar en 0
    float normalized = centered / 2048.0f; // Normalizar a [-1
    Serial.print("Joystick: ");
    Serial.println(normalized);

    // evitar zona muerta 
    if (abs(normalized) < 0.05f) {
        normalized = 0.0f;
    }

    return normalized * max_angle; // Escalar al rango de ángulo deseado
}

JoystickData readJoystick() {
    JoystickData data;
    data.roll = readJoystick(JOYX);
    data.pitch = readJoystick(JOYY);
    return data;
}
#include "pid.h"
#include <Arduino.h>

float PID::compute(float setpoint, float medido, float dt) {
    float error = setpoint - medido;

    // Proporcional
    float P = Kp * error;

    // Integral con anti-windup
    integral += error * dt;
    integral  = constrain(integral, -integral_max, integral_max);
    float I   = Ki * integral;

    // Derivativo filtrado
    float d_raw  = (error - error_anterior) / dt;
    d_filtrado   = alpha * d_raw + (1.0f - alpha) * d_filtrado;
    float D      = Kd * d_filtrado;

    error_anterior = error;

    return P + I + D;
}

void PID::reset() {
    integral       = 0;
    error_anterior = 0;
    d_filtrado     = 0;
}
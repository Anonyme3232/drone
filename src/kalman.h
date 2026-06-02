#pragma once

struct kalman {
    // Variar estos parametros para ajustar el filtro
    // Ruido del proceso
    float Q_angle = 0.001f;
    float Q_bias  = 0.002f;
    // Ruido de la medición
    float R_measure = 0.03f;

    float angle = 0; // Ángulo estimado
    float bias = 0;  // Sesgo del giroscopio

    float P[2][2] = {{0, 0}, {0, 0}}; // Matriz de covarianza de error
    float update(float anguloMedido, float gyroRate, float dt);
};
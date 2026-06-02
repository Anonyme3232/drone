#include "kalman.h"
#include <math.h>

float kalman::update(float anguloMedido, float gyroRate, float dt) {
    // prediccion

    float rate = gyroRate - bias;
    angle += dt * rate;

    // actualizacion de la matriz de covarianza
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correccion
    // Innovacion

    float y = anguloMedido - angle;

    // Ganancia de Kalman
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    // Aplicar correccion
    angle += K0 * y;
    bias  += K1 * y;

    // Actualizar matriz de covarianza
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
}
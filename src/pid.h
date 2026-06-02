// pid.h
#pragma once

struct PID {
    float Kp, Ki, Kd;
    float integral       = 0;
    float error_anterior = 0;
    float d_filtrado     = 0;
    float alpha          = 0.1f;
    float integral_max   = 200.0f;

    float compute(float setpoint, float medido, float dt);
    void  reset();
};
// imu.h
#pragma once

struct IMUData {
    float roll;
    float pitch;
    float gx, gy, gz; // velocidad angular en °/s
};

bool imu_init();
void calibrateMPU();
IMUData imu_read(float dt);
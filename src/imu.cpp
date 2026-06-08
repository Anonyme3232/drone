// imu.cpp
#include "imu.h"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include "kalman.h"

#define MPU_ADDR   0x68
#define AXL_OUT_H  0x3B
#define PWR_MGMT_1 0x6B
#define WHOAMI     0x75
#define IAM        0x71

static float roll_filtrado  = 0;
static float pitch_filtrado = 0;
static kalman kalmanRoll;
static kalman kalmanPitch;

static byte readMPU(byte reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (byte)1, true);
    return Wire.read();
};

float gx_sum = 0, gy_sum = 0, gz_sum = 0;
float ax_sum = 0, ay_sum = 0, az_sum = 0;

void calibrateMPU(){
    Serial.println("Calibrando MPU-9250... NO MOVER EL SENSOR");
    delay(2000);

    const int N = 1000;

    for (int i = 0; i<N; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(AXL_OUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);

        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // temperatura
        int16_t gx = Wire.read() << 8 | Wire.read();
        int16_t gy = Wire.read() << 8 | Wire.read();
        int16_t gz = Wire.read() << 8 | Wire.read();

        ax_sum += ax / 16384.0f;
        ay_sum += ay / 16384.0f;
        az_sum += az / 16384.0f;
        gx_sum += gx / 131.0f;
        gy_sum += gy / 131.0f;
        gz_sum += gz / 131.0f;

        delay(2);
    }

    gx_sum = gx_sum / N;
    gy_sum = gy_sum / N;
    gz_sum = gz_sum / N;
    ax_sum = ax_sum / N;
    ay_sum = ay_sum / N;
    az_sum = (az_sum / N) - 1.0f; // restar gravedad

    Serial.println("Calibración completa:");
    Serial.printf("Gyro bias (°/s): X=%.2f  Y=%.2f  Z=%.2f\n", gx_sum, gy_sum, gz_sum);
    Serial.printf("Accel bias (g):  X=%.2f  Y=%.2f  Z=%.2f\n", ax_sum, ay_sum, az_sum);
}

static void writeMPU(byte reg, byte data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

bool imu_init() {
    Wire.begin(8, 9); 
    delay(100);

    // Verificar que el MPU-9250 responde

    byte id = readMPU(WHOAMI);
    if (id != IAM && id != 0x70) {
        Serial.printf("Error: MPU no encontrado (WHOAMI=0x%02X)\n", id);
        return false;
    }

    writeMPU(PWR_MGMT_1, 0x00); // Despertar el MPU
    delay(100);
    return true;
}

IMUData imu_read(float dt) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(AXL_OUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t Ax = Wire.read() << 8 | Wire.read();
    int16_t Ay = Wire.read() << 8 | Wire.read();
    int16_t Az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // temperatura
    int16_t Gx = Wire.read() << 8 | Wire.read();
    int16_t Gy = Wire.read() << 8 | Wire.read();
    int16_t Gz = Wire.read() << 8 | Wire.read();

    float ax = (Ax / 16384.0f) - ax_sum;
    float ay = (Ay / 16384.0f) - ay_sum;
    float az = (Az / 16384.0f) - az_sum;
    float gx = (Gx / 131.0f) - gx_sum;
    float gy = (Gy / 131.0f) - gy_sum;
    float gz = (Gz / 131.0f) - gz_sum;

    // Ángulos del acelerómetro
    float roll_a  = atan2f(ay, az)                       * (180.0f / M_PI);
    float pitch_a = atan2f(-ax, sqrtf(ay*ay + az*az))   * (180.0f / M_PI);
    roll_a = -roll_a; // corregir signo para que coincida con la convención de giroscopio
    pitch_a = -pitch_a;

    // Filtro complementario
    /*
    roll_filtrado  = 0.98f * (roll_filtrado  + gx * dt) + 0.02f * roll_a;
    pitch_filtrado = 0.98f * (pitch_filtrado + gy * dt) + 0.02f * pitch_a;
    */

    // Filtro de Kalman
    
    IMUData data;
    data.roll  = kalmanRoll.update(roll_a, gx, dt);
    data.pitch = kalmanPitch.update(pitch_a, gy, dt);
    //data.roll  = roll_filtrado;
    //data.pitch = pitch_filtrado;
    data.gx    = gx;
    data.gy    = gy;
    data.gz    = gz;

    return data;
}


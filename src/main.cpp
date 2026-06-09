#include <Arduino.h>
#include <ESP32Servo.h>
#include "imu.h"
#include "pid.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "gimbal.h"
#include "mando.h"

#define RGB_LED 48
#define SERVO_1 6
  
// ---- Motores ----
Servo motor1, motor2, motor3, motor4;

// ---- PIDs (ángulo externo + velocidad interna) ----
PID pid_roll_ang,  pid_roll_vel;
PID pid_pitch_ang, pid_pitch_vel;
PID pid_yaw_vel;

// ---- PID gimbal (independiente del dron) ----
PID pid_gimbal_pitch;

// ---- Tiempo ----
unsigned long lastTime = 0;

// ---- Función para setear el brillo del LED del motor (simula el PWM) ----
void SetMotorLed(int pin, float microseconds) {
    int brightness = map(int(microseconds), 1000, 2000, 0, 255);
    analogWrite(pin, brightness);
};

Adafruit_NeoPixel pixels(1, RGB_LED, NEO_GRB + NEO_KHZ800);

void setup() {
    pinMode(21, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(1, OUTPUT);
    pinMode(7, OUTPUT);

    Serial.begin(115200);
    // Espera máximo 3 segundos el Serial
    // Si no hay PC conectada, continúa de todas formas
    unsigned long t = millis();
    while (!Serial && millis() - t < 3000) delay(10);
    
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.setBrightness(10);
    pixels.show();
    Serial.println("ESP32 CORRIENDO!!");
    delay(500);
    
    if(!imu_init()) {
        Serial.println("Error al inicializar IMU");
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.show();
        while(true) delay(1000);
    }
    
    pixels.setPixelColor(0, pixels.Color(210, 203, 0));
    pixels.setBrightness(10);
    pixels.show();
    Serial.println("¡¡MPU CALIBRANDOSE NO MOVER EL SENSOR!!");
    calibrateMPU();
    
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.setBrightness(10);
    pixels.show();
    Serial.println("MPU-9250 LISTO!!");
    lastTime = micros();

    // Ganancias al azar pero falta simular MATLAB
    // PID de roll
    pid_roll_ang.Kp  = 1.32f; pid_roll_ang.Ki  = 0.001f; pid_roll_ang.Kd  = 0.15f;
    pid_roll_vel.Kp  = 3.6f;  pid_roll_vel.Ki  = 0.005f; pid_roll_vel.Kd  = 0.005f;
    // PID de pitch
    pid_pitch_ang.Kp = 1.32f; pid_pitch_ang.Ki = 0.001f; pid_pitch_ang.Kd = 0.15f;
    pid_pitch_vel.Kp = 3.6f;  pid_pitch_vel.Ki = 0.005f; pid_pitch_vel.Kd = 0.005f;
    // PID de yaw (solo velocidad por mientras)
    pid_yaw_vel.Kp   = 0.0f;  pid_yaw_vel.Ki   = 0.0f;   pid_yaw_vel.Kd   = 0.0f;

    // PID gimbal (independiente, solo para el servo)
    /*
    pid_gimbal_pitch.Kp = 1.2f;
    pid_gimbal_pitch.Ki = 0.0001f;
    pid_gimbal_pitch.Kd = 0.0006f;
    */

    // Motores
    /*
    motor1.attach(4, 1000, 2000);
    motor2.attach(5, 1000, 2000);
    motor3.attach(6, 1000, 2000);
    motor4.attach(7, 1000, 2000);
    
    // Armar ESCs
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    delay(2000); // mandar señal de inicio a los ESCs y esperar 2 segundos
    */

    lastTime = micros();
    Serial.println("Listo!");
    servos_init(SERVO_1);
    setupJoystick();
}

void loop() {
    // ---- Delta tiempo ----
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    if (dt <= 0.0f) { return; }

    // ---- Leer IMU ----
    IMUData data = imu_read(dt);

    // ---- Leer mando ----
    JoystickData joystick = readJoystick();

    // ---- Setpoints (hover estatico) ----
    float sp_roll  = joystick.roll;
    float sp_pitch = -joystick.pitch;
    float sp_yaw   = 0.0f;
    float throttle = 1070.0f; // cambiar con el control después

    // ---- PIDs en cascada ----
    float roll_vel_sp  = pid_roll_ang.compute(sp_roll,   data.roll,  dt);
    float pitch_vel_sp = pid_pitch_ang.compute(sp_pitch, data.pitch, dt);

    float roll_out  = pid_roll_vel.compute(roll_vel_sp,  data.gx, dt);
    float pitch_out = pid_pitch_vel.compute(pitch_vel_sp, data.gy, dt);
    float yaw_out   = pid_yaw_vel.compute(sp_yaw, data.gz, dt);

    // ---- Mixer ----
    /*
        M1.       M2
              X
        M3.       M4
    */
    float m1 = throttle - roll_out - pitch_out - yaw_out;
    float m2 = throttle - roll_out + pitch_out + yaw_out;
    float m3 = throttle + roll_out - pitch_out + yaw_out;
    float m4 = throttle + roll_out + pitch_out - yaw_out;

    if (throttle < 1000.0f) { // si el throttle es muy bajo, apagar motores y resetear PID
        m1 = m2 = m3 = m4 = 1000.0f;
        pid_roll_ang.reset();
        pid_roll_vel.reset();
        pid_pitch_ang.reset();
        pid_pitch_vel.reset();
        pid_yaw_vel.reset();
    }

    // ---- Constrain a rangos válidos para los ESCs ----
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);
      
    /*
    // ---- Mandar a motores ----
    motor1.writeMicroseconds((int)m1);
    motor2.writeMicroseconds((int)m2);
    motor3.writeMicroseconds((int)m3);
    motor4.writeMicroseconds((int)m4);
    */
    /*
    float correccion = 90.0f + pid_gimbal_pitch.compute(0.0f, data.pitch, dt);
    servo_set_pitch(correccion);
    */
  
    // ---- Debug ----
    Serial.print(">roll:");  Serial.println(data.roll);
    Serial.print(">pitch:"); Serial.println(data.pitch);
    Serial.print(">gx:");    Serial.println(data.gx);
    Serial.print(">gy:");    Serial.println(data.gy);
    Serial.print(">gz:");    Serial.println(data.gz);
    /*
    Serial.print(">M1:"); Serial.println(m1);
    Serial.print(">M2:"); Serial.println(m2);
    Serial.print(">M3:"); Serial.println(m3);
    Serial.print(">M4:"); Serial.println(m4);
    */

    SetMotorLed(21, m1);
    SetMotorLed(12, m2);
    SetMotorLed(1,  m3);
    SetMotorLed(7,  m4);

    delay(5); // 200Hz
}
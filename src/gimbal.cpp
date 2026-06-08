#include "gimbal.h"

static Servo servo_pitch;

void servos_init(int pin_pitch) {
    servo_pitch.attach(pin_pitch);
    servos_center();
}

void servo_set_pitch(float angulo) {
    angulo = constrain(angulo, 0, 180);
    servo_pitch.write((int)angulo);
}

void servos_center() {
    servo_pitch.write(90);
}
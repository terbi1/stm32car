#include "pi_controller.h"

float Feed_Forward(float base, float setpoint) {
    return base + Kff * setpoint;
}

float Proportional(float error) {
    return Kp * error;
}

float Integral(float error, float dt, float current_integral, float output) {
    if(output > 100.0f && error > 0) return current_integral;
    if(output < 0 && error < 0) return current_integral;

    return current_integral + error * dt;
}

float System_Input(PI_Controller *p, float base, float setpoint, float error, float dt) {
    float ff = Feed_Forward(base, setpoint);
    float output = ff + Proportional(error) + Ki * p->integral;
    p->integral = Integral(error, dt, p->integral, output);

    if(output > 100.0f) output = 100.f;
    else if(output < 0.0f) output = 0.0f;

    return output;
};

float Speed_Update(Speed* spd, int32_t tick, uint32_t t, uint32_t now) {
    int32_t dn = tick - spd->tick;
    if(dn > 0) {
        spd->rpm = dn *  1.5f * 72000000.0f / (float)(t-spd->t);
        spd->tick = tick;
        spd->t = t;
    } else if ((now - spd->t) > 14400000u) {
        spd->rpm = 0;
    }

    return spd->rpm;
}
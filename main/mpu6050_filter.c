#include "mpu6050_filter.h"
#include <math.h>

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void filter_init(void) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
}

void mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    integralFBx += ex * Ki * dt;
    integralFBy += ey * Ki * dt;
    integralFBz += ez * Ki * dt;

    gx += Kp * ex + integralFBx;
    gy += Kp * ey + integralFBy;
    gz += Kp * ez + integralFBz;

    float pa = q1, pb = q2, pc = q3;
    q0 += (-pa * gx - pb * gy - pc * gz) * (0.5f * dt);
    q1 += (q0 * gx + pb * gz - pc * gy) * (0.5f * dt);
    q2 += (q0 * gy - pa * gz + pc * gx) * (0.5f * dt);
    q3 += (q0 * gz + pa * gy - pb * gx) * (0.5f * dt);

    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

void get_euler_angles(float *roll, float *pitch, float *yaw) {
    *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    *pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    *yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}

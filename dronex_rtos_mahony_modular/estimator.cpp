#include "estimator.h"
#include <math.h>

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
float twoKpDef = 0.8f;   
float twoKiDef = 0.002f; 
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

float actual_pitch = 0.0, actual_roll = 0.0, actual_yaw = 0.0;

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Estimator_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; 
        ay *= recipNorm; 
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(twoKiDef > 0.0f) {
            integralFBx += twoKiDef * halfex * dt;
            integralFBy += twoKiDef * halfey * dt;
            integralFBz += twoKiDef * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    gx *= (0.5f * dt); 
    gy *= (0.5f * dt); 
    gz *= (0.5f * dt);
    
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; 
    q1 *= recipNorm; 
    q2 *= recipNorm; 
    q3 *= recipNorm;

    actual_roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.2958f;
    actual_pitch = asin(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * 57.2958f;
    actual_yaw   = atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2958f;
}

float Estimator_GetPitch() { return actual_pitch; }
float Estimator_GetRoll() { return actual_roll; }
float Estimator_GetYaw() { return actual_yaw; }
#include "pid.h"

void PID_Init(PID_t *pid) {
    if (!pid) return;
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->OutMax = 0.0f;
    pid->OutMin = 0.0f;
    pid->ErrorIntMax = 0;
    pid->ErrorIntMin = 0;
    pid->Target = 0;
    pid->Actual = 0;
    pid->Out = 0.0f;
    pid->Error = 0.0f;
    pid->ErrorLast = 0.0f;
    pid->ErrorInt = 0.0f;
}

void PID_Update(PID_t *pid) {
    if (!pid) return;

    float err = (float)pid->Target - (float)pid->Actual;
    pid->Error = err;

    /* 积分限幅：仅当边界被显式设置（非 0）时才钳位，
       避免默认 0/0 边界把积分项强行清零 */
    pid->ErrorInt += err;
    if (pid->ErrorIntMax != 0 || pid->ErrorIntMin != 0) {
        if (pid->ErrorInt > pid->ErrorIntMax) pid->ErrorInt = (float)pid->ErrorIntMax;
        if (pid->ErrorInt < pid->ErrorIntMin) pid->ErrorInt = (float)pid->ErrorIntMin;
    }

    pid->Out = pid->Kp * err
             + pid->Ki * pid->ErrorInt
             + pid->Kd * (err - pid->ErrorLast);
    pid->ErrorLast = err;

    /* 输出限幅 */
    if (pid->Out > pid->OutMax) pid->Out = pid->OutMax;
    if (pid->Out < pid->OutMin) pid->Out = pid->OutMin;
}

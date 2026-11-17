#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 位置式 PID。字段类型与 demo.cpp 的用法一一对应：
 *   g_pid.Kp/Ki/Kd         -> float
 *   g_pid.OutMax/OutMin    -> float
 *   g_pid.ErrorIntMax/Min  -> int
 *   g_pid.Target/Actual    -> int（图像中心 / 目标点 x）
 *   g_pid.Out              -> float
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float OutMax;
    float OutMin;

    int   ErrorIntMax;
    int   ErrorIntMin;

    int   Target;      /* 期望值，如 g_IMG_CENTER */
    int   Actual;      /* 实测值，如 target_x */
    float Out;         /* 本次输出 */

    /* 内部状态（调用方无需关心） */
    float Error;
    float ErrorLast;
    float ErrorInt;
} PID_t;

void PID_Init(PID_t *pid);
void PID_Update(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */

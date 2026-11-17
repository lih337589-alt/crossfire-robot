#ifndef CROSSFIRE_STEERING_H
#define CROSSFIRE_STEERING_H

/*
 * 控制层 —— 转向合成（纯逻辑，不依赖 OpenCV/硬件，可独立测试）。
 * 对应原 demo.cpp 的三处控制逻辑：
 *   - PID 转向：steer = SERVO_MID + PID输出 × servo_gain
 *   - 朝向辅助：航向偏差 > 死区时按增益叠加补偿
 *   - 角度回正：steer = SERVO_MID - 角度误差 × P 增益
 * 所有输出都限幅到 [servo_right, servo_left]（舵机微秒值，更大=更左）。
 */

struct SteeringParams {
    int   servo_mid   = 1455;   /* 机械正前方 */
    int   servo_left  = 2000;   /* 左转极限（微秒） */
    int   servo_right = 1000;   /* 右转极限（微秒） */
    float servo_gain  = -2.0f;  /* PID 输出 → 舵机 增益 */
    float head_deadzone = 8.0f; /* 航向偏差死区（度） */
    float head_gain     = 6.0f; /* 航向补偿增益 */
};

/* 角度归一化到 [-180, 180]（±180 保持不变，与单元测试约定一致） */
float normalize_angle(float a);

/* 朝向辅助：按航向偏差修正 base 舵机值 */
int heading_assist(float yaw, float target_heading, int base, const SteeringParams& p);

/* 视觉转向合成：PID 输出 + 朝向辅助 + 限幅 */
int compose_steer(float pid_out, float yaw, float target_heading, const SteeringParams& p);

/* 角度回正转向：角度误差 × P 增益 + 限幅 */
int compose_align_steer(float angle_error, float steer_p_gain, const SteeringParams& p);

#endif /* CROSSFIRE_STEERING_H */

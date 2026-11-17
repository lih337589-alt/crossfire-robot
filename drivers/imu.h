#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * IMU 姿态模块（串口）。
 *
 * ★ 协议假设：按「维特智能 JY901 / JY61」串口协议重建（0x55 帧头）。
 *   依据：demo.cpp 里 imu_init("/dev/ttyS0", B9600) + 输出绝对偏航角 + 注释
 *   「约 148ms/轮」，正是 JY901 在 9600bps 下 11 字节角度帧的典型耗时。
 *   若你的模块是其他型号（GY-25 / 其他 0x55 协议），只需改 imu.c 的解析部分。
 */
#define IMU_FRAME_HEAD   0x55
#define IMU_ANGLE_TYPE   0x53   /* 角度帧：Roll / Pitch / Yaw */

typedef struct {
    float roll_f;       /* 横滚角（度） */
    float pitch_f;      /* 俯仰角（度） */
    float yaw_f;        /* 偏航角（度），JY901 默认 0~360 */
    float yaw_offset;   /* 校准零偏（备用） */
} IMU_Data;

/* 打开串口并配置为 raw 模式 + 200ms 读超时；成功返回 fd，失败返回 -1 */
int  imu_init(const char *dev, int baud);

/* 读 samples 个样本求稳定零偏（打印 "Keep IMU still" 提示） */
void imu_calibration(int fd, IMU_Data *data, int samples);

/* 读一帧角度数据；成功返回 1，超时/错误/持续噪声帧返回 -1
 * 帧布局（11 字节）：0x55 0x53 RollL RollH PitchL PitchH YawL YawH TL TH SUM */
int  imu_read_parse(int fd, IMU_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */

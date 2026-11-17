#ifndef PCA9685_H
#define PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * PCA9685 —— 16 通道 12-bit PWM 控制器（I2C）。
 *
 * ★ 上车前请核对下面三个硬件参数（这三点是我根据 run.sh 与芯片手册重建的）：
 *   - I2C 总线：run.sh 明确提到舵机/电调走 /dev/i2c-5
 *   - I2C 地址：PCA9685 默认 0x40（A0~A5 全接地）
 *   - PWM 频率：舵机/电调标准 50Hz（周期 20ms=20000us，与 SPEED_RUN=1670、
 *     SERVO_MID=1455 这些微秒值完全吻合）
 */
#define PCA9685_I2C_BUS   "/dev/i2c-5"
#define PCA9685_I2C_ADDR  0x40
#define PCA9685_FREQ_HZ   50

/* 通道号约定（与 demo.cpp 的 CH_MOTOR/CH_SERVO 一致，仅作参考注释） */
/* CH_MOTOR=1  电机电调 */
/* CH_SERVO=0  转向舵机 */
/* 通道 2=云台左右，通道 3=云台俯仰 */

/* 初始化（打开 I2C、设置 50Hz、全通道输出 0） */
void pca9685_init(void);
/* Returns nonzero only after the I2C device and all required init writes succeeded. */
int pca9685_is_ready(void);

/* 释放资源 */
void pca9685_deinit(void);

/* 写某通道的脉冲宽度（单位：微秒），内部做了通道/脉宽合法性钳位 */
void pca9685_write_us_safe(int channel, int us);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_H */

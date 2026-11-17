#include "pca9685.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define MODE1       0x00
#define MODE2       0x01
#define PRESCALE    0xFE
#define LED0_ON_L   0x06    /* 通道 0 的 ON_L 寄存器；每通道占 4 字节 */

static int g_fd = -1;
static int g_ready = 0;

static int i2c_read_reg(unsigned char reg, unsigned char *val) {
    if (g_fd < 0) return -1;
    if (write(g_fd, &reg, 1) != 1) return -1;
    if (read(g_fd, val, 1) != 1) return -1;
    return 0;
}

/* 写寄存器 reg + 若干数据字节（依赖 MODE1 的 AI=1 自动递增） */
static int i2c_write_bytes(unsigned char reg, const unsigned char *data, int len) {
    unsigned char buf[17];
    if (g_fd < 0) return -1;
    if (len < 0 || len + 1 > (int)sizeof(buf)) return -1;
    buf[0] = reg;
    for (int i = 0; i < len; i++) buf[i + 1] = data[i];
    if (write(g_fd, buf, len + 1) != len + 1) return -1;
    return 0;
}

void pca9685_init(void) {
    if (g_fd >= 0 && g_ready) return;
    g_ready = 0;
    if (g_fd >= 0) { close(g_fd); g_fd = -1; }

    g_fd = open(PCA9685_I2C_BUS, O_RDWR);
    if (g_fd < 0) {
        perror("pca9685_init: open " PCA9685_I2C_BUS " failed");
        return;
    }
    if (ioctl(g_fd, I2C_SLAVE, PCA9685_I2C_ADDR) < 0) {
        perror("pca9685_init: ioctl I2C_SLAVE failed");
        goto fail;
    }

    /* 初始化序列：SLEEP → 写 PRESCALE → 唤醒并开 AI（自动递增）。
       说明：PCA9685 真正的软复位（SWRST=0x06）必须发到 I2C 广播地址 0x00；
       向本设备地址单字节写 0x06 只会移动寄存器指针、无复位效果，故此处不再伪复位，
       下面的寄存器配置即可把芯片带入确定状态。 */
    usleep(10000);

    unsigned char mode1 = 0x00;
    if (i2c_read_reg(MODE1, &mode1) != 0) goto fail;
    unsigned char sleep_mode = (unsigned char)((mode1 & 0x7F) | 0x10);
    if (i2c_write_bytes(MODE1, &sleep_mode, 1) != 0) goto fail;

    unsigned char prescale =
        (unsigned char)(25000000.0f / (4096.0f * (float)PCA9685_FREQ_HZ) - 1.0f + 0.5f);
    if (i2c_write_bytes(PRESCALE, &prescale, 1) != 0) goto fail;

    unsigned char run_mode = (unsigned char)((mode1 & 0x6F) | 0x20);
    if (i2c_write_bytes(MODE1, &run_mode, 1) != 0) goto fail;
    usleep(5000);

    for (int ch = 0; ch < 16; ch++) {
        unsigned char d[4] = {0, 0, 0, 0};
        if (i2c_write_bytes((unsigned char)(LED0_ON_L + ch * 4), d, 4) != 0) goto fail;
    }
    g_ready = 1;
    return;

fail:
    perror("pca9685_init: I2C register operation failed");
    close(g_fd);
    g_fd = -1;
    g_ready = 0;
}

int pca9685_is_ready(void) { return g_ready; }

void pca9685_deinit(void) {
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    g_ready = 0;
}

void pca9685_write_us_safe(int channel, int us) {
    if (g_fd < 0 || !g_ready) return;
    if (channel < 0 || channel > 15) return;

    int period_us = 1000000 / PCA9685_FREQ_HZ;   /* 50Hz -> 20000us */
    if (us < 0) us = 0;
    /* OFF 寄存器仅 12bit：us == period_us 时 off 计算为 4096，会被 0x0F 掩码截成 0
       （100% 占空比变成 0%），因此钳到 period_us - 1，off 最大 4095 */
    if (us >= period_us) us = period_us - 1;

    /* ON=0, OFF=off：前 off/4096 周期为高电平，即脉宽 us 微秒 */
    int off = (int)((long)us * 4096 / period_us);
    unsigned char d[4];
    d[0] = 0;                    /* ON_L */
    d[1] = 0;                    /* ON_H */
    d[2] = (unsigned char)(off & 0xFF);          /* OFF_L */
    d[3] = (unsigned char)((off >> 8) & 0x0F);   /* OFF_H */
    if (i2c_write_bytes((unsigned char)(LED0_ON_L + channel * 4), d, 4) != 0) g_ready = 0;
}

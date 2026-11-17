#include "imu.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int imu_init(const char *dev, int baud) {
    if (!dev) return -1;

    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("imu_init: open failed");
        return -1;
    }
    fcntl(fd, F_SETFL, 0);   /* 清 O_NDELAY，恢复阻塞读（配合 VTIME 超时） */

    struct termios opt;
    if (tcgetattr(fd, &opt) != 0) { close(fd); return -1; }

    cfsetispeed(&opt, (speed_t)baud);
    cfsetospeed(&opt, (speed_t)baud);

    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;              /* 8 数据位 */
    opt.c_cflag &= ~PARENB;          /* 无校验 */
    opt.c_cflag &= ~CSTOPB;          /* 1 停止位 */
    opt.c_cflag &= ~CRTSCTS;         /* 无硬件流控 */

    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* raw 模式 */
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);          /* 无软件流控 */
    opt.c_oflag &= ~OPOST;

    opt.c_cc[VMIN]  = 0;
    opt.c_cc[VTIME] = 2;             /* 200ms 读超时：9600bps 下帧间隔约 100~150ms，
                                        留一帧余量；掉线时单次读最多阻塞 0.2s，
                                        避免实时控制环被卡死 */

    if (tcsetattr(fd, TCSANOW, &opt) != 0) { close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void imu_calibration(int fd, IMU_Data *data, int samples) {
    if (fd < 0 || !data || samples <= 0) return;
    printf("Keep IMU still, calibrating...\n");
    float sum = 0.0f;
    int ok = 0;
    for (int i = 0; i < samples; i++) {
        if (imu_read_parse(fd, data) == 1) {
            sum += data->yaw_f;
            ok++;
        }
    }
    if (ok > 0) data->yaw_offset = sum / (float)ok;
    printf("IMU calibration done (offset=%.2f, %d/%d samples)\n",
           data->yaw_offset, ok, samples);
}

static int read_exact(int fd, unsigned char *buf, int len) {
    int got = 0;
    while (got < len) {
        int n = read(fd, buf + got, len - got);
        if (n <= 0) return -1;
        got += n;
    }
    return 0;
}

/* 校验失败/非角度帧的最大重同步次数：防止持续噪声帧把本函数变成死循环 */
#define IMU_MAX_RESYNC 8

int imu_read_parse(int fd, IMU_Data *data) {
    if (fd < 0 || !data) return -1;

    unsigned char buf[11];
    int attempts = 0;
    while (attempts < IMU_MAX_RESYNC) {
        /* 1) 找帧头 0x55 */
        unsigned char c = 0;
        int n;
        do {
            n = read(fd, &c, 1);
            if (n != 1) return -1;   /* 超时/错误 */
        } while (c != IMU_FRAME_HEAD);
        buf[0] = IMU_FRAME_HEAD;

        /* 2) 读剩余 10 字节 */
        if (read_exact(fd, buf + 1, 10) != 0) return -1;

        /* 3) 校验和：前 10 字节累加 == 第 11 字节 */
        unsigned char sum = 0;
        for (int i = 0; i < 10; i++) sum += buf[i];
        if (sum != buf[10]) { ++attempts; continue; }

        /* 4) 只接受角度帧，其他帧跳过继续读 */
        if (buf[1] != IMU_ANGLE_TYPE) { ++attempts; continue; }

        /* JY901 角度帧：0x55 0x53 RollL RollH PitchL PitchH YawL YawH TL TH SUM
           小端有符号，单位 0.01°；数据从 buf[2] 开始（修复 buf[3] 起读的整体错位） */
        const int16_t roll  = (int16_t)((buf[3] << 8) | buf[2]);
        const int16_t pitch = (int16_t)((buf[5] << 8) | buf[4]);
        const int16_t yaw   = (int16_t)((buf[7] << 8) | buf[6]);

        data->roll_f  = roll  / 100.0f;   /* 单位 0.01° */
        data->pitch_f = pitch / 100.0f;
        data->yaw_f   = yaw   / 100.0f;
        return 1;
    }
    return -1;  /* 持续噪声帧：放弃本轮，交给上层丢失计数处理 */
}

/*
 * IMU 帧解析回归测试（管道模拟串口，不依赖真实硬件/OpenCV）
 *
 * 覆盖：
 *   1. 标准 JY901 角度帧解析出精确 roll/pitch/yaw（防止字节偏移回归）
 *   2. 帧前噪声字节自动重同步
 *   3. 校验和错误的帧被跳过
 *   4. 非角度帧（如 0x51 加速度帧）被跳过
 *   5. 持续噪声帧时有界返回 -1（不死循环）
 *
 * 编译：gcc -std=gnu11 -Wall -Wextra -Idrivers \
 *          tests/test_imu_parse.c drivers/imu.c -o test_imu_parse
 */
#include "imu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "FAIL: %s\n", msg); } \
} while (0)

/* 构造 11 字节帧并追加校验和 */
static void build_frame(unsigned char out[11], unsigned char type,
                        int roll_cdeg, int pitch_cdeg, int yaw_cdeg) {
    out[0] = 0x55;
    out[1] = type;
    out[2] = (unsigned char)(roll_cdeg & 0xFF);
    out[3] = (unsigned char)((roll_cdeg >> 8) & 0xFF);
    out[4] = (unsigned char)(pitch_cdeg & 0xFF);
    out[5] = (unsigned char)((pitch_cdeg >> 8) & 0xFF);
    out[6] = (unsigned char)(yaw_cdeg & 0xFF);
    out[7] = (unsigned char)((yaw_cdeg >> 8) & 0xFF);
    out[8] = 0xC4; out[9] = 0x09;           /* 温度字段，解析器应忽略 */
    unsigned char sum = 0;
    for (int i = 0; i < 10; ++i) sum += out[i];
    out[10] = sum;
}

/* 把若干字节写入管道读端，返回读 fd；调用方负责 close */
static int feed(const unsigned char* data, int len) {
    int fds[2];
    if (pipe(fds) != 0) { perror("pipe"); _exit(2); }
    if (write(fds[1], data, len) != len) { perror("write"); _exit(2); }
    close(fds[1]);
    return fds[0];
}

static int nearly(float a, float b) { return fabsf(a - b) < 0.001f; }

static void test_exact_parse(void) {
    unsigned char frame[11];
    build_frame(frame, IMU_ANGLE_TYPE, 1000, -500, 18000); /* 10.00, -5.00, 180.00 */
    IMU_Data d;
    memset(&d, 0, sizeof(d));
    const int fd = feed(frame, 11);
    const int rc = imu_read_parse(fd, &d);
    close(fd);
    CHECK(rc == 1, "有效角度帧返回 1");
    CHECK(nearly(d.roll_f, 10.0f) && nearly(d.pitch_f, -5.0f) && nearly(d.yaw_f, 180.0f),
          "roll/pitch/yaw 字节偏移正确（10.00/-5.00/180.00）");
}

static void test_resync_after_garbage(void) {
    unsigned char buf[16];
    unsigned char frame[11];
    build_frame(frame, IMU_ANGLE_TYPE, 0, 0, 9000);        /* yaw = 90.00 */
    buf[0] = 0x00; buf[1] = 0x12; buf[2] = 0x54;           /* 帧前噪声（不含 0x55） */
    buf[3] = 0x01; buf[4] = 0xFF;
    memcpy(buf + 5, frame, 11);
    IMU_Data d;
    memset(&d, 0, sizeof(d));
    const int fd = feed(buf, 16);
    const int rc = imu_read_parse(fd, &d);
    close(fd);
    CHECK(rc == 1 && nearly(d.yaw_f, 90.0f), "帧前噪声后自动重同步到 0x55");
}

static void test_skip_bad_checksum_and_other_type(void) {
    unsigned char buf[22];
    unsigned char frame[11];
    build_frame(frame, IMU_ANGLE_TYPE, 0, 0, 1234);
    frame[10] ^= 0xFF;                                     /* 破坏校验和 */
    memcpy(buf, frame, 11);
    build_frame(frame, 0x51, 0, 0, 0);                     /* 加速度帧：应跳过 */
    memcpy(buf + 11, frame, 11);
    unsigned char good[11];
    build_frame(good, IMU_ANGLE_TYPE, 0, 0, 4500);         /* yaw = 45.00 */
    unsigned char all[33];
    memcpy(all, buf, 22);
    memcpy(all + 22, good, 11);
    IMU_Data d;
    memset(&d, 0, sizeof(d));
    const int fd = feed(all, 33);
    const int rc = imu_read_parse(fd, &d);
    close(fd);
    CHECK(rc == 1 && nearly(d.yaw_f, 45.0f), "跳过校验错误帧与非角度帧");
}

static void test_bounded_on_persistent_noise(void) {
    /* 8 个校验和全错的帧：达到 IMU_MAX_RESYNC 上限后必须返回 -1 而不是死循环 */
    unsigned char all[8 * 11];
    for (int i = 0; i < 8; ++i) {
        unsigned char frame[11];
        build_frame(frame, IMU_ANGLE_TYPE, i, i, i);
        frame[10] ^= 0xA5;
        memcpy(all + i * 11, frame, 11);
    }
    IMU_Data d;
    memset(&d, 0, sizeof(d));
    const int fd = feed(all, 8 * 11);
    const int rc = imu_read_parse(fd, &d);
    close(fd);
    CHECK(rc == -1, "持续噪声帧时有界失败（返回 -1）");
}

int main(void) {
    test_exact_parse();
    test_resync_after_garbage();
    test_skip_bad_checksum_and_other_type();
    test_bounded_on_persistent_noise();
    printf("IMU 解析测试结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

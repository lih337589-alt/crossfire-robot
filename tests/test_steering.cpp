/*
 * 控制层单元测试（纯逻辑，不依赖 OpenCV/硬件）
 *
 * 编译：g++ -std=c++17 -Wall -Wextra -Icontrol \
 *          tests/test_steering.cpp control/steering.cpp -o test_steering
 */
#include "steering.h"

#include <cstdio>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL: %s\n", msg); } \
} while (0)

static void test_normalize() {
    CHECK(normalize_angle(350.0f) == -10.0f, "350 → -10");
    CHECK(normalize_angle(-200.0f) == 160.0f, "-200 → 160");
    CHECK(normalize_angle(180.0f) == 180.0f, "180 不变");
    CHECK(normalize_angle(-180.0f) == -180.0f, "-180 不变");
}

static void test_heading_assist() {
    SteeringParams p;   /* 默认：deadzone=8, gain=6 */
    /* yaw 偏 +20°：base -= (20-8)*6 = 72 */
    CHECK(heading_assist(20.0f, 0.0f, 1455, p) == 1383, "偏+20° 补偿 -72");
    /* 死区内不补偿 */
    CHECK(heading_assist(5.0f, 0.0f, 1455, p) == 1455, "死区内不变");
    /* 偏 -20°：base += 72 */
    CHECK(heading_assist(-20.0f, 0.0f, 1455, p) == 1527, "偏-20° 补偿 +72");
}

static void test_compose_steer() {
    SteeringParams p;
    /* 无 PID 输出、无偏差 → 正中 */
    CHECK(compose_steer(0.0f, 0.0f, 0.0f, p) == 1455, "中立 1455");
    /* PID 输出 100, gain=-2 → 1455-200=1255 */
    CHECK(compose_steer(100.0f, 0.0f, 0.0f, p) == 1255, "PID 转向");
    /* 朝向辅助叠加 */
    CHECK(compose_steer(0.0f, 20.0f, 0.0f, p) == 1383, "朝向辅助叠加");
    /* 限幅：过大 → 2000，过小 → 1000 */
    CHECK(compose_steer(1000.0f, 0.0f, 0.0f, p) == 1000, "右转限幅 1000");
    CHECK(compose_steer(-1000.0f, 0.0f, 0.0f, p) == 2000, "左转限幅 2000");
}

static void test_compose_align() {
    SteeringParams p;
    /* 误差 10°, P=10 → 1455-100=1355 */
    CHECK(compose_align_steer(10.0f, 10.0f, p) == 1355, "回正转向");
    CHECK(compose_align_steer(100.0f, 10.0f, p) == 1000, "回正右限幅");
    CHECK(compose_align_steer(-100.0f, 10.0f, p) == 2000, "回正左限幅");
}

int main() {
    test_normalize();
    test_heading_assist();
    test_compose_steer();
    test_compose_align();

    std::printf("控制层测试结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

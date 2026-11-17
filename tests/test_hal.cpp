/*
 * HAL 层单元测试（用 Fake 桩实现，不碰真实硬件；用桩 OpenCV 头编译）
 *
 * 编译：g++ -std=c++17 -Wall -Wextra -Ihal -Itests/stub \
 *          tests/test_hal.cpp hal/hal_fake.cpp hal/voice_cmd.cpp -o test_hal
 */
#include "hal.h"
#include "hal_fake.h"
#include "voice_cmd.h"
#include "../control/drive_controller.h"
#include "../app/hardware_self_test.h"

#include <cstdio>
#include <string>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL: %s\n", msg); } \
} while (0)

static void test_hardware_self_test() {
    Config cfg;
    FakeCamera camera;
    FakeImu imu;
    FakePwm pwm;
    CHECK(run_hardware_self_test(cfg, camera, imu, pwm), "Fake HAL 全部就绪时硬件自检通过");
    CHECK(!camera.opened && !imu.opened && !pwm.initialized, "自检结束后释放摄像头、IMU 与 PWM");

    FakeCamera camera2;
    FakeImu imu2;
    FakePwm pwm2;
    pwm2.init_ok = false;
    CHECK(!run_hardware_self_test(cfg, camera2, imu2, pwm2), "PWM 未就绪时自检返回失败");
}

static void test_camera() {
    FakeCamera cam;
    CHECK(cam.open(2, 640, 480, 30), "FakeCamera open");
    CHECK(cam.opened && cam.opened_dev == 2 && cam.opened_w == 640, "open 参数");
    cv::Mat frame;
    CHECK(cam.grab(), "grab");
    CHECK(cam.retrieve(frame) && !frame.empty(), "retrieve 生成帧");
    cam.release();
    CHECK(!cam.opened, "release 关闭");
    CHECK(cam.open_path("/dev/video4", 1920, 1080, 30), "按设备路径打开 FakeCamera");
    CHECK(cam.opened_path == "/dev/video4" && cam.opened_w == 1920 && cam.opened_h == 1080, "摄像头路径与分辨率参数");
    cam.release();
}

static void test_imu() {
    FakeImu imu;
    CHECK(imu.open("/dev/ttyS0", 9600), "FakeImu open");
    imu.calibrate(8);
    CHECK(imu.calibrate_calls == 8, "calibrate 样本数");
    float yaw = 0;
    imu.yaw_value = 123.4f;
    CHECK(imu.read_yaw(yaw) && yaw == 123.4f, "read_yaw");
    imu.close();
    CHECK(!imu.opened, "close");
}

static void test_pwm() {
    FakePwm pwm;
    pwm.init();
    CHECK(pwm.initialized, "pwm init");
    pwm.write(0, 1455);
    pwm.write(1, 1670);
    CHECK(pwm.writes.size() == 2, "两次 write");
    CHECK(pwm.writes[0].channel == 0 && pwm.writes[0].us == 1455, "write 参数1");
    CHECK(pwm.writes[1].channel == 1 && pwm.writes[1].us == 1670, "write 参数2");
    pwm.deinit();
    CHECK(!pwm.initialized, "pwm deinit");
}

static void test_voice() {
    FakeVoice v;
    CHECK(v.speak("我将前往路线A"), "speak 成功");
    CHECK(v.speak_calls == 1 && v.last_text == "我将前往路线A", "记录文本");
}

static void test_display() {
    FakeDisplay d;
    CHECK(d.show_message("我将前往路线A"), "display message succeeds");
    d.show_status("计时开始");
    CHECK(d.message_calls == 1 && d.last_message == "我将前往路线A", "display announcement captured");
    CHECK(d.status_calls == 1 && d.last_status == "计时开始", "display status captured");
    CHECK(!d.show_message(""), "empty display message rejected");
}

static void test_syssvc() {
    FakeSysSvc s;
    s.stop("opi-control.service");
    s.start("ffmpeg-stream.service");
    CHECK(s.stopped.size() == 1 && s.stopped[0] == "opi-control.service", "stop 记录");
    CHECK(s.started.size() == 1 && s.started[0] == "ffmpeg-stream.service", "start 记录");
}

static void test_drive_controller() {
    FakePwm pwm;
    DriveParams p;
    p.motor_stop = 1600; p.servo_mid = 1455;
    p.motor_channel = 1; p.servo_channel = 0;
    p.motor_slew_us_per_cycle = 10; p.servo_slew_us_per_cycle = 20;
    DriveController drive(pwm, p);
    CHECK(drive.initialize(), "drive init confirms PWM readiness");
    drive.command(1670, 1600);
    CHECK(pwm.writes.back().channel == 0 && pwm.writes.back().us == 1475, "servo slew limit");
    CHECK(pwm.writes[pwm.writes.size()-2].channel == 1 && pwm.writes[pwm.writes.size()-2].us == 1610, "motor slew limit");
    drive.stop();
    CHECK(pwm.writes[pwm.writes.size()-2].us == 1600 && pwm.writes.back().us == 1455, "emergency stop bypasses slew");
    drive.shutdown();
    CHECK(!pwm.initialized, "drive shutdown releases PWM");

    FakePwm failed_pwm;
    failed_pwm.init_ok = false;
    DriveController failed_drive(failed_pwm, p);
    CHECK(!failed_drive.initialize(), "PWM not-ready inhibits drive initialization");
    CHECK(failed_pwm.writes.empty(), "no wheel commands sent when PWM init fails");
}

static void test_voice_command() {
    std::string c1 = build_voice_command("/root/A.mp3");
    CHECK(c1.find("mpg123") != std::string::npos, "mp3 走 mpg123");

    std::string c2 = build_voice_command("我将前往路线B");
    CHECK(c2.find("espeak") != std::string::npos, "文本走 espeak TTS");
    CHECK(c2.find("我将前往路线B") != std::string::npos, "TTS 文案内嵌");
}

int main() {
    test_hardware_self_test();
    test_camera();
    test_imu();
    test_pwm();
    test_voice();
    test_syssvc();
    test_display();
    test_drive_controller();
    test_voice_command();

    std::printf("HAL 测试结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

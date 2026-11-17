#include "hardware_self_test.h"

#include <cmath>
#include <cstdio>

bool run_hardware_self_test(const Config& cfg, ICamera& camera, IImu& imu, IPwm& pwm) {
    bool all_ok = true;
    bool camera_opened = false;
    if (!cfg.camera_device_path.empty()) camera_opened = camera.open_path(cfg.camera_device_path, cfg.cam_w, cfg.cam_h, cfg.cam_fps);
    else camera_opened = camera.open(cfg.cam_dev, cfg.cam_w, cfg.cam_h, cfg.cam_fps);
    if (!camera_opened) {
        std::printf("[FAIL] 摄像头：无法打开设备；检查 CAMERA_DEVICE_PATH/CAM_DEV、权限及连接\n");
        all_ok = false;
    } else {
        cv::Mat frame;
        const bool frame_ok = camera.grab() && camera.retrieve(frame) && !frame.empty();
        if (!frame_ok) {
            std::printf("[FAIL] 摄像头：设备已打开但无法读取有效帧；检查驱动、带宽与摄像头连接\n");
            all_ok = false;
        } else {
            std::printf("[INFO] 摄像头帧：%dx%d，channels=%d，type=%d\n", frame.cols, frame.rows,
                        frame.channels(), frame.type());
            if (frame.cols != cfg.cam_w || frame.rows != cfg.cam_h || frame.channels() != 3) {
                std::printf("[FAIL] 摄像头格式/尺寸不匹配：期望 %dx%d、3 通道 BGR；检查 UVC 输出格式及 CAM_W/CAM_H\n",
                            cfg.cam_w, cfg.cam_h);
                all_ok = false;
            } else {
                std::printf("[PASS] 摄像头：帧格式与配置尺寸匹配\n");
            }
        }
    }
    camera.release();

    if (!imu.open(cfg.imu_device, cfg.imu_baud)) {
        std::printf("[FAIL] IMU 串口：无法打开 %s @ %d baud；检查设备路径、权限、接线与波特率\n",
                    cfg.imu_device.c_str(), cfg.imu_baud);
        all_ok = false;
    } else {
        float yaw = 0.0f;
        if (!imu.read_yaw(yaw) || !std::isfinite(yaw)) {
            std::printf("[FAIL] IMU 串口：已打开但未收到有效航向帧；检查 IMU 供电、协议、波特率及 TX/RX 接线\n");
            all_ok = false;
        } else {
            std::printf("[PASS] IMU 串口：%s @ %d baud，航向=%.2f deg\n", cfg.imu_device.c_str(), cfg.imu_baud, yaw);
        }
        imu.close();
    }

    pwm.init();
    if (!pwm.ready()) {
        std::printf("[FAIL] PCA9685/PWM：初始化后设备未就绪；检查 I2C 总线、地址、供电及驱动日志\n");
        all_ok = false;
    } else {
        std::printf("[PASS] PCA9685/PWM：初始化就绪（未发送运动指令）\n");
    }
    pwm.deinit();
    std::printf("[INFO] 硬件自检汇总：%s\n", all_ok ? "全部通过" : "存在失败项，请按诊断修复后重试");
    return all_ok;
}

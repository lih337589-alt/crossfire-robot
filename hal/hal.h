#ifndef CROSSFIRE_HAL_H
#define CROSSFIRE_HAL_H

/*
 * HAL —— 硬件适配层接口契约。
 *
 * 设计目标：把散落在业务代码里的硬件访问（/dev/video2、/dev/ttyS0、/dev/i2c-5、
 * 系统服务与进程启动）集中到这一层。换摄像头/换 IMU 型号/换舵机总线，只改对应的
 * Real* 实现，业务代码零改动。每个接口都有 Real 实现（Linux）和 Fake 桩（测试/仿真）。
 */

#include <string>
#include <opencv2/opencv.hpp>

/* ---------- 摄像头 ---------- */
class ICamera {
public:
    virtual ~ICamera() = default;
    virtual bool open(int dev, int w, int h, int fps) = 0;
    // Preferred for UVC cameras because /dev/videoN enumeration can change between boots.
    // Empty paths fall back to the numeric device index in the application layer.
    virtual bool open_path(const std::string& path, int w, int h, int fps) {
        (void)path; (void)w; (void)h; (void)fps; return false;
    }
    virtual bool grab() = 0;
    virtual bool retrieve(cv::Mat& frame) = 0;
    virtual void release() = 0;
};

/* ---------- IMU（姿态/航向） ---------- */
class IImu {
public:
    virtual ~IImu() = default;
    virtual bool open(const std::string& dev, int baud) = 0;
    virtual void calibrate(int samples) = 0;
    virtual bool read_yaw(float& yaw) = 0;   /* 返回是否读到有效航向 */
    virtual void close() = 0;
};

/* ---------- PWM（舵机/电机/云台） ---------- */
class IPwm {
public:
    virtual ~IPwm() = default;
    virtual void init() = 0;
    virtual bool ready() const = 0;
    virtual void write(int channel, int us) = 0;
    virtual void deinit() = 0;
};

/* ---------- 语音播报 ---------- */
class IVoice {
public:
    virtual ~IVoice() = default;
    /* 播报文本（规则要求可核对的文案）；实现可用 TTS 合成或预录音频 */
    virtual bool speak(const std::string& text) = 0;
};


/* ---------- 状态显示/播报文本 ---------- */
class IDisplay {
public:
    virtual ~IDisplay() = default;
    virtual bool show_message(const std::string& text) = 0;
    virtual void show_status(const std::string& text) = 0;
};

/* ---------- 系统服务（opi-control/ffmpeg 等） ---------- */
class ISysSvc {
public:
    virtual ~ISysSvc() = default;
    virtual void stop(const std::string& service) = 0;
    virtual void start(const std::string& service) = 0;
};

#endif /* CROSSFIRE_HAL_H */

#ifndef CROSSFIRE_HAL_LINUX_H
#define CROSSFIRE_HAL_LINUX_H

#include "hal.h"
#include "../drivers/imu.h"

class RealCamera : public ICamera {
public:
    bool open(int dev, int w, int h, int fps) override;
    bool open_path(const std::string& path, int w, int h, int fps) override;
    bool grab() override;
    bool retrieve(cv::Mat& frame) override;
    void release() override;
private:
    cv::VideoCapture cap_;
};

class RealImu : public IImu {
public:
    bool open(const std::string& dev, int baud) override;
    void calibrate(int samples) override;
    bool read_yaw(float& yaw) override;
    void close() override;
private:
    int fd_ = -1;
    IMU_Data data_{};
};

class RealPwm : public IPwm {
public:
    void init() override;
    bool ready() const override;
    void write(int channel, int us) override;
    void deinit() override;
};

class RealVoice : public IVoice {
public:
    explicit RealVoice(int timeout_ms = 8000) : timeout_ms_(timeout_ms) {}
    void set_timeout_ms(int timeout_ms) { timeout_ms_ = timeout_ms < 100 ? 100 : timeout_ms; }
    bool speak(const std::string& text) override;
private:
    int timeout_ms_;
};

class ConsoleDisplay : public IDisplay {
public:
    bool show_message(const std::string& text) override;
    void show_status(const std::string& text) override;
};

class RealSysSvc : public ISysSvc {
public:
    void stop(const std::string& service) override;
    void start(const std::string& service) override;
};

#endif

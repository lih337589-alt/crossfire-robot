#ifndef CROSSFIRE_HAL_FAKE_H
#define CROSSFIRE_HAL_FAKE_H

/*
 * Fake 桩实现 —— 用于单元测试与仿真，不碰真实硬件。
 * 每个 Fake 记录最近一次操作，供测试断言。
 */

#include "hal.h"
#include <vector>

class FakeCamera : public ICamera {
public:
    bool open(int dev, int w, int h, int fps) override;
    bool open_path(const std::string& path, int w, int h, int fps) override;
    bool grab() override;
    bool retrieve(cv::Mat& frame) override;
    void release() override;

    bool opened = false;
    int  opened_dev = -1, opened_w = 0, opened_h = 0;
    std::string opened_path;
};

class FakeImu : public IImu {
public:
    bool open(const std::string& dev, int baud) override;
    void calibrate(int samples) override;
    bool read_yaw(float& yaw) override;
    void close() override;

    bool opened = false;
    int  calibrate_calls = 0;
    float yaw_value = 90.0f;   /* 测试可设 */
};

class FakePwm : public IPwm {
public:
    void init() override;
    bool ready() const override { return initialized; }
    void write(int channel, int us) override;
    void deinit() override;

    bool initialized = false;
    bool init_ok = true;
    struct Write { int channel; int us; };
    std::vector<Write> writes;
};

class FakeVoice : public IVoice {
public:
    bool speak(const std::string& text) override;
    std::string last_text;
    int speak_calls = 0;
};

class FakeDisplay : public IDisplay {
public:
    bool show_message(const std::string& text) override;
    void show_status(const std::string& text) override;
    std::string last_message;
    std::string last_status;
    int message_calls = 0, status_calls = 0;
};

class FakeSysSvc : public ISysSvc {
public:
    void stop(const std::string& service) override;
    void start(const std::string& service) override;
    std::vector<std::string> stopped;
    std::vector<std::string> started;
};

#endif /* CROSSFIRE_HAL_FAKE_H */

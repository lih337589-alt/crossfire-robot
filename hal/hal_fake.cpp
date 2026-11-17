#include "hal_fake.h"

bool FakeCamera::open(int dev, int w, int h, int) {
    opened = true;
    opened_dev = dev;
    opened_w = w;
    opened_h = h;
    return true;
}

bool FakeCamera::open_path(const std::string& path, int w, int h, int) {
    opened = !path.empty();
    opened_path = path;
    opened_dev = -1;
    opened_w = w;
    opened_h = h;
    return opened;
}

bool FakeCamera::grab() { return opened; }

bool FakeCamera::retrieve(cv::Mat& frame) {
    frame = cv::Mat(opened_h, opened_w, 16);   /* 生成 BGR 假帧 */
    return opened && !frame.empty();
}

void FakeCamera::release() { opened = false; }

bool FakeImu::open(const std::string&, int) { opened = true; return true; }
void FakeImu::calibrate(int samples) { calibrate_calls = samples; }
bool FakeImu::read_yaw(float& yaw) { yaw = yaw_value; return true; }
void FakeImu::close() { opened = false; }

void FakePwm::init() { initialized = init_ok; }
void FakePwm::write(int channel, int us) { writes.push_back({channel, us}); }
void FakePwm::deinit() { initialized = false; }

bool FakeVoice::speak(const std::string& text) {
    last_text = text;
    ++speak_calls;
    return true;
}

bool FakeDisplay::show_message(const std::string& text) {
    last_message = text; ++message_calls; return !text.empty();
}
void FakeDisplay::show_status(const std::string& text) { last_status = text; ++status_calls; }

void FakeSysSvc::stop(const std::string& service) { stopped.push_back(service); }
void FakeSysSvc::start(const std::string& service) { started.push_back(service); }

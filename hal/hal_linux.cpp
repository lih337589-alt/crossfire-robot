#include "hal_linux.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>
#include "../drivers/pca9685.h"

namespace {
speed_t baud_constant(int baud) {
    switch (baud) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
        default: return static_cast<speed_t>(baud); // compatibility with callers passing termios constants
    }
}

bool run_bounded(const std::vector<std::string>& args, int timeout_ms) {
    if (args.empty()) return false;
    const pid_t pid = fork();
    if (pid < 0) return false;
    if (pid == 0) {
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& s : args) argv.push_back(const_cast<char*>(s.c_str()));
        argv.push_back(nullptr);
        execvp(argv[0], argv.data());
        _exit(127);
    }
    const int bounded_timeout = std::max(100, timeout_ms);
    int elapsed_ms = 0;
    int status = 0;
    while (elapsed_ms <= bounded_timeout) {
        const pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) return WIFEXITED(status) && WEXITSTATUS(status) == 0;
        if (r < 0 && errno != EINTR) return false;
        usleep(10000);
        elapsed_ms += 10;
    }
    kill(pid, SIGTERM);
    usleep(50000);
    if (waitpid(pid, &status, WNOHANG) == 0) {
        kill(pid, SIGKILL);
        (void)waitpid(pid, &status, 0);
    }
    return false;
}

bool has_suffix_ci(const std::string& s, const char* suffix) {
    const size_t n = std::strlen(suffix);
    if (s.size() < n) return false;
    for (size_t i = 0; i < n; ++i) {
        if (std::tolower(static_cast<unsigned char>(s[s.size() - n + i])) != suffix[i]) return false;
    }
    return true;
}
bool safe_service_name(const std::string& s) {
    if (s.empty()) return false;
    for (unsigned char c : s) if (!(std::isalnum(c) || c == '.' || c == '_' || c == '-' || c == '@')) return false;
    return true;
}
} // namespace

bool RealCamera::open(int dev, int w, int h, int fps) {
    release();
    if (!cap_.open(dev, cv::CAP_V4L2)) return false;
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, w);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap_.set(cv::CAP_PROP_FPS, fps);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return cap_.isOpened();
}

bool RealCamera::open_path(const std::string& path, int w, int h, int fps) {
    release();
    if (path.empty() || !cap_.open(path, cv::CAP_V4L2)) return false;
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, w);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap_.set(cv::CAP_PROP_FPS, fps);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1); // minimize stale-frame latency
    return cap_.isOpened();
}
bool RealCamera::grab() { return cap_.isOpened() && cap_.grab(); }
bool RealCamera::retrieve(cv::Mat& frame) { return cap_.isOpened() && cap_.retrieve(frame) && !frame.empty(); }
void RealCamera::release() { if (cap_.isOpened()) cap_.release(); }

bool RealImu::open(const std::string& dev, int baud) {
    close();
    fd_ = imu_init(dev.c_str(), static_cast<int>(baud_constant(baud)));
    return fd_ >= 0;
}
void RealImu::calibrate(int samples) { if (fd_ >= 0 && samples > 0) imu_calibration(fd_, &data_, samples); }
bool RealImu::read_yaw(float& yaw) {
    if (fd_ < 0 || imu_read_parse(fd_, &data_) != 1) return false;
    yaw = data_.yaw_f;
    return true;
}
void RealImu::close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

void RealPwm::init() { pca9685_init(); }
bool RealPwm::ready() const { return pca9685_is_ready() != 0; }
void RealPwm::write(int channel, int us) { pca9685_write_us_safe(channel, us); }
void RealPwm::deinit() { pca9685_deinit(); }

bool RealVoice::speak(const std::string& text) {
    if (text.empty()) return false;
    const bool audio = has_suffix_ci(text, ".mp3") || has_suffix_ci(text, ".wav");
    if (audio) {
        if (run_bounded({"mpg123", "-q", "-f", "16384", text}, timeout_ms_)) return true;
        if (run_bounded({"ffplay", "-nodisp", "-autoexit", "-volume", "50", "-loglevel", "quiet", text}, timeout_ms_)) return true;
        return run_bounded({"cvlc", "--play-and-exit", "--gain", "0.5", "--intf", "dummy", text}, timeout_ms_);
    }
    return run_bounded({"espeak", "-v", "zh", "-s", "160", text}, timeout_ms_);
}

bool ConsoleDisplay::show_message(const std::string& text) {
    if (text.empty()) return false;
    std::printf("[display/announce] %s\n", text.c_str());
    return true;
}
void ConsoleDisplay::show_status(const std::string& text) {
    std::printf("[display] %s\n", text.c_str());
}

void RealSysSvc::stop(const std::string& service) {
    if (safe_service_name(service)) (void)run_bounded({"systemctl", "stop", service}, 3000);
}
void RealSysSvc::start(const std::string& service) {
    if (safe_service_name(service)) (void)run_bounded({"systemctl", "start", service}, 3000);
}

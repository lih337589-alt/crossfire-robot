#include <csignal>
#include <cstdio>
#include <unistd.h>

#include "../config/config.h"
#include "../hal/hal_linux.h"
#include "mission_controller.h"
#include "hardware_self_test.h"

namespace {
volatile sig_atomic_t g_running = 1;
void signal_handler(int) { g_running = 0; }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    const bool self_test = argc > 1 && std::string(argv[1]) == "--self-test";
    const char* config_path = self_test ? (argc > 2 ? argv[2] : "config/config.ini")
                                        : (argc > 1 ? argv[1] : "config/config.ini");
    Config cfg;
    if (!cfg.load(config_path)) {
        std::fprintf(stderr, "[fatal] failed to load valid config: %s\n", config_path);
        return 2;
    }
    std::string reason;
    if (!cfg.validate(&reason)) {
        std::fprintf(stderr, "[fatal] invalid config: %s\n", reason.c_str());
        return 2;
    }

    RealCamera camera;
    RealImu imu;
    RealPwm pwm;
    if (self_test) return run_hardware_self_test(cfg, camera, imu, pwm) ? 0 : 1;

    RealVoice voice(cfg.voice_timeout_ms);
    ConsoleDisplay display;
    MissionController app(cfg, camera, imu, pwm, voice, display);

    if (!app.initialize()) {
        app.shutdown();
        return 2;
    }

    // No keyboard, UDP remote-control, or interactive tuning listener is registered here.
    while (g_running && app.step()) usleep(static_cast<useconds_t>(cfg.loop_sleep_ms) * 1000);
    if (!g_running) app.abort();
    app.shutdown();
    return app.state() == State::FINISHED ? 0 : (app.state() == State::FAILED ? 1 : 0);
}

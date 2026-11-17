#ifndef CROSSFIRE_DRIVE_CONTROLLER_H
#define CROSSFIRE_DRIVE_CONTROLLER_H

#include "../hal/hal.h"

struct DriveParams {
    int motor_channel = 1, servo_channel = 0;
    int motor_stop = 1600, servo_mid = 1455;
    int motor_pulse_min_us = 1000, motor_pulse_max_us = 2500;
    int pwm_pulse_min_us = 500, pwm_pulse_max_us = 2500;
    int servo_left = 2000, servo_right = 1000;
    int motor_slew_us_per_cycle = 12, servo_slew_us_per_cycle = 40;
};

// The only layer allowed to issue wheel/steering PWM writes. Emergency stop bypasses slew limiting.
class DriveController {
public:
    DriveController(IPwm& pwm, const DriveParams& params);
    bool initialize();
    bool healthy() const { return initialized_ && pwm_.ready(); }
    void command(int motor_us, int servo_us);
    void stop();
    void write_aux(int channel, int pulse_us);
    void shutdown();
    bool initialized() const { return initialized_; }
private:
    static int step_toward(int current, int target, int max_step);
    IPwm& pwm_;
    DriveParams p_;
    int current_motor_;
    int current_servo_;
    bool initialized_ = false;
};

#endif

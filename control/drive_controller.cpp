#include "drive_controller.h"
#include <algorithm>
#include <cstdlib>

DriveController::DriveController(IPwm& pwm, const DriveParams& params)
    : pwm_(pwm), p_(params), current_motor_(params.motor_stop), current_servo_(params.servo_mid) {}

int DriveController::step_toward(int current, int target, int max_step) {
    max_step = std::max(1, max_step);
    if (target > current) return std::min(target, current + max_step);
    if (target < current) return std::max(target, current - max_step);
    return current;
}

bool DriveController::initialize() {
    pwm_.init();
    initialized_ = pwm_.ready();
    if (!initialized_) return false;
    current_motor_ = p_.motor_stop;
    current_servo_ = p_.servo_mid;
    pwm_.write(p_.motor_channel, p_.motor_stop);
    pwm_.write(p_.servo_channel, p_.servo_mid);
    return true;
}

void DriveController::command(int motor_us, int servo_us) {
    if (!initialized_) return;
    motor_us = std::clamp(motor_us, p_.motor_pulse_min_us, p_.motor_pulse_max_us);
    const int servo_lo = std::min(p_.servo_left, p_.servo_right);
    const int servo_hi = std::max(p_.servo_left, p_.servo_right);
    servo_us = std::clamp(servo_us, servo_lo, servo_hi);
    current_motor_ = step_toward(current_motor_, motor_us, p_.motor_slew_us_per_cycle);
    current_servo_ = step_toward(current_servo_, servo_us, p_.servo_slew_us_per_cycle);
    pwm_.write(p_.motor_channel, current_motor_);
    pwm_.write(p_.servo_channel, current_servo_);
}

void DriveController::stop() {
    if (!initialized_) return;
    current_motor_ = p_.motor_stop;
    current_servo_ = p_.servo_mid;
    pwm_.write(p_.motor_channel, p_.motor_stop);
    pwm_.write(p_.servo_channel, p_.servo_mid);
}

void DriveController::write_aux(int channel, int pulse_us) {
    if (!initialized_ || channel < 0 || channel > 15 || channel == p_.motor_channel || channel == p_.servo_channel) return;
    pwm_.write(channel, std::clamp(pulse_us, p_.pwm_pulse_min_us, p_.pwm_pulse_max_us));
}

void DriveController::shutdown() {
    if (!initialized_) return;
    stop();
    pwm_.deinit();
    initialized_ = false;
}

#ifndef CROSSFIRE_HARDWARE_SELF_TEST_H
#define CROSSFIRE_HARDWARE_SELF_TEST_H

#include "../config/config.h"
#include "../hal/hal.h"

// Performs read-only hardware checks. It never commands motor/servo movement.
bool run_hardware_self_test(const Config& cfg, ICamera& camera, IImu& imu, IPwm& pwm);

#endif

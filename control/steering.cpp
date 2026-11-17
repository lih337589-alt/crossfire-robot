#include "steering.h"
#include <algorithm>
#include <cmath>

float normalize_angle(float a) {
    if (!std::isfinite(a)) return 0.0f;
    a = std::fmod(a, 360.0f);
    if (a > 180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

static int clamp_steer(int steer, const SteeringParams& p) {
    const int lo = std::min(p.servo_left, p.servo_right);
    const int hi = std::max(p.servo_left, p.servo_right);
    return std::clamp(steer, lo, hi);
}

int heading_assist(float yaw, float target_heading, int base, const SteeringParams& p) {
    if (!std::isfinite(yaw) || !std::isfinite(target_heading)) return clamp_steer(base, p);
    const float hy = normalize_angle(yaw - target_heading);
    if (hy > p.head_deadzone) base -= static_cast<int>((hy - p.head_deadzone) * p.head_gain);
    else if (hy < -p.head_deadzone) base += static_cast<int>((-hy - p.head_deadzone) * p.head_gain);
    return clamp_steer(base, p);
}

int compose_steer(float pid_out, float yaw, float target_heading, const SteeringParams& p) {
    if (!std::isfinite(pid_out)) pid_out = 0.0f;
    int steer = p.servo_mid + static_cast<int>(pid_out * p.servo_gain);
    steer = heading_assist(yaw, target_heading, steer, p);
    return clamp_steer(steer, p);
}

int compose_align_steer(float angle_error, float steer_p_gain, const SteeringParams& p) {
    if (!std::isfinite(angle_error) || !std::isfinite(steer_p_gain)) return p.servo_mid;
    const int steer = p.servo_mid - static_cast<int>(angle_error * steer_p_gain);
    return clamp_steer(steer, p);
}

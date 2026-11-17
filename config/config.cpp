#include "config.h"

#include <cctype>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace {
std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}
int parse_int(const std::string& key, const std::string& raw, int current,
              int lo, int hi, bool& warned) {
    char* end = nullptr;
    const long n = std::strtol(raw.c_str(), &end, 10);
    if (end == raw.c_str() || *end != '\0') { warned = true; return current; }
    long v = n;
    if (v < lo) { v = lo; warned = true; }
    if (v > hi) { v = hi; warned = true; }
    (void)key;
    return static_cast<int>(v);
}
float parse_float(const std::string& key, const std::string& raw, float current,
                  float lo, float hi, bool& warned) {
    char* end = nullptr;
    const float n = std::strtof(raw.c_str(), &end);
    if (end == raw.c_str() || *end != '\0' || !std::isfinite(n)) {
        warned = true; return current;
    }
    float v = n;
    if (v < lo) { v = lo; warned = true; }
    if (v > hi) { v = hi; warned = true; }
    (void)key;
    return v;
}

// 已知配置项白名单（含 C() 解析器处理的 12 个 BLUE_/RED1_/RED2_ 阈值键）。
// 放在命名空间作用域，避免每次解析一行都重建整张表。
const char* const kKnownKeys[] = {
    "FIELD_LENGTH_M","FIELD_WIDTH_M","CAM_DEV","CAM_W","CAM_H","CAM_FPS","PIXEL_REF_W","PIXEL_REF_H","ROI_X1","ROI_Y1","ROI_X2","ROI_Y2",
    "BLUE_H_LOW","BLUE_H_HIGH","BLUE_S_LOW","BLUE_V_LOW","RED1_H_LOW","RED1_H_HIGH","RED1_S_LOW","RED1_V_LOW","RED2_H_LOW","RED2_H_HIGH","RED2_S_LOW","RED2_V_LOW",
    "MIN_AREA_BLUE","MIN_AREA_RED","MIN_CONT_W","MIN_CONT_H","CONE_EDGE_MARGIN_PX","CONE_SHAPE_MIN_HEIGHT","CONE_BOTTOM_BAND_MIN_PX","CONE_BOTTOM_BAND_MAX_PX","CONE_BOTTOM_BAND_HEIGHT_DIVISOR","CONE_BOTTOM_MIN_WIDTH","CONE_MID_MIN_WIDTH","CONE_BOTTOM_WIDTH_RATIO","CONE_MID_WIDTH_RATIO","ASPECT_MIN","ASPECT_MAX","FILL_MIN","FILL_MAX","TOP_EXCLUDE_GLOBAL_Y","ROW_TOLERANCE","MIN_GAP_WIDTH","MAX_GAP_WIDTH","RED_BLOCK_BELOW","RED_CORRIDOR_MARGIN","RED_MISS_CONFIRM_FRAMES","RED_TRACK_MATCH_RADIUS_PX","RED_UNCERTAIN_EDGE_PX","RED_CONFIDENCE_THRESHOLD","MORPH_KERNEL_SIZE","MORPHOLOGY_ITERATIONS",
    "MOTOR_PULSE_MIN_US","MOTOR_PULSE_MAX_US","PWM_PULSE_MIN_US","PWM_PULSE_MAX_US","SPEED_RUN","SPEED_APPROACH","SPEED_STOP","STATION_MOTOR_SPEED","ALIGN_MOTOR_SPEED","SERVO_MID","SERVO_LEFT","SERVO_RIGHT","MOTOR_CHANNEL","SERVO_CHANNEL","GIMBAL_YAW_CHANNEL","GIMBAL_PITCH_CHANNEL","GIMBAL_YAW_US","GIMBAL_PITCH_US","MOTOR_SLEW_US_PER_CYCLE","SERVO_SLEW_US_PER_CYCLE",
    "KP","KI","KD","OUT_MAX","OUT_MIN","ERR_INT_MAX","ERR_INT_MIN","SERVO_GAIN","COLOR_CARD_THRESHOLD","COLOR_STABLE_FRAMES","STOP_AREA_RATIO","STOP_CENTER_TOL","STOP_FRAMES_NEED","CAMP_MIN_RATIO","CAMP_MAX_RATIO","CAMP_SLOW_RATIO","CAMP_MIN_GLOBAL_Y","CAMP_LOST_GRACE_FRAMES","CAMP_CENTER_BYPASS_FACTOR",
    "BLUE_ROUTE","RED_ROUTE","TTS_A","TTS_B","VOICE_TIMEOUT_MS","HEAD_DEADZONE","HEAD_GAIN","HEAD_PROTECT","ALIGN_TOLERANCE_DEG","ALIGN_P_GAIN","ALIGN_STABLE_FRAMES",
    "BLUE_LEG1_SERVO","BLUE_LEG2_SERVO","BLUE_LEG1_MS","BLUE_LEG2_MS","BLUE_SINGLE_SERVO","BLUE_NO_SAFE_SERVO","BLUE_HEADING_OFFSET_DEG","RED_LEG1_SERVO","RED_LEG2_SERVO","RED_LEG1_MS","RED_LEG2_MS","RED_SINGLE_SERVO","RED_NO_SAFE_SERVO","RED_HEADING_OFFSET_DEG","STATION_STAY_MS",
    "RED_HIT_Y","RED_HIT_DX","BLUE_HIT_Y","BLUE_HIT_DX","CONTACT_EVENT_COOLDOWN_MS","FENCE_PROXY_ENABLED","FENCE_EDGE_STRIP_PX","FENCE_BOTTOM_BAND_PX","FENCE_EDGE_DARK_RATIO","FENCE_OPPOSITE_EDGE_MAX_RATIO","DARK_THRESHOLD_V","FRAME_BRIGHTNESS_MIN","FRAME_BRIGHTNESS_MAX","MAX_BRIGHTNESS_DELTA","BAD_FRAME_LIMIT","MAX_CAMERA_MISSES","CAMERA_REOPEN_ATTEMPTS","CAMERA_REOPEN_INTERVAL_MS","MAX_IMU_MISSES","MAX_NO_CONE_FRAMES","CONE_LOSS_STOP_FRAMES","SPEED_SEARCH",
    "STATION_LEG_TIMEOUT_MS","ALIGN_TIMEOUT_MS","TRACK_TIMEOUT_MS","MISSION_TIMEOUT_MS","LOOP_SLEEP_MS","CAMERA_DEVICE_PATH","PC_HOST","ORANGEPI_IP","SEND_PORT","RECV_PORT","IMU_DEVICE","IMU_BAUD","IMU_CALIBRATION_SAMPLES","EVENT_LOG_PATH","MODE","VOLTAGE_NOMINAL_V","VOLTAGE_MIN_V","VOLTAGE_MAX_V"
};
} // namespace

void Config::load_defaults() { *this = Config{}; }

bool Config::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "[config] 无法打开配置文件: %s\n", path.c_str());
        return false;
    }
    // Every successful load starts from a clean default profile. This also prevents
    // applying resolution scaling twice if the same Config instance loads another file.
    load_defaults();
    std::string line;
    int line_no = 0;
    while (std::getline(f, line)) {
        ++line_no;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        const std::string val = trim(line.substr(eq + 1));
        if (key.empty()) continue;
        bool warned = false, recognized = true;
        auto I = [&](const char* k, int& d, int lo, int hi) {
            if (key == k) d = parse_int(key, val, d, lo, hi, warned);
        };
        auto F = [&](const char* k, float& d, float lo, float hi) {
            if (key == k) d = parse_float(key, val, d, lo, hi, warned);
        };
        auto S = [&](const char* k, std::string& d) { if (key == k) d = val; };
        auto C = [&](const char* prefix, ColorThreshold& c) {
            const std::string p(prefix);
            if (key == p + "_H_LOW") c.h_low = parse_int(key, val, c.h_low, 0, 180, warned);
            else if (key == p + "_H_HIGH") c.h_high = parse_int(key, val, c.h_high, 0, 180, warned);
            else if (key == p + "_S_LOW") c.s_low = parse_int(key, val, c.s_low, 0, 255, warned);
            else if (key == p + "_V_LOW") c.v_low = parse_int(key, val, c.v_low, 0, 255, warned);
        };

        F("FIELD_LENGTH_M", field_length_m, 6.0f, 8.0f); F("FIELD_WIDTH_M", field_width_m, 3.0f, 5.0f);
        I("CAM_DEV", cam_dev, 0, 9); I("CAM_W", cam_w, 160, 3840); I("CAM_H", cam_h, 120, 2160); I("CAM_FPS", cam_fps, 1, 120);
        I("PIXEL_REF_W", pixel_ref_w, 160, 3840); I("PIXEL_REF_H", pixel_ref_h, 120, 2160);
        I("ROI_X1", roi_x1, 0, 3840); I("ROI_Y1", roi_y1, 0, 2160); I("ROI_X2", roi_x2, 1, 3840); I("ROI_Y2", roi_y2, 1, 2160);
        C("BLUE", blue); C("RED1", red1); C("RED2", red2);
        F("MIN_AREA_BLUE", min_area_blue, 0.0f, 100000.0f); F("MIN_AREA_RED", min_area_red, 0.0f, 100000.0f);
        I("MIN_CONT_W", min_cont_w, 1, 1000); I("MIN_CONT_H", min_cont_h, 1, 1000);
        I("CONE_EDGE_MARGIN_PX", cone_edge_margin_px, 0, 100); I("CONE_SHAPE_MIN_HEIGHT", cone_shape_min_height, 1, 1000);
        I("CONE_BOTTOM_BAND_MIN_PX", cone_bottom_band_min_px, 1, 100); I("CONE_BOTTOM_BAND_MAX_PX", cone_bottom_band_max_px, 1, 200); I("CONE_BOTTOM_BAND_HEIGHT_DIVISOR", cone_bottom_band_height_divisor, 1, 100);
        I("CONE_BOTTOM_MIN_WIDTH", cone_bottom_min_width, 1, 1000); I("CONE_MID_MIN_WIDTH", cone_mid_min_width, 1, 1000);
        F("CONE_BOTTOM_WIDTH_RATIO", cone_bottom_width_ratio, 0.0f, 10.0f); F("CONE_MID_WIDTH_RATIO", cone_mid_width_ratio, 0.0f, 10.0f);
        F("ASPECT_MIN", aspect_min, 0.01f, 10.0f); F("ASPECT_MAX", aspect_max, 0.01f, 10.0f);
        F("FILL_MIN", fill_min, 0.0f, 1.0f); F("FILL_MAX", fill_max, 0.0f, 1.0f); I("TOP_EXCLUDE_GLOBAL_Y", top_exclude_global_y, 0, 2160);
        I("ROW_TOLERANCE", row_tolerance, 1, 500); I("MIN_GAP_WIDTH", min_gap_width, 1, 2000); I("MAX_GAP_WIDTH", max_gap_width, 1, 3000);
        I("RED_BLOCK_BELOW", red_block_below, 0, 1000); I("RED_CORRIDOR_MARGIN", red_corridor_margin, 0, 500); I("RED_MISS_CONFIRM_FRAMES", red_miss_confirm_frames, 1, 100); I("RED_TRACK_MATCH_RADIUS_PX", red_track_match_radius_px, 0, 1000); I("RED_UNCERTAIN_EDGE_PX", red_uncertain_edge_px, 0, 1000); F("RED_CONFIDENCE_THRESHOLD", red_confidence_threshold, 0.0f, 1.0f);
        I("MORPH_KERNEL_SIZE", morphology_kernel_size, 1, 31); I("MORPHOLOGY_ITERATIONS", morphology_iterations, 0, 5);
        I("MOTOR_PULSE_MIN_US", motor_pulse_min_us, 500, 2500); I("MOTOR_PULSE_MAX_US", motor_pulse_max_us, 500, 2500); I("PWM_PULSE_MIN_US", pwm_pulse_min_us, 500, 2500); I("PWM_PULSE_MAX_US", pwm_pulse_max_us, 500, 2500);
        I("SPEED_RUN", speed_run, 1000, 2500); I("SPEED_APPROACH", speed_approach, 1000, 2500); I("SPEED_STOP", speed_stop, 1000, 2500); I("SPEED_SEARCH", speed_search, 1000, 2500);
        I("STATION_MOTOR_SPEED", station_motor_speed, 1000, 2500); I("ALIGN_MOTOR_SPEED", align_motor_speed, 1000, 2500);
        I("SERVO_MID", servo_mid, 500, 2500); I("SERVO_LEFT", servo_left, 500, 2500); I("SERVO_RIGHT", servo_right, 500, 2500);
        I("MOTOR_CHANNEL", motor_channel, 0, 15); I("SERVO_CHANNEL", servo_channel, 0, 15);
        I("GIMBAL_YAW_CHANNEL", gimbal_yaw_channel, 0, 15); I("GIMBAL_PITCH_CHANNEL", gimbal_pitch_channel, 0, 15);
        I("GIMBAL_YAW_US", gimbal_yaw_us, 500, 2500); I("GIMBAL_PITCH_US", gimbal_pitch_us, 500, 2500);
        I("MOTOR_SLEW_US_PER_CYCLE", motor_slew_us_per_cycle, 1, 500); I("SERVO_SLEW_US_PER_CYCLE", servo_slew_us_per_cycle, 1, 500);
        F("KP", kp, -100.0f, 100.0f); F("KI", ki, -100.0f, 100.0f); F("KD", kd, -100.0f, 100.0f);
        F("OUT_MAX", out_max, -10000.0f, 10000.0f); F("OUT_MIN", out_min, -10000.0f, 10000.0f);
        I("ERR_INT_MAX", err_int_max, -10000, 10000); I("ERR_INT_MIN", err_int_min, -10000, 10000); F("SERVO_GAIN", servo_gain, -100.0f, 100.0f);
        F("COLOR_CARD_THRESHOLD", color_card_threshold, 0.01f, 1.0f); I("COLOR_STABLE_FRAMES", color_stable_frames, 1, 30);
        F("STOP_AREA_RATIO", stop_area_ratio, 0.01f, 1.0f); I("STOP_CENTER_TOL", stop_center_tol, 0, 3840); I("STOP_FRAMES_NEED", stop_frames_need, 1, 100);
        F("CAMP_MIN_RATIO", camp_min_ratio, 0.0f, 1.0f); F("CAMP_MAX_RATIO", camp_max_ratio, 0.0f, 1.0f); F("CAMP_SLOW_RATIO", camp_slow_ratio, 0.0f, 1.0f);
        I("CAMP_MIN_GLOBAL_Y", camp_min_global_y, 0, 2160); I("CAMP_LOST_GRACE_FRAMES", camp_lost_grace_frames, 0, 1000); F("CAMP_CENTER_BYPASS_FACTOR", camp_center_bypass_factor, 1.0f, 3.0f);
        S("BLUE_ROUTE", blue_route); S("RED_ROUTE", red_route); S("TTS_A", tts_a); S("TTS_B", tts_b); I("VOICE_TIMEOUT_MS", voice_timeout_ms, 100, 120000);
        F("HEAD_DEADZONE", head_deadzone, 0.0f, 90.0f); F("HEAD_GAIN", head_gain, 0.0f, 100.0f); F("HEAD_PROTECT", head_protect, 0.0f, 180.0f);
        F("ALIGN_TOLERANCE_DEG", align_tolerance_deg, 0.1f, 30.0f); F("ALIGN_P_GAIN", align_p_gain, 0.0f, 100.0f); I("ALIGN_STABLE_FRAMES", align_stable_frames, 1, 100);
        I("BLUE_LEG1_SERVO", blue_leg1_servo, 500, 2500); I("BLUE_LEG2_SERVO", blue_leg2_servo, 500, 2500);
        I("BLUE_LEG1_MS", blue_leg1_ms, 0, 30000); I("BLUE_LEG2_MS", blue_leg2_ms, 0, 30000); I("BLUE_SINGLE_SERVO", blue_single_servo, 500, 2500); I("BLUE_NO_SAFE_SERVO", blue_no_safe_servo, 500, 2500); F("BLUE_HEADING_OFFSET_DEG", blue_heading_offset_deg, -180.0f, 180.0f);
        I("RED_LEG1_SERVO", red_leg1_servo, 500, 2500); I("RED_LEG2_SERVO", red_leg2_servo, 500, 2500);
        I("RED_LEG1_MS", red_leg1_ms, 0, 30000); I("RED_LEG2_MS", red_leg2_ms, 0, 30000); I("RED_SINGLE_SERVO", red_single_servo, 500, 2500); I("RED_NO_SAFE_SERVO", red_no_safe_servo, 500, 2500); F("RED_HEADING_OFFSET_DEG", red_heading_offset_deg, -180.0f, 180.0f);
        I("STATION_STAY_MS", station_stay_ms, 3000, 30000);
        I("RED_HIT_Y", red_hit_y, 0, 2160); I("RED_HIT_DX", red_hit_dx, 0, 3840); I("BLUE_HIT_Y", blue_hit_y, 0, 2160); I("BLUE_HIT_DX", blue_hit_dx, 0, 3840); I("CONTACT_EVENT_COOLDOWN_MS", contact_event_cooldown_ms, 0, 60000);
        I("FENCE_PROXY_ENABLED", fence_proxy_enabled, 0, 1); I("FENCE_EDGE_STRIP_PX", fence_edge_strip_px, 1, 1000); I("FENCE_BOTTOM_BAND_PX", fence_bottom_band_px, 1, 1000); F("FENCE_EDGE_DARK_RATIO", fence_edge_dark_ratio, 0.0f, 1.0f); F("FENCE_OPPOSITE_EDGE_MAX_RATIO", fence_opposite_edge_max_ratio, 0.0f, 1.0f); I("DARK_THRESHOLD_V", dark_threshold_v, 0, 255);
        F("FRAME_BRIGHTNESS_MIN", frame_brightness_min, 0.0f, 255.0f); F("FRAME_BRIGHTNESS_MAX", frame_brightness_max, 0.0f, 255.0f); F("MAX_BRIGHTNESS_DELTA", max_brightness_delta, 0.0f, 255.0f);
        I("BAD_FRAME_LIMIT", bad_frame_limit, 1, 1000); I("MAX_CAMERA_MISSES", max_camera_misses, 1, 1000); I("CAMERA_REOPEN_ATTEMPTS", camera_reopen_attempts, 0, 100); I("CAMERA_REOPEN_INTERVAL_MS", camera_reopen_interval_ms, 10, 60000); I("MAX_IMU_MISSES", max_imu_misses, 1, 1000); I("MAX_NO_CONE_FRAMES", max_no_cone_frames, 1, 10000); I("CONE_LOSS_STOP_FRAMES", cone_loss_stop_frames, 1, 1000);
        I("STATION_LEG_TIMEOUT_MS", station_leg_timeout_ms, 100, 120000); I("ALIGN_TIMEOUT_MS", align_timeout_ms, 100, 120000); I("TRACK_TIMEOUT_MS", track_timeout_ms, 100, 600000); I("MISSION_TIMEOUT_MS", mission_timeout_ms, 1000, 1800000); I("LOOP_SLEEP_MS", loop_sleep_ms, 1, 1000);
        S("CAMERA_DEVICE_PATH", camera_device_path); S("PC_HOST", pc_host); S("ORANGEPI_IP", orangepi_ip); I("SEND_PORT", send_port, 1, 65535); I("RECV_PORT", recv_port, 1, 65535); S("IMU_DEVICE", imu_device); I("IMU_BAUD", imu_baud, 1200, 230400); I("IMU_CALIBRATION_SAMPLES", imu_calibration_samples, 0, 1000); S("EVENT_LOG_PATH", event_log_path); S("MODE", mode); F("VOLTAGE_NOMINAL_V", voltage_nominal_v, 1.0f, 30.0f); F("VOLTAGE_MIN_V", voltage_min_v, 1.0f, 30.0f); F("VOLTAGE_MAX_V", voltage_max_v, 1.0f, 30.0f);

        // Reject silently misspelled options in the delivered configuration.
        // 白名单已包含 C() 解析器的全部 12 个阈值键；不再按前缀放行，
        // 否则 BLUE_H_LOWX 这类拼错键会被误当成合法配置而静默忽略。
        recognized = false;
        for (const char* k : kKnownKeys) if (key == k) { recognized = true; break; }
        if (!recognized) std::fprintf(stderr, "[config] 第 %d 行未知配置项: %s\n", line_no, key.c_str());
        if (warned) std::fprintf(stderr, "[config] 第 %d 行 %s=%s 非法或越界，已回退/钳位\n", line_no, key.c_str(), val.c_str());
    }
    // Pixel values in the INI are authored against PIXEL_REF_W x PIXEL_REF_H.
    // Scale coordinates and size thresholds to the configured processing resolution.
    // Ratios, HSV thresholds, timings and PWM pulses remain unchanged.
    const double sx = static_cast<double>(cam_w) / std::max(1, pixel_ref_w);
    const double sy = static_cast<double>(cam_h) / std::max(1, pixel_ref_h);
    const double sa = sx * sy;
    auto scale_x = [sx](int v) { return static_cast<int>(std::lround(v * sx)); };
    auto scale_y = [sy](int v) { return static_cast<int>(std::lround(v * sy)); };
    roi_x1 = scale_x(roi_x1); roi_x2 = scale_x(roi_x2);
    roi_y1 = scale_y(roi_y1); roi_y2 = scale_y(roi_y2);
    min_area_blue = static_cast<float>(min_area_blue * sa);
    min_area_red = static_cast<float>(min_area_red * sa);
    min_cont_w = std::max(1, scale_x(min_cont_w));
    min_cont_h = std::max(1, scale_y(min_cont_h));
    top_exclude_global_y = scale_y(top_exclude_global_y);
    cone_edge_margin_px = std::max(0, scale_x(cone_edge_margin_px));
    cone_shape_min_height = std::max(1, scale_y(cone_shape_min_height));
    cone_bottom_band_min_px = std::max(1, scale_y(cone_bottom_band_min_px));
    cone_bottom_band_max_px = std::max(cone_bottom_band_min_px, scale_y(cone_bottom_band_max_px));
    cone_bottom_min_width = std::max(1, scale_x(cone_bottom_min_width));
    cone_mid_min_width = std::max(1, scale_x(cone_mid_min_width));
    row_tolerance = std::max(1, scale_y(row_tolerance));
    min_gap_width = std::max(1, scale_x(min_gap_width));
    max_gap_width = std::max(min_gap_width, scale_x(max_gap_width));
    red_block_below = std::max(0, scale_y(red_block_below));
    red_corridor_margin = std::max(0, scale_x(red_corridor_margin));
    red_track_match_radius_px = std::max(0, scale_x(red_track_match_radius_px));
    red_uncertain_edge_px = std::max(0, scale_x(red_uncertain_edge_px));
    stop_center_tol = std::max(0, scale_x(stop_center_tol));
    camp_min_global_y = scale_y(camp_min_global_y);
    red_hit_y = scale_y(red_hit_y); red_hit_dx = std::max(0, scale_x(red_hit_dx));
    blue_hit_y = scale_y(blue_hit_y); blue_hit_dx = std::max(0, scale_x(blue_hit_dx));
    fence_edge_strip_px = std::max(1, scale_x(fence_edge_strip_px));
    fence_bottom_band_px = std::max(1, scale_y(fence_bottom_band_px));
    morphology_kernel_size = std::max(1, static_cast<int>(std::lround(morphology_kernel_size * std::sqrt(sa))));
    if ((morphology_kernel_size % 2) == 0) ++morphology_kernel_size;

    std::string reason;
    if (!validate(&reason)) {
        std::fprintf(stderr, "[config] 配置校验失败: %s\n", reason.c_str());
        return false;
    }
    return true;
}

bool Config::validate(std::string* reason) const {
    auto fail = [&](const char* msg) { if (reason) *reason = msg; return false; };
    if (field_length_m < 6.0f || field_length_m > 8.0f || field_width_m < 3.0f || field_width_m > 5.0f) return fail("场地尺寸必须在 6~8m × 3~5m 范围内");
    if (mode != "race" && mode != "debug" && mode != "bench") return fail("MODE 仅允许 race/debug/bench");
    if (cam_w <= 0 || cam_h <= 0 || cam_fps <= 0) return fail("摄像头分辨率和帧率必须为正数");
    if (pixel_ref_w <= 0 || pixel_ref_h <= 0) return fail("PIXEL_REF_W/H 必须大于 0");
    const auto valid_hue = [](const ColorThreshold& c) {
        return c.h_low >= 0 && c.h_high <= 180 && c.h_low <= c.h_high &&
               c.s_low >= 0 && c.s_low <= 255 && c.v_low >= 0 && c.v_low <= 255;
    };
    if (!valid_hue(blue) || !valid_hue(red1) || !valid_hue(red2))
        return fail("HSV 色相/饱和度/亮度阈值范围非法");
    const bool red_hue_overlap = std::max(red1.h_low, red2.h_low) <= std::min(red1.h_high, red2.h_high);
    if (red_hue_overlap) return fail("RED1 与 RED2 色相区间必须不重叠，避免红色占比重复计数");
    if (color_card_threshold <= 0.0f || color_card_threshold > 1.0f || color_stable_frames < 1)
        return fail("色卡占比阈值必须在 (0,1]，稳定帧数必须至少为 1");
    if (station_stay_ms < 3000) return fail("STATION_STAY_MS 不得少于 3000ms");
    if (roi_x1 < 0 || roi_y1 < 0 || roi_x2 <= roi_x1 || roi_y2 <= roi_y1 || roi_x2 > cam_w || roi_y2 > cam_h)
        return fail("缩放后的 ROI 必须为正面积且完全位于摄像头分辨率内");
    if (servo_left < servo_right) return fail("SERVO_LEFT 应大于等于 SERVO_RIGHT");
    if (out_min > out_max) return fail("OUT_MIN 不得大于 OUT_MAX");
    if (aspect_min > aspect_max || fill_min > fill_max) return fail("ASPECT/FILL 下限不得大于上限");
    if (cone_bottom_band_min_px > cone_bottom_band_max_px) return fail("锥桶底部带宽下限不得大于上限");
    if (frame_brightness_min >= frame_brightness_max) return fail("FRAME_BRIGHTNESS_MIN 必须小于 FRAME_BRIGHTNESS_MAX");
    if (min_gap_width > max_gap_width) return fail("MIN_GAP_WIDTH 不得大于 MAX_GAP_WIDTH");
    if (camp_min_ratio > camp_max_ratio) return fail("CAMP_MIN_RATIO 不得大于 CAMP_MAX_RATIO");
    if (blue_route.empty() || red_route.empty() || tts_a.empty() || tts_b.empty()) return fail("路线映射与播报文案不得为空");
    if (motor_channel == servo_channel) return fail("电机和转向舵机不得使用同一 PWM 通道");
    if (gimbal_yaw_channel == gimbal_pitch_channel || gimbal_yaw_channel == motor_channel ||
        gimbal_yaw_channel == servo_channel || gimbal_pitch_channel == motor_channel ||
        gimbal_pitch_channel == servo_channel) return fail("电机、转向和云台必须使用不同 PWM 通道");
    if (motor_pulse_min_us > motor_pulse_max_us || pwm_pulse_min_us > pwm_pulse_max_us) return fail("PWM 脉宽上下限配置错误");
    const auto motor_value_valid = [&](int v) { return v >= motor_pulse_min_us && v <= motor_pulse_max_us; };
    if (!motor_value_valid(speed_stop) || !motor_value_valid(speed_run) || !motor_value_valid(speed_approach) || !motor_value_valid(speed_search) ||
        !motor_value_valid(station_motor_speed) || !motor_value_valid(align_motor_speed)) return fail("电机速度脉宽必须在安全上下限内");
    if (!(speed_stop <= speed_search && speed_search < speed_approach)) return fail("SPEED_SEARCH 必须位于停车脉宽与接近速度脉宽之间，确保低速搜索");
    if (servo_mid < std::min(servo_left, servo_right) || servo_mid > std::max(servo_left, servo_right)) return fail("舵机中位必须位于舵机安全范围内");
    if (gimbal_yaw_us < pwm_pulse_min_us || gimbal_yaw_us > pwm_pulse_max_us ||
        gimbal_pitch_us < pwm_pulse_min_us || gimbal_pitch_us > pwm_pulse_max_us) return fail("云台脉宽超出 PWM 安全范围");
    if (voltage_min_v >= voltage_max_v || voltage_nominal_v < voltage_min_v || voltage_nominal_v > voltage_max_v) return fail("电压校正范围配置错误");
    const int max_leg_ms = std::max(blue_leg1_ms + blue_leg2_ms, red_leg1_ms + red_leg2_ms);
    const float max_voltage_scale = std::max(1.0f, voltage_nominal_v / voltage_min_v);
    if (station_leg_timeout_ms <= static_cast<int>(max_leg_ms * max_voltage_scale)) return fail("STATION_LEG_TIMEOUT_MS 必须大于动作序列最长时长（含电压缩放）");
    return true;
}

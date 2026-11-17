#ifndef CROSSFIRE_CONFIG_H
#define CROSSFIRE_CONFIG_H

#include <string>

struct ColorThreshold {
    int h_low, h_high, s_low, v_low;
};

struct Config {
    // Field geometry is configuration metadata; timed maneuvers must still be calibrated per field.
    float field_length_m = 7.0f; // legal range: 6..8 m
    float field_width_m = 4.0f;  // legal range: 3..5 m

    // Camera output size requested from UVC. Pixel parameters in config.ini are expressed
    // in the reference calibration coordinate system below and scaled during load().
    int cam_dev = 2, cam_w = 640, cam_h = 480, cam_fps = 30;
    int pixel_ref_w = 640, pixel_ref_h = 480;
    int roi_x1 = 20, roi_y1 = 180, roi_x2 = 620, roi_y2 = 440;

    ColorThreshold blue{95, 132, 100, 55};
    ColorThreshold red1{0, 10, 90, 65};
    ColorThreshold red2{170, 180, 90, 65};

    float min_area_blue = 25.0f, min_area_red = 30.0f;
    int min_cont_w = 7, min_cont_h = 6;
    float aspect_min = 0.2f, aspect_max = 1.8f;
    float fill_min = 0.08f, fill_max = 0.82f;
    int top_exclude_global_y = 230;
    int cone_edge_margin_px = 1;
    int cone_shape_min_height = 28;
    int cone_bottom_band_min_px = 3, cone_bottom_band_max_px = 14, cone_bottom_band_height_divisor = 5;
    int cone_bottom_min_width = 7, cone_mid_min_width = 5;
    float cone_bottom_width_ratio = 1.05f, cone_mid_width_ratio = 0.60f;

    int row_tolerance = 35;
    int min_gap_width = 45, max_gap_width = 200;
    int red_block_below = 40, red_corridor_margin = 10;
    int red_miss_confirm_frames = 3, red_track_match_radius_px = 45;
    int red_uncertain_edge_px = 12;
    float red_confidence_threshold = 0.25f;
    int morphology_kernel_size = 5, morphology_iterations = 1;

    int motor_pulse_min_us = 1000, motor_pulse_max_us = 2500;
    int pwm_pulse_min_us = 500, pwm_pulse_max_us = 2500;
    int speed_run = 1670, speed_approach = 1630, speed_stop = 1600;
    int station_motor_speed = 1660, align_motor_speed = 1640;
    int servo_mid = 1455, servo_left = 2000, servo_right = 1000;
    int motor_channel = 1, servo_channel = 0;
    int gimbal_yaw_channel = 2, gimbal_pitch_channel = 3;
    int gimbal_yaw_us = 1500, gimbal_pitch_us = 1350;
    int motor_slew_us_per_cycle = 12, servo_slew_us_per_cycle = 40;

    float kp = 0.6f, ki = 0.0f, kd = 0.1f;
    float out_max = 800.0f, out_min = -800.0f;
    int err_int_max = 30, err_int_min = -30;
    float servo_gain = -2.0f;

    // Color card selection: require a stable consecutive-frame vote.
    float color_card_threshold = 0.60f;
    int color_stable_frames = 3;

    float stop_area_ratio = 0.24f;
    int stop_center_tol = 30, stop_frames_need = 3;
    float camp_min_ratio = 0.06f, camp_max_ratio = 0.25f;
    float camp_slow_ratio = 0.12f;
    int camp_min_global_y = 250;
    int camp_lost_grace_frames = 5;
    float camp_center_bypass_factor = 1.20f;

    std::string blue_route = "A", red_route = "B";
    std::string tts_a = "我将前往路线A，到达据点A并穿越雷区返回营地A";
    std::string tts_b = "我将前往路线B，到达据点B并穿越雷区返回营地B";
    int voice_timeout_ms = 8000;

    float head_deadzone = 8.0f, head_gain = 6.0f, head_protect = 75.0f;
    float align_tolerance_deg = 3.0f;
    float align_p_gain = 10.0f;
    int align_stable_frames = 3;

    // Timed maneuver profile. These are calibrated parameters, not a localization system.
    int blue_leg1_servo = 1700, blue_leg2_servo = 1300;
    int blue_leg1_ms = 1700, blue_leg2_ms = 500;
    int blue_single_servo = 1520, blue_no_safe_servo = 1480;
    float blue_heading_offset_deg = -20.0f;
    int red_leg1_servo = 1300, red_leg2_servo = 1700;
    int red_leg1_ms = 1700, red_leg2_ms = 500;
    int red_single_servo = 1430, red_no_safe_servo = 1520;
    float red_heading_offset_deg = 13.0f;
    int station_stay_ms = 3000;

    int red_hit_y = 380, red_hit_dx = 40;
    int blue_hit_y = 380, blue_hit_dx = 40;
    int contact_event_cooldown_ms = 1000;
    int fence_proxy_enabled = 1;
    int fence_edge_strip_px = 18, fence_bottom_band_px = 24;
    float fence_edge_dark_ratio = 0.90f;
    float fence_opposite_edge_max_ratio = 0.65f;
    int dark_threshold_v = 70;

    float frame_brightness_min = 8.0f, frame_brightness_max = 248.0f;
    float max_brightness_delta = 75.0f;
    int bad_frame_limit = 8;
    int max_camera_misses = 3, camera_reopen_attempts = 3;
    int camera_reopen_interval_ms = 500;
    int max_imu_misses = 5;
    int max_no_cone_frames = 40; // legacy compatibility
    int cone_loss_stop_frames = 4;
    int speed_search = 1615; // slower than approach; calibrated PWM pulse

    int station_leg_timeout_ms = 10000;
    int align_timeout_ms = 4000;
    int track_timeout_ms = 60000;
    int mission_timeout_ms = 180000;
    int loop_sleep_ms = 10;

    std::string camera_device_path; // preferred V4L2 device path; empty means use CAM_DEV index
    std::string pc_host = "192.168.137.1", orangepi_ip = "192.168.137.86";
    int send_port = 8766, recv_port = 8767; // retained for compatibility; no UDP listener is registered in this app
    std::string imu_device = "/dev/ttyS0";
    int imu_baud = 9600, imu_calibration_samples = 8;
    float voltage_nominal_v = 12.6f, voltage_min_v = 9.0f, voltage_max_v = 13.5f;
    std::string event_log_path = "logs/crossfire_events.csv";
    std::string mode = "race"; // race/debug/bench; none of these modes enables manual control

    bool load(const std::string& path);
    void load_defaults();
    bool validate(std::string* reason = nullptr) const;

    int roi_w() const { return roi_x2 - roi_x1; }
    int roi_h() const { return roi_y2 - roi_y1; }
    int img_center() const { return roi_x1 + roi_w() / 2; }
    bool is_race_mode() const { return mode == "race"; }
    bool is_debug_mode() const { return mode == "debug"; }
};

#endif

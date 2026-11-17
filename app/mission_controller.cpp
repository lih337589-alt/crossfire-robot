#include "mission_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <cstdint>
#include <climits>
#include <time.h>
#include <unistd.h>

namespace {
ColorRatioParams color_params(const Config& c) {
    ColorRatioParams p;
    p.blue = c.blue; p.red1 = c.red1; p.red2 = c.red2;
    return p;
}

// Read once at initialize(): the environment is fixed at exec time, so there is no
// need to rescan environ on every control-loop iteration.
float voltage_scale_from_env(const Config& c) {
    const char* env = std::getenv("VOLT");
    if (!env) return 1.0f;
    char* end = nullptr;
    const float v = std::strtof(env, &end);
    if (end == env || *end != '\0' || !std::isfinite(v) || v < c.voltage_min_v || v > c.voltage_max_v) return 1.0f;
    return c.voltage_nominal_v / v; // retained compatibility with the supplied run.sh convention
}
}

int64_t MissionController::now_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}

DriveParams MissionController::make_drive_params(const Config& c) {
    DriveParams p;
    p.motor_channel = c.motor_channel; p.servo_channel = c.servo_channel;
    p.motor_stop = c.speed_stop; p.servo_mid = c.servo_mid;
    p.motor_pulse_min_us = c.motor_pulse_min_us; p.motor_pulse_max_us = c.motor_pulse_max_us;
    p.pwm_pulse_min_us = c.pwm_pulse_min_us; p.pwm_pulse_max_us = c.pwm_pulse_max_us;
    p.servo_left = c.servo_left; p.servo_right = c.servo_right;
    p.motor_slew_us_per_cycle = c.motor_slew_us_per_cycle;
    p.servo_slew_us_per_cycle = c.servo_slew_us_per_cycle;
    return p;
}

MissionController::MissionController(const Config& cfg, ICamera& camera, IImu& imu, IPwm& pwm, IVoice& voice, IDisplay& display)
    : cfg_(cfg), camera_(camera), imu_(imu), voice_(voice), display_(display), drive_(pwm, make_drive_params(cfg)),
      blue_detector_(make_cone_params()), red_detector_(make_cone_params()) {
    blue_detector_.set_output_offset(cfg_.roi_x1, cfg_.roi_y1);
    red_detector_.set_output_offset(cfg_.roi_x1, cfg_.roi_y1);
    ConeDetectParams red_params = make_cone_params();
    red_params.suppress_large_blob = false; // never suppress a large red near-cone from the safety path
    red_detector_.set_params(red_params);
}

ConeDetectParams MissionController::make_cone_params() const {
    ConeDetectParams p;
    p.min_cont_w = cfg_.min_cont_w; p.min_cont_h = cfg_.min_cont_h;
    p.aspect_min = cfg_.aspect_min; p.aspect_max = cfg_.aspect_max;
    p.fill_min = cfg_.fill_min; p.fill_max = cfg_.fill_max;
    p.stop_area_ratio = cfg_.stop_area_ratio;
    p.roi_x1 = cfg_.roi_x1; p.roi_y1 = cfg_.roi_y1;
    p.roi_w = cfg_.roi_w(); p.roi_h = cfg_.roi_h();
    p.top_exclude_global_y = cfg_.top_exclude_global_y;
    p.edge_margin_px = cfg_.cone_edge_margin_px; p.shape_min_height = cfg_.cone_shape_min_height;
    p.bottom_band_min_px = cfg_.cone_bottom_band_min_px; p.bottom_band_max_px = cfg_.cone_bottom_band_max_px;
    p.bottom_band_height_divisor = cfg_.cone_bottom_band_height_divisor;
    p.bottom_min_width = cfg_.cone_bottom_min_width; p.mid_min_width = cfg_.cone_mid_min_width;
    p.bottom_width_ratio = cfg_.cone_bottom_width_ratio; p.mid_width_ratio = cfg_.cone_mid_width_ratio;
    return p;
}

SteeringParams MissionController::make_steering_params() const {
    SteeringParams p;
    p.servo_mid = cfg_.servo_mid; p.servo_left = cfg_.servo_left; p.servo_right = cfg_.servo_right;
    p.servo_gain = cfg_.servo_gain; p.head_deadzone = cfg_.head_deadzone; p.head_gain = cfg_.head_gain;
    return p;
}

VisionPipelineParams MissionController::make_vision_params() const {
    VisionPipelineParams p;
    p.colors = color_params(cfg_);
    p.morphology_kernel_size = cfg_.morphology_kernel_size;
    p.morphology_iterations = cfg_.morphology_iterations;
    p.dark_threshold_v = cfg_.dark_threshold_v;
    return p;
}

MissionController::ActionSeq MissionController::action_seq() const {
    if (route_is_blue_) return {cfg_.blue_leg1_servo, cfg_.blue_leg2_servo,
        cfg_.blue_leg1_ms, cfg_.blue_leg2_ms, cfg_.blue_single_servo,
        cfg_.blue_no_safe_servo, cfg_.blue_heading_offset_deg};
    return {cfg_.red_leg1_servo, cfg_.red_leg2_servo,
        cfg_.red_leg1_ms, cfg_.red_leg2_ms, cfg_.red_single_servo,
        cfg_.red_no_safe_servo, cfg_.red_heading_offset_deg};
}

float MissionController::voltage_coefficient() const {
    return voltage_scale_; // cached at initialize(); see voltage_scale_from_env()
}

bool MissionController::initialize() {
    std::string reason;
    if (!cfg_.validate(&reason)) {
        std::fprintf(stderr, "[config] refuse to run: %s\n", reason.c_str());
        return false;
    }
    fsm_.set_min_station_stay_ms(cfg_.station_stay_ms);
    vision_.configure(make_vision_params());
    voltage_scale_ = voltage_scale_from_env(cfg_);
    blue_detector_.set_params(make_cone_params());
    ConeDetectParams red_params = make_cone_params();
    red_params.suppress_large_blob = false;
    red_detector_.set_params(red_params);
    blue_detector_.set_output_offset(cfg_.roi_x1, cfg_.roi_y1);
    red_detector_.set_output_offset(cfg_.roi_x1, cfg_.roi_y1);

    if (!drive_.initialize()) {
        fsm_.on_event(EventType::SENSOR_FAILURE, now_ms());
        std::fprintf(stderr, "[fail-safe] PWM controller initialization failed; refusing to launch\n");
        return false;
    }
    drive_.write_aux(cfg_.gimbal_yaw_channel, cfg_.gimbal_yaw_us);
    drive_.write_aux(cfg_.gimbal_pitch_channel, cfg_.gimbal_pitch_us);

    PID_Init(&pid_);
    pid_.Kp = cfg_.kp; pid_.Ki = cfg_.ki; pid_.Kd = cfg_.kd;
    pid_.OutMax = cfg_.out_max; pid_.OutMin = cfg_.out_min;
    pid_.ErrorIntMax = cfg_.err_int_max; pid_.ErrorIntMin = cfg_.err_int_min;

    if (imu_.open(cfg_.imu_device, cfg_.imu_baud)) {
        if (cfg_.imu_calibration_samples > 0) imu_.calibrate(cfg_.imu_calibration_samples);
        imu_ready_ = imu_.read_yaw(initial_yaw_);
        if (!imu_ready_) {
            std::fprintf(stderr, "[warn] IMU opened but initial yaw unavailable; alignment will use safe degraded path\n");
            imu_.close();
        }
    } else {
        std::fprintf(stderr, "[warn] IMU unavailable; vision steering remains active, heading assist disabled\n");
    }

    if (!open_camera()) {
        std::fprintf(stderr, "[error] camera open failed: path='%s' index=%d\n", cfg_.camera_device_path.c_str(), cfg_.cam_dev);
        fsm_.on_event(EventType::SENSOR_FAILURE, now_ms());
        drive_.stop();
        return false;
    }
    initialized_ = true;
    last_state_ = State::IDLE;
    has_last_state_ = false;
    char boot_message[160];
    std::snprintf(boot_message, sizeof(boot_message), "mode=%s field=%.1fm x %.1fm ROI=%dx%d", cfg_.mode.c_str(),
                  cfg_.field_length_m, cfg_.field_width_m, cfg_.roi_w(), cfg_.roi_h());
    display_.show_status(boot_message);
    if (cfg_.is_race_mode()) std::puts("[mode] race: keyboard/remote/manual-control entry points are not registered");
    return true;
}

int64_t MissionController::elapsed_ms() const { return fsm_.elapsed_ms(now_ms()); }

void MissionController::update_state_entry(int64_t now) {
    const State current = fsm_.state();
    if (has_last_state_ && current == last_state_) return;
    if (cfg_.is_debug_mode()) std::printf("[fsm] %s -> %s\n", has_last_state_ ? StateMachine::state_name(last_state_) : "BOOT", StateMachine::state_name(current));
    last_state_ = current;
    has_last_state_ = true;
    state_enter_ms_ = now;
    // New control context: drop stale integral/derivative history so the first PID
    // update in the new state does not inherit the previous state's error (derivative kick).
    pid_.Error = 0.0f; pid_.ErrorLast = 0.0f; pid_.ErrorInt = 0.0f;
    if (current == State::CROSSING) track_start_ms_ = now;
    if (current == State::TO_CAMP) { camp_lost_frames_ = 0; stop_frames_ = 0; }
    if (current == State::CROSSING) { no_cone_frames_ = 0; red_tracker_.reset(); }
    if (current == State::ALIGN) { align_stable_frames_ = 0; align_imu_misses_ = 0; target_heading_ = normalize_angle(initial_yaw_ + action_seq().heading_offset_deg); }
    last_brightness_ = -1.0;
    bad_frame_count_ = 0;
    vision_degraded_latched_ = false;
}

bool MissionController::open_camera() {
    if (!cfg_.camera_device_path.empty())
        return camera_.open_path(cfg_.camera_device_path, cfg_.cam_w, cfg_.cam_h, cfg_.cam_fps);
    return camera_.open(cfg_.cam_dev, cfg_.cam_w, cfg_.cam_h, cfg_.cam_fps);
}

bool MissionController::acquire_frame(int64_t now) {
    if (camera_.grab() && camera_.retrieve(capture_frame_) && !capture_frame_.empty()) {
        // Some UVC devices ignore CAP_PROP_FRAME_* and return their native mode.
        // Normalize to the configured processing size so ROI/safety thresholds stay valid.
        if (capture_frame_.cols == cfg_.cam_w && capture_frame_.rows == cfg_.cam_h) {
            frame_ = capture_frame_; // shallow Mat header copy; no pixel copy
        } else {
            const bool downscale = capture_frame_.cols >= cfg_.cam_w && capture_frame_.rows >= cfg_.cam_h;
            const int interpolation = downscale ? cv::INTER_AREA : cv::INTER_LINEAR;
            cv::resize(capture_frame_, frame_, cv::Size(cfg_.cam_w, cfg_.cam_h), 0.0, 0.0, interpolation);
        }
        camera_misses_ = 0;
        camera_reopen_count_ = 0;
        return !frame_.empty();
    }
    capture_frame_.release();
    frame_.release();
    drive_.stop();
    ++camera_misses_;
    if (camera_misses_ < cfg_.max_camera_misses) return false;
    if (now - last_camera_reopen_ms_ < cfg_.camera_reopen_interval_ms) return false;
    last_camera_reopen_ms_ = now;
    if (camera_reopen_count_ >= cfg_.camera_reopen_attempts) {
        fsm_.on_event(EventType::SENSOR_FAILURE, now);
        std::fprintf(stderr, "[fail-safe] camera unavailable after %d reopen attempts\n", camera_reopen_count_);
        drive_.stop();
        return false;
    }
    ++camera_reopen_count_;
    camera_.release();
    if (open_camera()) {
        std::fprintf(stderr, "[camera] reopen attempt %d/%d succeeded; waiting for valid frame\n", camera_reopen_count_, cfg_.camera_reopen_attempts);
    } else {
        std::fprintf(stderr, "[camera] reopen attempt %d/%d failed\n", camera_reopen_count_, cfg_.camera_reopen_attempts);
    }
    return false;
}

bool MissionController::frame_quality_ok(int64_t now) {
    const double b = vision_.brightness();
    bool bad = b < cfg_.frame_brightness_min || b > cfg_.frame_brightness_max;
    if (last_brightness_ >= 0.0 && std::abs(b - last_brightness_) > cfg_.max_brightness_delta) bad = true;
    last_brightness_ = b;
    if (bad) {
        ++bad_frame_count_;
        if (!vision_degraded_latched_) {
            fsm_.on_event(EventType::VISION_DEGRADED, now);
            vision_degraded_latched_ = true;
            std::fprintf(stderr, "[vision] brightness anomaly %.1f; vehicle held stopped\n", b);
        }
        drive_.stop();
        if (bad_frame_count_ >= cfg_.bad_frame_limit) {
            fsm_.on_event(EventType::SENSOR_FAILURE, now);
            std::fprintf(stderr, "[fail-safe] repeated unusable frames; mission failed\n");
        }
        return false;
    }
    bad_frame_count_ = 0;
    vision_degraded_latched_ = false;
    return true;
}

bool MissionController::check_red_hit(const std::vector<Cone>& reds) const {
    const int64_t center = cfg_.img_center();
    for (const auto& r : reds) {
        const int64_t dx = static_cast<int64_t>(r.bottom_x) - center;
        const int64_t abs_dx = dx < 0 ? -dx : dx;
        if (r.bottom_y >= cfg_.red_hit_y && abs_dx <= cfg_.red_hit_dx) return true;
    }
    return false;
}
bool MissionController::check_blue_hit(const std::vector<Cone>& blues) const {
    const int64_t center = cfg_.img_center();
    for (const auto& b : blues) {
        const int64_t dx = static_cast<int64_t>(b.bottom_x) - center;
        const int64_t abs_dx = dx < 0 ? -dx : dx;
        if (b.bottom_y >= cfg_.blue_hit_y && abs_dx <= cfg_.blue_hit_dx) return true;
    }
    return false;
}

bool MissionController::check_proximity(int64_t now, const std::vector<Cone>& blues,
                                        const std::vector<Cone>& reds) {
    if (check_red_hit(reds)) {
        fsm_.on_event(EventType::RED_CONE_HIT, now);
        drive_.stop();
        std::fprintf(stderr, "[FAILED] red-cone near-contact proxy triggered; immediate safe stop\n");
        return false;
    }
    const bool blue_risk = check_blue_hit(blues);
    if (blue_risk) {
        if (!blue_hit_latched_ && now - last_blue_event_ms_ >= cfg_.contact_event_cooldown_ms) {
            fsm_.on_event(EventType::BLUE_CONE_HIT, now);
            last_blue_event_ms_ = now;
            std::fprintf(stderr, "[penalty-proxy] blue cone near-contact: +5s\n");
        }
        blue_hit_latched_ = true;
    } else blue_hit_latched_ = false;

    const cv::Rect roi(cfg_.roi_x1, cfg_.roi_y1, cfg_.roi_w(), cfg_.roi_h());
    const bool fence_risk = cfg_.fence_proxy_enabled && vision_.fence_contact_proxy(
        roi, cfg_.fence_edge_strip_px, cfg_.fence_bottom_band_px, cfg_.fence_edge_dark_ratio, cfg_.fence_opposite_edge_max_ratio);
    if (fence_risk) {
        if (!fence_hit_latched_ && now - last_fence_event_ms_ >= cfg_.contact_event_cooldown_ms) {
            fsm_.on_event(EventType::FENCE_HIT, now);
            last_fence_event_ms_ = now;
            std::fprintf(stderr, "[penalty-proxy] dark boundary near ROI edge: +5s; hold stopped until clear\n");
        }
        fence_hit_latched_ = true;
        drive_.stop();
        return false;
    }
    fence_hit_latched_ = false;
    return true;
}

bool MissionController::read_yaw(float& yaw, int64_t now, bool fail_on_repeated_miss) {
    if (!imu_ready_) return false;
    if (imu_.read_yaw(yaw) && std::isfinite(yaw)) {
        runtime_imu_misses_ = 0;
        return true;
    }
    ++runtime_imu_misses_;
    if (fail_on_repeated_miss) {
        ++align_imu_misses_;
        drive_.stop();
        if (align_imu_misses_ >= cfg_.max_imu_misses) {
            fsm_.on_event(EventType::SENSOR_FAILURE, now);
            std::fprintf(stderr, "[fail-safe] IMU read timeout during alignment\n");
        }
    } else if (runtime_imu_misses_ >= cfg_.max_imu_misses && !imu_degraded_logged_) {
        fsm_.on_event(EventType::IMU_DEGRADED, now);
        imu_degraded_logged_ = true;
        std::fprintf(stderr, "[imu] repeated read failures; heading correction disabled, vision-only fallback\n");
    }
    return false;
}

void MissionController::process_crossing(int64_t now, const std::vector<Cone>& blues,
        const std::vector<Cone>& reds, const cv::Mat& camp_mask) {
    const std::vector<Cone> safe_reds = red_tracker_.update(reds, cfg_.red_miss_confirm_frames,
        cfg_.red_track_match_radius_px, cfg_.red_confidence_threshold);
    if (blues.empty()) {
        ++no_cone_frames_;
        CampDetectParams cp; cp.min_ratio = cfg_.camp_min_ratio; cp.roi_x1 = cfg_.roi_x1; cp.roi_y1 = cfg_.roi_y1;
        const CampResult camp = camp_detector_.detect(camp_mask, cp);
        const bool camp_near = camp.found && camp.center_y >= cfg_.camp_min_global_y;
        if (camp_near) {
            fsm_.on_event(EventType::CAMP_SEEN, now);
            drive_.stop();
            return;
        }
        const ConeLossAction loss_action = decide_cone_loss_action(no_cone_frames_, cfg_.cone_loss_stop_frames);
        if (loss_action == ConeLossAction::STOP_AND_WAIT) {
            drive_.stop();
            if (no_cone_frames_ == cfg_.cone_loss_stop_frames)
                std::fprintf(stderr, "[fail-safe] crossing cones missing for %d frames; stopped pending visual recovery\n", no_cone_frames_);
        } else {
            // Never continue at approach speed when lane markers are unavailable.
            drive_.command(cfg_.speed_search, cfg_.servo_mid);
        }
        return;
    }
    no_cone_frames_ = 0;
    const ActionSeq a = action_seq();
    if (blues.size() == 1) {
        drive_.command(cfg_.speed_approach, a.single_servo);
        return;
    }
    const size_t row_count = row_grouper_.group(blues, cfg_.row_tolerance);
    int target_x = -1;
    const int image_center = cfg_.img_center();
    for (size_t row_index = 0; row_index < row_count; ++row_index) {
        const auto& row = row_grouper_.row(row_index);
        if (row.size() < 2) continue;

        int64_t y_sum = 0;
        for (const Cone* cone : row) y_sum += cone->bottom_y;
        const int row_y = static_cast<int>(y_sum / static_cast<int64_t>(row.size()));

        // Stream candidates instead of allocating a temporary vector<Gap> on every row/frame.
        // Tie-breaking matches choose_gap_center(..., "center"): nearest image center, then wider gap.
        int row_target_x = -1;
        int row_best_width = -1;
        int64_t row_best_distance = INT64_MAX;
        for (size_t i = 0; i + 1 < row.size(); ++i) {
            const int left_x = row[i]->bottom_x;
            const int right_x = row[i + 1]->bottom_x;
            const int64_t width64 = static_cast<int64_t>(right_x) - left_x;
            if (width64 < cfg_.min_gap_width || width64 > cfg_.max_gap_width) continue;
            if (corridor_near_uncertain_edge(left_x, right_x, cfg_.roi_x1, cfg_.roi_x2,
                                             cfg_.red_uncertain_edge_px)) continue;
            const bool low_confidence_red = std::any_of(safe_reds.begin(), safe_reds.end(), [&](const Cone& red) {
                return red.confidence < cfg_.red_confidence_threshold;
            });
            if (low_confidence_red || corridor_has_red_blocker(safe_reds, left_x, right_x, row_y,
                                         cfg_.red_corridor_margin, cfg_.red_block_below)) continue;

            const int width = static_cast<int>(width64);
            const int center_x = static_cast<int>((static_cast<int64_t>(left_x) + right_x) / 2);
            const int64_t delta = static_cast<int64_t>(center_x) - image_center;
            const int64_t distance = delta < 0 ? -delta : delta;
            if (distance < row_best_distance ||
                (distance == row_best_distance && width > row_best_width)) {
                row_target_x = center_x;
                row_best_width = width;
                row_best_distance = distance;
            }
        }
        if (row_target_x >= 0) {
            target_x = row_target_x;
            break;
        }
    }
    if (target_x < 0) {
        // All candidate corridors are blocked or visually uncertain: fail closed.
        drive_.stop();
        return;
    }
    pid_.Target = cfg_.img_center(); pid_.Actual = target_x; PID_Update(&pid_);
    float yaw = target_heading_;
    const bool yaw_ok = read_yaw(yaw, now, false);
    const int steer = compose_steer(pid_.Out, yaw_ok ? yaw : target_heading_, target_heading_, make_steering_params());
    drive_.command(yaw_ok ? cfg_.speed_run : cfg_.speed_approach, steer);
}

void MissionController::process_camp(int64_t now, const cv::Mat& camp_mask) {
    CampDetectParams cp; cp.min_ratio = cfg_.camp_min_ratio; cp.roi_x1 = cfg_.roi_x1; cp.roi_y1 = cfg_.roi_y1;
    const CampResult camp = camp_detector_.detect(camp_mask, cp);
    if (!camp.found || camp.center_y < cfg_.camp_min_global_y) {
        ++camp_lost_frames_;
        stop_frames_ = 0;
        if (camp_lost_frames_ <= cfg_.camp_lost_grace_frames) drive_.command(cfg_.speed_approach, cfg_.servo_mid);
        else drive_.stop();
        return;
    }
    camp_lost_frames_ = 0;
    pid_.Target = cfg_.img_center(); pid_.Actual = camp.center_x; PID_Update(&pid_);
    float yaw = target_heading_;
    const bool yaw_ok = read_yaw(yaw, now, false);
    const int steer = compose_steer(pid_.Out, yaw_ok ? yaw : target_heading_, target_heading_, make_steering_params());
    const int motor = (camp.ratio >= cfg_.camp_slow_ratio || camp.ratio > cfg_.camp_max_ratio)
        ? cfg_.speed_approach : cfg_.speed_run;
    drive_.command(motor, steer);

    const int64_t center_error = static_cast<int64_t>(camp.center_x) - cfg_.img_center();
    const int64_t abs_center_error = center_error < 0 ? -center_error : center_error;
    const bool centered = abs_center_error <= cfg_.stop_center_tol;
    const bool close_enough_to_bypass_center = camp.ratio >= cfg_.stop_area_ratio * cfg_.camp_center_bypass_factor;
    if ((centered || close_enough_to_bypass_center) && camp.ratio >= cfg_.stop_area_ratio) ++stop_frames_;
    else stop_frames_ = 0;
    if (stop_frames_ >= cfg_.stop_frames_need) {
        drive_.stop();
        fsm_.on_event(EventType::CAMP_STOPPED, now);
        display_.show_status(std::string("抵达营地，停车确认帧数=") + std::to_string(stop_frames_));
    }
}

bool MissionController::step() {
    if (!initialized_ || fsm_.is_terminal()) return false;
    const int64_t now = now_ms();
    if (!drive_.healthy()) {
        fsm_.on_event(EventType::SENSOR_FAILURE, now);
        std::fprintf(stderr, "[fail-safe] PWM controller lost readiness\n");
        drive_.stop();
        return false;
    }
    update_state_entry(now);
    State st = fsm_.state();
    if (st == State::TO_STATION && now - state_enter_ms_ > cfg_.station_leg_timeout_ms) {
        fsm_.on_event(EventType::TIMEOUT, now); drive_.stop(); return false;
    }
    if (st == State::ALIGN && now - state_enter_ms_ > cfg_.align_timeout_ms) {
        fsm_.on_event(EventType::TIMEOUT, now); drive_.stop(); std::fprintf(stderr, "[fail-safe] alignment timeout\n"); return false;
    }
    if ((st == State::CROSSING || st == State::TO_CAMP) && track_start_ms_ > 0 && now - track_start_ms_ > cfg_.track_timeout_ms) {
        fsm_.on_event(EventType::TIMEOUT, now); drive_.stop(); std::fprintf(stderr, "[fail-safe] crossing/camp tracking timeout\n"); return false;
    }
    if (fsm_.start_ts_ms() > 0 && now - fsm_.start_ts_ms() > cfg_.mission_timeout_ms) {
        fsm_.on_event(EventType::TIMEOUT, now); drive_.stop(); std::fprintf(stderr, "[fail-safe] mission timeout\n"); return false;
    }

    const bool moving_state = st == State::TO_STATION || st == State::ALIGN || st == State::CROSSING || st == State::TO_CAMP;
    const std::vector<Cone>* blues = nullptr;
    const std::vector<Cone>* reds = nullptr;
    cv::Mat blue_roi, red_roi;
    if (st == State::IDLE || moving_state) {
        if (!acquire_frame(now)) return !fsm_.is_terminal();
        if (!vision_.process(frame_, moving_state)) {
            drive_.stop(); fsm_.on_event(EventType::SENSOR_FAILURE, now); return false;
        }
        if (moving_state) {
            if (!frame_quality_ok(now)) return !fsm_.is_terminal();
            const cv::Rect roi(cfg_.roi_x1, cfg_.roi_y1, cfg_.roi_w(), cfg_.roi_h());
            blue_roi = vision_.blue_mask()(roi);
            red_roi = vision_.red_mask()(roi);
            blues = &blue_detector_.detect(blue_roi, "blue", cfg_.min_area_blue);
            reds = &red_detector_.detect(red_roi, "red", cfg_.min_area_red);
            if (!check_proximity(now, *blues, *reds)) return !fsm_.is_terminal();
        }
    }

    switch (st) {
        case State::IDLE: {
            const float br = vision_.blue_ratio(), rr = vision_.red_ratio();
            int candidate = 0;
            if (br >= cfg_.color_card_threshold && br > rr) candidate = 1;
            else if (rr >= cfg_.color_card_threshold && rr > br) candidate = 2;
            if (candidate == 0) { color_candidate_ = 0; stable_color_frames_ = 0; break; }
            if (candidate == color_candidate_) ++stable_color_frames_;
            else { color_candidate_ = candidate; stable_color_frames_ = 1; }
            if (stable_color_frames_ >= cfg_.color_stable_frames) {
                route_is_blue_ = candidate == 1;
                selected_route_ = route_is_blue_ ? cfg_.blue_route : cfg_.red_route;
                fsm_.on_event(route_is_blue_ ? EventType::COLOR_BLUE : EventType::COLOR_RED, now);
                char message[128];
                std::snprintf(message, sizeof(message), "识别%s色卡，将前往路线%s", route_is_blue_ ? "蓝" : "红", selected_route_.c_str());
                display_.show_status(message);
            }
            break;
        }
        case State::ANNOUNCE: {
            drive_.stop();
            const std::string& announcement = route_is_blue_ ? cfg_.tts_a : cfg_.tts_b;
            display_.show_message(announcement);
            const bool ok = voice_.speak(announcement);
            if (!ok) {
                std::fprintf(stderr, "[fail-safe] voice announcement failed; launch inhibited\n");
                fsm_.on_event(EventType::ANNOUNCE_FAILED, now);
                drive_.stop();
            } else fsm_.on_event(EventType::ANNOUNCE_DONE, now_ms());
            break;
        }
        case State::READY:
            drive_.stop();
            fsm_.on_event(EventType::START, now);
            display_.show_status(std::string("计时开始；路线") + selected_route_);
            break;
        case State::TO_STATION: {
            const ActionSeq a = action_seq();
            const double scale = voltage_coefficient();
            const int64_t leg1 = static_cast<int64_t>(a.duration1_ms * scale);
            const int64_t leg2 = static_cast<int64_t>(a.duration2_ms * scale);
            const int64_t elapsed = now - state_enter_ms_;
            if (elapsed < leg1) drive_.command(cfg_.station_motor_speed, a.servo1);
            else if (elapsed < leg1 + leg2) drive_.command(cfg_.station_motor_speed, a.servo2);
            else { drive_.stop(); fsm_.on_event(EventType::ARRIVE_STATION, now); }
            break;
        }
        case State::STATION_STAY:
            drive_.stop();
            if (now - state_enter_ms_ >= cfg_.station_stay_ms) fsm_.on_event(EventType::STATION_STAY_DONE, now);
            break;
        case State::ALIGN: {
            if (!imu_ready_) {
                if (!imu_degraded_logged_) { fsm_.on_event(EventType::IMU_DEGRADED, now); imu_degraded_logged_ = true; }
                drive_.stop();
                fsm_.on_event(EventType::ALIGN_DONE, now);
                break;
            }
            float yaw = 0.0f;
            if (!read_yaw(yaw, now, true)) break;
            align_imu_misses_ = 0;
            const float err = normalize_angle(yaw - target_heading_);
            if (std::abs(err) > cfg_.head_protect) {
                fsm_.on_event(EventType::SAFETY_STOP, now); drive_.stop();
                std::fprintf(stderr, "[fail-safe] heading error %.1f deg exceeds limit\n", err);
                break;
            }
            if (std::abs(err) <= cfg_.align_tolerance_deg) ++align_stable_frames_;
            else align_stable_frames_ = 0;
            if (align_stable_frames_ >= cfg_.align_stable_frames) {
                drive_.stop(); fsm_.on_event(EventType::ALIGN_DONE, now);
            } else {
                drive_.command(cfg_.align_motor_speed, compose_align_steer(err, cfg_.align_p_gain, make_steering_params()));
            }
            break;
        }
        case State::CROSSING: {
            const cv::Mat camp_mask = route_is_blue_ ? blue_roi : red_roi;
            process_crossing(now, *blues, *reds, camp_mask);
            break;
        }
        case State::TO_CAMP: {
            const cv::Mat camp_mask = route_is_blue_ ? blue_roi : red_roi;
            process_camp(now, camp_mask);
            break;
        }
        case State::FINISHED:
        case State::FAILED:
            drive_.stop();
            return false;
    }
    return !fsm_.is_terminal();
}

void MissionController::abort() {
    if (!fsm_.is_terminal()) fsm_.on_event(EventType::OPERATOR_ABORT, now_ms());
    drive_.stop();
}

bool MissionController::save_log() const {
    try {
        const std::filesystem::path p(cfg_.event_log_path);
        if (!p.parent_path().empty()) std::filesystem::create_directories(p.parent_path());
        std::ofstream out(p);
        if (!out) return false;
        out << "elapsed_ms,event,penalty_s\n";
        for (const auto& r : fsm_.log().records()) out << r.ts_ms << ',' << EventLog::name(r.type) << ',' << r.penalty_s << '\n';
        return static_cast<bool>(out);
    } catch (...) { return false; }
}

void MissionController::shutdown() {
    if (shutdown_) return;
    drive_.stop();
    camera_.release();
    imu_.close();
    drive_.shutdown();
    if (!save_log()) std::fprintf(stderr, "[warn] failed to write event log: %s\n", cfg_.event_log_path.c_str());
    shutdown_ = true;
    if (fsm_.state() == State::FINISHED) {
        std::printf("[FINISHED] elapsed=%.3fs penalty=%ds events=%d\n", elapsed_ms() / 1000.0,
                    fsm_.log().total_penalty_s(), fsm_.log().count());
    } else if (fsm_.state() == State::FAILED) {
        std::printf("[FAILED] elapsed=%.3fs penalty=%ds events=%d\n", elapsed_ms() / 1000.0,
                    fsm_.log().total_penalty_s(), fsm_.log().count());
    } else {
        std::printf("[STOPPED] state=%s elapsed=%.3fs events=%d\n", StateMachine::state_name(fsm_.state()),
                    elapsed_ms() / 1000.0, fsm_.log().count());
    }
}

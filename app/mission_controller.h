#ifndef CROSSFIRE_MISSION_CONTROLLER_H
#define CROSSFIRE_MISSION_CONTROLLER_H

#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "../config/config.h"
#include "../hal/hal.h"
#include "../decision/state_machine.h"
#include "../perception/vision_pipeline.h"
#include "../perception/cone_detect.h"
#include "../perception/camp_detect.h"
#include "../perception/cone_logic.h"
#include "../control/drive_controller.h"
#include "../control/steering.h"
#include "../drivers/pid.h"

class MissionController {
public:
    MissionController(const Config& cfg, ICamera& camera, IImu& imu, IPwm& pwm, IVoice& voice, IDisplay& display);
    bool initialize();
    bool step(); // advances the mission by one non-blocking control iteration
    void abort();
    void shutdown();
    State state() const { return fsm_.state(); }
    const EventLog& log() const { return fsm_.log(); }
    int64_t elapsed_ms() const;
private:
    struct ActionSeq {
        int servo1, servo2, duration1_ms, duration2_ms;
        int single_servo, no_safe_servo;
        float heading_offset_deg;
    };
    static int64_t now_ms();
    static DriveParams make_drive_params(const Config& c);
    ConeDetectParams make_cone_params() const;
    SteeringParams make_steering_params() const;
    VisionPipelineParams make_vision_params() const;
    ActionSeq action_seq() const;
    float voltage_coefficient() const;
    bool open_camera();
    bool acquire_frame(int64_t now);
    bool frame_quality_ok(int64_t now);
    bool check_proximity(int64_t now, const std::vector<Cone>& blues,
                         const std::vector<Cone>& reds);
    bool check_red_hit(const std::vector<Cone>& reds) const;
    bool check_blue_hit(const std::vector<Cone>& blues) const;
    void update_state_entry(int64_t now);
    void process_crossing(int64_t now, const std::vector<Cone>& blues,
                          const std::vector<Cone>& reds, const cv::Mat& camp_mask);
    void process_camp(int64_t now, const cv::Mat& camp_mask);
    bool read_yaw(float& yaw, int64_t now, bool fail_on_repeated_miss);
    bool save_log() const;

    const Config& cfg_;
    ICamera& camera_;
    IImu& imu_;
    IVoice& voice_;
    IDisplay& display_;
    DriveController drive_;
    StateMachine fsm_;
    VisionPipeline vision_;
    ConeDetector blue_detector_;
    ConeDetector red_detector_;
    ConeRowGrouper row_grouper_;
    RedConeTracker red_tracker_; // persistent workspace avoids per-frame row/vector allocation
    CampDetector camp_detector_;
    PID_t pid_{};
    cv::Mat capture_frame_; // native UVC frame; only resized if driver ignores requested mode
    cv::Mat frame_;         // fixed processing size matching the scaled configuration

    bool initialized_ = false, shutdown_ = false;
    bool imu_ready_ = false, imu_degraded_logged_ = false;
    bool route_is_blue_ = true;
    bool has_last_state_ = false;
    State last_state_ = State::IDLE;
    std::string selected_route_ = "A";
    float initial_yaw_ = 0.0f, target_heading_ = 0.0f;
    float voltage_scale_ = 1.0f; // computed once in initialize() from the VOLT env
    int64_t state_enter_ms_ = 0, track_start_ms_ = 0;
    int64_t last_camera_reopen_ms_ = 0;
    int camera_misses_ = 0, camera_reopen_count_ = 0;
    int stable_color_frames_ = 0, color_candidate_ = 0;
    int no_cone_frames_ = 0, camp_lost_frames_ = 0, stop_frames_ = 0;
    int align_stable_frames_ = 0, align_imu_misses_ = 0, runtime_imu_misses_ = 0;
    int bad_frame_count_ = 0;
    double last_brightness_ = -1.0;
    bool vision_degraded_latched_ = false;
    bool blue_hit_latched_ = false, fence_hit_latched_ = false;
    int64_t last_blue_event_ms_ = -1000000, last_fence_event_ms_ = -1000000;
};

#endif

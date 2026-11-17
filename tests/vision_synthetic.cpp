#include "../perception/vision_pipeline.h"
#include "../perception/cone_detect.h"
#include "../config/config.h"
#include <cstdio>
#include <cmath>
#include <vector>

static int pass = 0, fail = 0;
#define CHECK(c, m) do { if (c) ++pass; else { ++fail; std::fprintf(stderr, "FAIL: %s\n", m); } } while (0)

static VisionPipelineParams make_params(const Config& c) {
    VisionPipelineParams p;
    p.colors.blue = c.blue; p.colors.red1 = c.red1; p.colors.red2 = c.red2;
    p.morphology_kernel_size = c.morphology_kernel_size;
    p.morphology_iterations = c.morphology_iterations;
    p.dark_threshold_v = c.dark_threshold_v;
    return p;
}

int main() {
    Config cfg;
    VisionPipeline vision;
    vision.configure(make_params(cfg));

    cv::Mat invalid_gray(480, 640, CV_8UC1, cv::Scalar(0));
    CHECK(!vision.process(invalid_gray, false), "reject non-BGR input without calling cvtColor");
    cv::Mat invalid_color_mask(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    ConeDetector guard_detector;
    CHECK(guard_detector.detect(invalid_color_mask, "red", 1.0f).empty(),
          "reject multi-channel mask before findContours");

    cv::Mat blue_card(480, 640, CV_8UC3, cv::Scalar(255, 0, 0));
    CHECK(vision.process(blue_card, false), "blue card frame processes");
    CHECK(vision.blue_ratio() >= 0.95f, "blue card ratio high");

    cv::Mat red_card(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
    CHECK(vision.process(red_card, false), "red card frame processes");
    CHECK(vision.red_ratio() >= 0.95f, "red card ratio high");

    // Randomized cone position is represented by a synthetic trapezoid, not a fixed map coordinate.
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(110, 110, 110));
    std::vector<cv::Point> polygon = {{285, 315}, {325, 315}, {350, 430}, {260, 430}};
    cv::fillConvexPoly(frame, polygon, cv::Scalar(0, 0, 255));
    CHECK(vision.process(frame, true), "cone frame processes");
    ConeDetectParams dp;
    dp.roi_x1 = cfg.roi_x1; dp.roi_y1 = cfg.roi_y1; dp.roi_w = cfg.roi_w(); dp.roi_h = cfg.roi_h();
    dp.min_cont_w = cfg.min_cont_w; dp.min_cont_h = cfg.min_cont_h; dp.aspect_min = cfg.aspect_min;
    dp.aspect_max = cfg.aspect_max; dp.fill_min = cfg.fill_min; dp.fill_max = cfg.fill_max;
    dp.stop_area_ratio = cfg.stop_area_ratio; dp.top_exclude_global_y = cfg.top_exclude_global_y;
    ConeDetector detector(dp);
    detector.set_output_offset(cfg.roi_x1, cfg.roi_y1);
    cv::Mat roi = vision.red_mask()(cv::Rect(cfg.roi_x1, cfg.roi_y1, cfg.roi_w(), cfg.roi_h()));
    const auto& cones = detector.detect(roi, "red", cfg.min_area_red);
    CHECK(!cones.empty(), "random red cone detected");
    if (!cones.empty()) CHECK(cones.front().bottom_y >= cfg.red_hit_y, "near-zone bottom coordinate is global");

    cv::Mat occluded(480, 640, CV_8UC3, cv::Scalar(110, 110, 110));
    CHECK(vision.process(occluded, true), "occluded/low-signal frame still processes");
    CHECK(vision.red_ratio() < 0.01f && vision.blue_ratio() < 0.01f, "no-color frame produces no false card");

    cv::Mat bright(480, 640, CV_8UC3, cv::Scalar(245, 245, 245));
    const float before = vision.brightness();
    CHECK(vision.process(bright, false), "brightness-jump frame processes");
    CHECK(std::fabs(vision.brightness() - before) > cfg.max_brightness_delta, "brightness jump exceeds configured anomaly threshold");

    std::printf("OpenCV 合成视觉场景: %d 通过, %d 失败\n", pass, fail);
    return fail == 0 ? 0 : 1;
}

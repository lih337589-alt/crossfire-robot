#include "../config/config.h"
#include "../perception/vision_pipeline.h"
#include "../perception/cone_detect.h"
#include "../perception/cone_logic.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <numeric>
#include <cstdlib>
#include <string>
#include <vector>

namespace {
VisionPipelineParams make_vision_params(const Config& c) {
    VisionPipelineParams p;
    p.colors.blue = c.blue;
    p.colors.red1 = c.red1;
    p.colors.red2 = c.red2;
    p.morphology_kernel_size = c.morphology_kernel_size;
    p.morphology_iterations = c.morphology_iterations;
    p.dark_threshold_v = c.dark_threshold_v;
    return p;
}

ConeDetectParams make_cone_params(const Config& c, bool suppress_large_blob) {
    ConeDetectParams p;
    p.min_cont_w = c.min_cont_w;
    p.min_cont_h = c.min_cont_h;
    p.aspect_min = c.aspect_min;
    p.aspect_max = c.aspect_max;
    p.fill_min = c.fill_min;
    p.fill_max = c.fill_max;
    p.stop_area_ratio = c.stop_area_ratio;
    p.suppress_large_blob = suppress_large_blob;
    p.roi_x1 = c.roi_x1;
    p.roi_y1 = c.roi_y1;
    p.roi_w = c.roi_w();
    p.roi_h = c.roi_h();
    p.top_exclude_global_y = c.top_exclude_global_y;
    p.edge_margin_px = c.cone_edge_margin_px;
    p.shape_min_height = c.cone_shape_min_height;
    p.bottom_band_min_px = c.cone_bottom_band_min_px;
    p.bottom_band_max_px = c.cone_bottom_band_max_px;
    p.bottom_band_height_divisor = c.cone_bottom_band_height_divisor;
    p.bottom_min_width = c.cone_bottom_min_width;
    p.mid_min_width = c.cone_mid_min_width;
    p.bottom_width_ratio = c.cone_bottom_width_ratio;
    p.mid_width_ratio = c.cone_mid_width_ratio;
    return p;
}

void draw_trapezoid(cv::Mat& frame, int cx, int top_y, int bottom_y, int top_w, int bottom_w,
                    const cv::Scalar& color) {
    const std::vector<cv::Point> points = {
        {cx - top_w / 2, top_y}, {cx + top_w / 2, top_y},
        {cx + bottom_w / 2, bottom_y}, {cx - bottom_w / 2, bottom_y}
    };
    cv::fillConvexPoly(frame, points, color);
}
}

int main(int argc, char** argv) {
    const std::string config_path = argc > 1 ? argv[1] : "config/config.ini";
    Config cfg;
    if (!cfg.load(config_path)) {
        std::fprintf(stderr, "cannot load config: %s\n", config_path.c_str());
        return 2;
    }

    cv::Mat frame(cfg.cam_h, cfg.cam_w, CV_8UC3, cv::Scalar(100, 100, 100));
    // Deterministic synthetic workload. This measures code throughput only, not real-world
    // recognition accuracy or a field-camera latency guarantee.
    const int y0 = cfg.roi_y1 + cfg.roi_h() / 3;
    const int y1 = std::min(cfg.cam_h - 2, cfg.roi_y2 - 2);
    const int x1 = cfg.roi_x1 + cfg.roi_w() / 4;
    const int x2 = cfg.roi_x1 + cfg.roi_w() / 2;
    const int x3 = cfg.roi_x1 + (cfg.roi_w() * 3) / 4;
    draw_trapezoid(frame, x1, y0, y1, 20, 56, cv::Scalar(255, 0, 0));
    draw_trapezoid(frame, x2, y0, y1, 20, 56, cv::Scalar(255, 0, 0));
    draw_trapezoid(frame, x3, y0, y1, 20, 56, cv::Scalar(255, 0, 0));
    draw_trapezoid(frame, cfg.roi_x1 + cfg.roi_w() / 3, y0 + 10, y1 - 20, 18, 50,
                   cv::Scalar(0, 0, 255));

    VisionPipeline vision;
    vision.configure(make_vision_params(cfg));
    ConeDetector blue_detector(make_cone_params(cfg, true));
    ConeDetector red_detector(make_cone_params(cfg, false));
    blue_detector.set_output_offset(cfg.roi_x1, cfg.roi_y1);
    red_detector.set_output_offset(cfg.roi_x1, cfg.roi_y1);
    ConeRowGrouper row_grouper;
    const cv::Rect roi_rect(cfg.roi_x1, cfg.roi_y1, cfg.roi_w(), cfg.roi_h());

    constexpr int kWarmupFrames = 300;
    constexpr int kMeasuredFrames = 3000;
    for (int i = 0; i < kWarmupFrames; ++i) {
        if (!vision.process(frame, true)) return 3;
        const cv::Mat blue_roi = vision.blue_mask()(roi_rect);
        const cv::Mat red_roi = vision.red_mask()(roi_rect);
        const auto& blues = blue_detector.detect(blue_roi, "blue", cfg.min_area_blue);
        (void)red_detector.detect(red_roi, "red", cfg.min_area_red);
        (void)row_grouper.group(blues, cfg.row_tolerance);
    }

    std::vector<double> us;
    us.reserve(kMeasuredFrames);
    for (int i = 0; i < kMeasuredFrames; ++i) {
        const auto begin = std::chrono::steady_clock::now();
        if (!vision.process(frame, true)) return 3;
        const cv::Mat blue_roi = vision.blue_mask()(roi_rect);
        const cv::Mat red_roi = vision.red_mask()(roi_rect);
        const auto& blues = blue_detector.detect(blue_roi, "blue", cfg.min_area_blue);
        (void)red_detector.detect(red_roi, "red", cfg.min_area_red);
        (void)row_grouper.group(blues, cfg.row_tolerance);
        const auto end = std::chrono::steady_clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
    }

    const double total = std::accumulate(us.begin(), us.end(), 0.0);
    std::sort(us.begin(), us.end());
    const size_t p95_index = (us.size() - 1) * 95 / 100;
    std::printf("Synthetic vision benchmark (%dx%d, %d measured frames)\n", cfg.cam_w, cfg.cam_h, kMeasuredFrames);
    std::printf("mean=%.1f us, p50=%.1f us, p95=%.1f us, max=%.1f us\n",
                total / static_cast<double>(kMeasuredFrames), us[us.size() / 2],
                us[p95_index], us.back());
    std::puts("NOTE: synthetic workload only; compare builds on the same target, compiler flags, camera profile and thermal state.");
    return 0;
}

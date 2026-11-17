#ifndef CROSSFIRE_VISION_PIPELINE_H
#define CROSSFIRE_VISION_PIPELINE_H

#include <opencv2/opencv.hpp>
#include "color_detect.h"

struct VisionPipelineParams {
    ColorRatioParams colors{};
    int morphology_kernel_size = 5;
    int morphology_iterations = 1;
    int dark_threshold_v = 70;
};

// Owns reusable frame-sized buffers. ROI access returns OpenCV Mat headers/views, not pixel copies.
class VisionPipeline {
public:
    void configure(const VisionPipelineParams& params);
    bool process(const cv::Mat& bgr, bool clean_masks);
    float blue_ratio() const;
    float red_ratio() const;
    float brightness() const { return brightness_; }
    const cv::Mat& hsv() const { return hsv_; }
    const cv::Mat& blue_mask() const { return blue_mask_; }
    const cv::Mat& red_mask() const { return red_mask_; }
    const cv::Mat& dark_mask() const { return dark_mask_; }
    bool fence_contact_proxy(const cv::Rect& roi, int edge_strip_px,
                             int bottom_band_px, float dark_ratio_threshold,
                             float opposite_edge_max_ratio) const;
private:
    VisionPipelineParams p_{};
    cv::Mat hsv_, blue_mask_, red1_mask_, red2_mask_, red_mask_, dark_mask_, kernel_;
    float brightness_ = 0.0f;
    bool configured_ = false;
};

#endif
